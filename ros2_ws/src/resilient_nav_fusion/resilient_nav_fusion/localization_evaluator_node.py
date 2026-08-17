"""ROS boundary for the evaluation-only Phase 8 localization metrics."""

import json
from collections import deque
from math import atan2, isfinite

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from .localization_evaluator import (
    EvaluationError,
    TimedPose,
    evaluate_alignment_sensitivity,
    evaluate_trajectories,
)


class LocalizationEvaluator(Node):
    """Read only truth/fixed/adaptive trajectories and publish evaluation JSON."""

    def __init__(self):
        super().__init__('localization_evaluator')
        self.declare_parameter('ground_truth_topic', '/evaluation/ground_truth_pose')
        self.declare_parameter('fixed_topic', '/odometry/faulted')
        self.declare_parameter('adaptive_topic', '/odometry/adaptive')
        self.declare_parameter('output_topic', '/evaluation/localization_metrics')
        self.declare_parameter('expected_frame', 'odom')
        self.declare_parameter('expected_child_frame', 'base_footprint')
        self.declare_parameter('max_alignment_delta_sec', 0.05)
        self.declare_parameter(
            'alignment_sweep_windows_sec', [0.02, 0.03, 0.05]
        )
        self.declare_parameter(
            'alignment_sweep_output_topic', '/evaluation/localization_alignment_sweep'
        )
        self.declare_parameter('min_samples', 3)
        self.declare_parameter('max_samples', 2000)
        self.declare_parameter('evaluation_period_sec', 1.0)
        max_samples = int(self.get_parameter('max_samples').value)
        self._truth: deque[TimedPose] = deque(maxlen=max_samples)
        self._fixed: deque[TimedPose] = deque(maxlen=max_samples)
        self._adaptive: deque[TimedPose] = deque(maxlen=max_samples)
        self._publisher = self.create_publisher(
            String, str(self.get_parameter('output_topic').value), 10
        )
        self._alignment_sweep_publisher = self.create_publisher(
            String,
            str(self.get_parameter('alignment_sweep_output_topic').value),
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('ground_truth_topic').value),
            self._on_truth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('fixed_topic').value),
            self._on_fixed,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('adaptive_topic').value),
            self._on_adaptive,
            qos_profile_sensor_data,
        )
        self.create_timer(
            float(self.get_parameter('evaluation_period_sec').value), self._evaluate
        )

    def _on_truth(self, message: PoseStamped) -> None:
        self._append(self._truth, _pose_stamped_to_timed_pose(message, self._expected_frame()))

    def _on_fixed(self, message: Odometry) -> None:
        self._append(self._fixed, _odometry_to_timed_pose(message, self._expected_frame(), self._expected_child_frame()))

    def _on_adaptive(self, message: Odometry) -> None:
        self._append(self._adaptive, _odometry_to_timed_pose(message, self._expected_frame(), self._expected_child_frame()))

    def _expected_frame(self) -> str:
        return str(self.get_parameter('expected_frame').value)

    def _expected_child_frame(self) -> str:
        return str(self.get_parameter('expected_child_frame').value)

    def _append(self, samples: deque[TimedPose], sample: TimedPose | None) -> None:
        if sample is None:
            self.get_logger().warning('suppressed invalid evaluation input')
            return
        if samples and sample.stamp_sec <= samples[-1].stamp_sec:
            self.get_logger().warning('suppressed non-monotonic evaluation input')
            return
        samples.append(sample)

    def _evaluate(self) -> None:
        try:
            result = evaluate_trajectories(
                self._truth,
                self._fixed,
                self._adaptive,
                max_alignment_delta_sec=float(
                    self.get_parameter('max_alignment_delta_sec').value
                ),
                min_samples=int(self.get_parameter('min_samples').value),
            )
        except EvaluationError as error:
            self.get_logger().debug(f'evaluation withheld: {error}')
            return
        message = String()
        message.data = json.dumps(result.to_dict(), sort_keys=True, separators=(',', ':'))
        self._publisher.publish(message)
        self._publish_alignment_sweep()

    def _publish_alignment_sweep(self) -> None:
        """Optionally publish evaluation-only metrics for declared tolerances."""
        windows = tuple(
            float(value)
            for value in self.get_parameter('alignment_sweep_windows_sec').value
        )
        if not windows:
            return
        try:
            results = evaluate_alignment_sensitivity(
                self._truth,
                self._fixed,
                self._adaptive,
                windows_sec=windows,
                min_samples=int(self.get_parameter('min_samples').value),
            )
        except EvaluationError as error:
            self.get_logger().warning(f'alignment sensitivity withheld: {error}')
            return
        message = String()
        message.data = json.dumps(
            {'windows_sec': results}, sort_keys=True, separators=(',', ':')
        )
        self._alignment_sweep_publisher.publish(message)


def _pose_stamped_to_timed_pose(
    message: PoseStamped, expected_frame: str
) -> TimedPose | None:
    if message.header.frame_id != expected_frame:
        return None
    return _to_timed_pose(message.header.stamp, message.pose.position, message.pose.orientation)


def _odometry_to_timed_pose(
    message: Odometry, expected_frame: str, expected_child_frame: str
) -> TimedPose | None:
    if (
        message.header.frame_id != expected_frame
        or message.child_frame_id != expected_child_frame
    ):
        return None
    return _to_timed_pose(message.header.stamp, message.pose.pose.position, message.pose.pose.orientation)


def _to_timed_pose(stamp, position, orientation) -> TimedPose | None:
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        return None
    values = (position.x, position.y, orientation.x, orientation.y, orientation.z, orientation.w)
    if not all(isfinite(value) for value in values):
        return None
    norm_sq = sum(
        value * value
        for value in (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
    )
    if norm_sq <= 1e-12:
        return None
    yaw = atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )
    return TimedPose(
        stamp_sec=float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0,
        x=float(position.x),
        y=float(position.y),
        yaw=yaw,
    )


def main(args=None):
    """Run the evaluation-only ROS adapter."""
    rclpy.init(args=args)
    node = LocalizationEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
