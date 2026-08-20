"""Evaluation-only ROS boundary for Phase 9 SLAM trajectory metrics."""

import json
from collections import deque
from pathlib import Path

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from resilient_nav_fusion.localization_evaluator import EvaluationError
from resilient_nav_fusion.localization_evaluator_node import _pose_stamped_to_timed_pose

from .slam_evaluator import evaluate_slam_trajectory


class SlamEvaluator(Node):
    """Read only evaluation topics and persist structured post-hoc metrics."""

    def __init__(self):
        super().__init__('slam_evaluator')
        self.declare_parameter('ground_truth_topic', '/evaluation/ground_truth_pose')
        self.declare_parameter('slam_pose_topic', '/evaluation/slam_pose')
        self.declare_parameter('map_metrics_topic', '/evaluation/slam_map_metrics')
        self.declare_parameter('output_topic', '/evaluation/slam_metrics')
        self.declare_parameter('output_path', '')
        self.declare_parameter('run_label', 'unspecified')
        self.declare_parameter('ground_truth_frame', 'odom')
        self.declare_parameter('slam_frame', 'map')
        self.declare_parameter('max_alignment_delta_sec', 0.05)
        self.declare_parameter('min_samples', 3)
        self.declare_parameter('max_samples', 5000)
        self.declare_parameter('evaluation_period_sec', 1.0)
        max_samples = int(self.get_parameter('max_samples').value)
        self._truth = deque(maxlen=max_samples)
        self._slam_pose = deque(maxlen=max_samples)
        self._map_metrics: dict[str, object] | None = None
        self._first_truth_stamp: float | None = None
        self._first_slam_stamp: float | None = None
        self._publisher = self.create_publisher(
            String, str(self.get_parameter('output_topic').value), 10
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('ground_truth_topic').value),
            self._on_truth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('slam_pose_topic').value),
            self._on_slam_pose,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('map_metrics_topic').value),
            self._on_map_metrics,
            10,
        )
        self.create_timer(
            float(self.get_parameter('evaluation_period_sec').value), self._evaluate
        )

    def _on_truth(self, message: PoseStamped) -> None:
        sample = _pose_stamped_to_timed_pose(
            message, str(self.get_parameter('ground_truth_frame').value)
        )
        self._append(self._truth, sample)
        if sample is not None and self._first_truth_stamp is None:
            self._first_truth_stamp = sample.stamp_sec

    def _on_slam_pose(self, message: PoseStamped) -> None:
        sample = _pose_stamped_to_timed_pose(
            message, str(self.get_parameter('slam_frame').value)
        )
        self._append(self._slam_pose, sample)
        if sample is not None and self._first_slam_stamp is None:
            self._first_slam_stamp = sample.stamp_sec

    def _on_map_metrics(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().warning('suppressed invalid evaluation map metadata')
            return
        if not isinstance(payload, dict):
            self.get_logger().warning('suppressed non-object evaluation map metadata')
            return
        self._map_metrics = payload

    def _append(self, samples, sample) -> None:
        if sample is None:
            self.get_logger().warning('suppressed invalid evaluation input')
            return
        if samples and sample.stamp_sec <= samples[-1].stamp_sec:
            return
        samples.append(sample)

    def _evaluate(self) -> None:
        try:
            metrics = evaluate_slam_trajectory(
                self._truth,
                self._slam_pose,
                max_alignment_delta_sec=float(
                    self.get_parameter('max_alignment_delta_sec').value
                ),
                min_samples=int(self.get_parameter('min_samples').value),
            )
        except EvaluationError as error:
            self.get_logger().debug(f'evaluation withheld: {error}')
            return
        output = metrics.to_dict()
        output.update({
            'run_label': str(self.get_parameter('run_label').value),
            'map_metrics': self._map_metrics,
            'startup_time_sec': _startup_time(
                self._first_truth_stamp, self._first_slam_stamp
            ),
            'time_to_first_valid_map_to_odom_sec': _startup_time(
                self._first_truth_stamp, self._first_slam_stamp
            ),
            'relocalization_position_rmse': metrics.position_rmse,
            'relocalization_yaw_rmse': metrics.yaw_rmse,
        })
        message = String()
        message.data = json.dumps(output, sort_keys=True, separators=(',', ':'))
        self._publisher.publish(message)
        self._write_output(output)

    def _write_output(self, output: dict[str, object]) -> None:
        output_path = str(self.get_parameter('output_path').value)
        if not output_path:
            return
        path = Path(output_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + '.tmp')
        temporary.write_text(
            json.dumps(output, sort_keys=True, indent=2) + '\n', encoding='utf-8'
        )
        temporary.replace(path)


def _startup_time(
    first_truth_stamp: float | None, first_slam_stamp: float | None
) -> float | None:
    """Report first valid map-to-odom availability relative to evaluation start."""
    if first_truth_stamp is None or first_slam_stamp is None:
        return None
    return max(0.0, first_slam_stamp - first_truth_stamp)


def main(args=None):
    """Run the read-only Phase 9 evaluator."""
    rclpy.init(args=args)
    node = SlamEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
