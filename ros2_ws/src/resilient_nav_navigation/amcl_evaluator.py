"""Evaluation-only AMCL metrics in the frozen Phase 9 map frame."""

from __future__ import annotations

import json
import math
from collections import deque
from dataclasses import asdict
from pathlib import Path

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from resilient_nav_slam.slam_evaluator import (
    SE2Alignment,
    TimedPose,
    evaluate_persisted_map_localization,
)


AMCL_POSE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _yaw_from_quaternion(quaternion) -> float:
    """Return planar yaw while rejecting malformed orientations."""
    norm = (
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError('invalid quaternion')
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _timed_pose(header, pose, expected_frame: str) -> TimedPose | None:
    if header.frame_id != expected_frame:
        return None
    stamp = header.stamp.sec + header.stamp.nanosec / 1_000_000_000.0
    values = (stamp, pose.position.x, pose.position.y)
    if not all(math.isfinite(value) for value in values):
        return None
    try:
        yaw = _yaw_from_quaternion(pose.orientation)
    except ValueError:
        return None
    return TimedPose(stamp_sec=stamp, x=pose.position.x, y=pose.position.y, yaw=yaw)


class AmclEvaluator(Node):
    """Keep Ground Truth in an evaluator process outside the localization runtime."""

    def __init__(self) -> None:
        super().__init__('phase10_amcl_evaluator')
        self.declare_parameter('output_path', '')
        self.declare_parameter('map_to_odom_x', 0.0)
        self.declare_parameter('map_to_odom_y', 0.0)
        self.declare_parameter('map_to_odom_yaw', 0.0)
        self.declare_parameter('min_samples', 10)
        self.declare_parameter('max_alignment_delta_sec', 0.05)
        self._truth = deque(maxlen=5000)
        self._healthy_ekf = deque(maxlen=5000)
        self._amcl = deque(maxlen=5000)
        self.create_subscription(
            PoseStamped, '/evaluation/ground_truth_pose', self._on_truth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, '/odometry/filtered', self._on_healthy_ekf,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, AMCL_POSE_QOS
        )
        self.create_timer(1.0, self._evaluate)

    def _on_truth(self, message: PoseStamped) -> None:
        sample = _timed_pose(message.header, message.pose, 'odom')
        if sample is not None:
            self._truth.append(sample)

    def _on_healthy_ekf(self, message: Odometry) -> None:
        if message.child_frame_id != 'base_footprint':
            return
        sample = _timed_pose(message.header, message.pose.pose, 'odom')
        if sample is not None:
            self._healthy_ekf.append(sample)

    def _on_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        sample = _timed_pose(message.header, message.pose.pose, 'map')
        if sample is not None:
            self._amcl.append(sample)

    def _evaluate(self) -> None:
        minimum = int(self.get_parameter('min_samples').value)
        if min(len(self._truth), len(self._healthy_ekf), len(self._amcl)) < minimum:
            return
        alignment = SE2Alignment(
            x=float(self.get_parameter('map_to_odom_x').value),
            y=float(self.get_parameter('map_to_odom_y').value),
            yaw=float(self.get_parameter('map_to_odom_yaw').value),
        )
        try:
            metrics = evaluate_persisted_map_localization(
                self._truth,
                self._healthy_ekf,
                self._amcl,
                map_to_odom=alignment,
                max_alignment_delta_sec=float(
                    self.get_parameter('max_alignment_delta_sec').value
                ),
                min_samples=minimum,
            )
        except ValueError as error:
            self.get_logger().warning(f'AMCL evaluation not ready: {error}')
            return
        output = {
            'outcome': 'MEASURED',
            'evaluation_mode': 'persisted_map_localization_absolute',
            'fixed_map_to_odom': asdict(metrics.map_to_odom),
            'amcl_absolute_localization_metrics': asdict(metrics.slam),
            'healthy_ekf_reference_metrics': asdict(metrics.healthy_ekf),
        }
        output_path = str(self.get_parameter('output_path').value)
        if output_path:
            path = Path(output_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(output, indent=2, sort_keys=True) + '\n')
        self.get_logger().info(json.dumps(output, sort_keys=True))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AmclEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
