"""Evaluation-only Gazebo Ground Truth recorder for one completed benchmark run."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from costmap_contract import yaw_from_quaternion


class GroundTruthRecorder(Node):
    """Read `/evaluation/*` only; this node exposes no navigation interface."""

    def __init__(self, output_path: Path) -> None:
        super().__init__('phase10_navigation_benchmark_gt_recorder')
        self.output_path = output_path
        self.samples: list[dict[str, float]] = []
        self.invalid_samples = 0
        self.create_subscription(PoseStamped, '/evaluation/ground_truth_pose', self._on_pose, qos_profile_sensor_data)

    def _on_pose(self, message: PoseStamped) -> None:
        if message.header.frame_id != 'odom':
            self.invalid_samples += 1
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        values = (stamp, message.pose.position.x, message.pose.position.y, message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w)
        if not all(math.isfinite(value) for value in values):
            self.invalid_samples += 1
            return
        yaw = yaw_from_quaternion(message.pose.orientation)
        if not math.isfinite(yaw):
            self.invalid_samples += 1
            return
        if self.samples and stamp <= self.samples[-1]['stamp_sec']:
            self.invalid_samples += 1
            return
        self.samples.append({'stamp_sec': stamp, 'x': message.pose.position.x, 'y': message.pose.position.y, 'yaw': yaw})

    def write(self) -> None:
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path.write_text(json.dumps({'source': '/evaluation/ground_truth_pose', 'frame_id': 'odom', 'sample_count': len(self.samples), 'invalid_sample_count': self.invalid_samples, 'samples': self.samples}, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-path', required=True)
    # launch_ros appends ROS arguments after these application arguments.
    return parser.parse_known_args(argv)[0]


def main(argv: list[str] | None = None) -> int:
    arguments = parse_arguments(argv)
    rclpy.init(args=None)
    node = GroundTruthRecorder(Path(arguments.output_path))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
