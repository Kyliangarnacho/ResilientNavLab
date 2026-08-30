"""Publish one explicit map-frame AMCL initial pose after AMCL is active."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


AMCL_POSE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class InitialPoseHelper(Node):
    """Gate one `/initialpose` message on an active AMCL lifecycle node."""

    def __init__(self) -> None:
        super().__init__('phase10_initial_pose_helper')
        self.initial_pose_seen = False
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._on_amcl_pose,
            AMCL_POSE_QOS,
        )
        self.lifecycle_client = self.create_client(GetState, '/amcl/get_state')

    def _on_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        if message.header.frame_id == 'map':
            self.initial_pose_seen = True

    def amcl_active(self) -> bool:
        if not self.lifecycle_client.service_is_ready():
            return False
        future = self.lifecycle_client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        return bool(
            future.done()
            and future.result() is not None
            and future.result().current_state.id == State.PRIMARY_STATE_ACTIVE
        )


def initial_pose_message(x: float, y: float, yaw: float, node: Node) -> PoseWithCovarianceStamped:
    """Build the fixed two-dimensional covariance used by RViz and automation."""
    message = PoseWithCovarianceStamped()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = 'map'
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    message.pose.covariance[0] = 0.25
    message.pose.covariance[7] = 0.25
    message.pose.covariance[35] = 0.06853891909122467
    return message


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--yaw', type=float, required=True)
    parser.add_argument('--timeout-sec', type=float, default=30.0)
    parser.add_argument('--result-path', required=True)
    # launch_ros appends ROS arguments after this helper's explicit CLI.
    arguments, _ros_arguments = parser.parse_known_args(argv)
    return arguments


def main(argv: list[str] | None = None) -> int:
    args = parse_arguments(argv)
    if not all(math.isfinite(value) for value in (args.x, args.y, args.yaw)):
        raise SystemExit('initial pose values must be finite')
    if not math.isfinite(args.timeout_sec) or args.timeout_sec <= 0.0:
        raise SystemExit('--timeout-sec must be finite and positive')

    result_path = Path(args.result_path)
    result: dict[str, object] = {
        'outcome': 'FAIL',
        'pose': {'frame_id': 'map', 'x': args.x, 'y': args.y, 'yaw': args.yaw},
    }
    rclpy.init(args=argv)
    helper = InitialPoseHelper()
    try:
        deadline = time.monotonic() + args.timeout_sec
        published = False
        while time.monotonic() < deadline:
            rclpy.spin_once(helper, timeout_sec=0.2)
            if not published:
                if not helper.amcl_active():
                    continue
                # Do not stamp initial pose before the Gazebo clock begins.
                if helper.get_clock().now().nanoseconds <= 0:
                    continue
                if helper.publisher.get_subscription_count() < 1:
                    continue
                helper.publisher.publish(
                    initial_pose_message(args.x, args.y, args.yaw, helper)
                )
                published = True
                result['published_after_amcl_active'] = True
                continue
            if helper.initial_pose_seen:
                result['outcome'] = 'PASS'
                break
        if not published:
            result['error'] = 'AMCL was not active with an /initialpose subscriber'
        elif not helper.initial_pose_seen:
            result['error'] = 'no map-frame /amcl_pose observed after one initial pose'
    finally:
        result_path.parent.mkdir(parents=True, exist_ok=True)
        result_path.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
        helper.destroy_node()
        rclpy.shutdown()
    if result['outcome'] != 'PASS':
        print(json.dumps(result, indent=2), file=sys.stderr)
        return 1
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
