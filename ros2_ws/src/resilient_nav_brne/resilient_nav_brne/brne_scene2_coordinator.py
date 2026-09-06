"""Release Scene 2 pedestrian two after robot progress and a short delay."""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


READY_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class BrneScene2Coordinator(Node):
    """A scenario gate, not a second BRNE temporal/control mechanism."""

    def __init__(self):
        super().__init__('brne_scene2_coordinator')
        self.declare_parameter('robot_pose_topic', '/amcl_pose')
        self.declare_parameter('ready_topic', '/brne/pedestrian2/ready')
        self.declare_parameter('robot_min_map_x', 0.8)
        self.declare_parameter('scene_delay_sec', 0.4)
        self.declare_parameter('map_frame', 'map')
        self.robot_min_map_x = float(self.get_parameter('robot_min_map_x').value)
        self.scene_delay_sec = float(self.get_parameter('scene_delay_sec').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        if (
            not self.map_frame
            or not math.isfinite(self.robot_min_map_x)
            or not math.isfinite(self.scene_delay_sec) or self.scene_delay_sec < 0.0
        ):
            raise ValueError('Scene 2 coordinator parameters are invalid')
        self.publisher = self.create_publisher(
            Bool, str(self.get_parameter('ready_topic').value), READY_QOS
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter('robot_pose_topic').value), self._on_robot_pose, 10,
        )
        self._progress_gate_at = None
        self._robot_map_x = None
        self._released = False
        self.publisher.publish(Bool(data=False))
        self.create_timer(0.05, self._advance)

    def _on_robot_pose(self, message: PoseWithCovarianceStamped) -> None:
        if message.header.frame_id != self.map_frame:
            self._robot_map_x = None
            return
        x_position = message.pose.pose.position.x
        self._robot_map_x = x_position if math.isfinite(x_position) else None

    def _advance(self) -> None:
        if self._released or self._robot_map_x is None:
            return
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self._progress_gate_at is None:
            if self._robot_map_x <= self.robot_min_map_x:
                return
            self._progress_gate_at = now_sec
            self.get_logger().info('pedestrian 1 robot-progress gate observed')
        if (
            self._robot_map_x > self.robot_min_map_x
            and now_sec - self._progress_gate_at >= self.scene_delay_sec
        ):
            self.publisher.publish(Bool(data=True))
            self._released = True
            self.get_logger().info('Scene 2 pedestrian 2 scenario gate released')

    def publish_shutdown_stop(self) -> None:
        if rclpy.ok():
            self.publisher.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = BrneScene2Coordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_shutdown_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
