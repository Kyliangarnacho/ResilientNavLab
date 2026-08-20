"""Publish map metadata on an evaluation-only topic without changing SLAM."""

import json

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from .probe_metrics import occupancy_ratios


class SlamMapAdapter(Node):
    """Translate read-only /map observations to evaluation-only JSON."""

    def __init__(self):
        super().__init__('slam_map_adapter')
        self.declare_parameter('input_topic', '/map')
        self.declare_parameter('output_topic', '/evaluation/slam_map_metrics')
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            String, str(self.get_parameter('output_topic').value), 10
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('input_topic').value),
            self._on_map,
            map_qos,
        )

    def _on_map(self, message: OccupancyGrid) -> None:
        payload = {
            'width': message.info.width,
            'height': message.info.height,
            'resolution': message.info.resolution,
            'origin': {
                'x': message.info.origin.position.x,
                'y': message.info.origin.position.y,
                'yaw': _yaw(message.info.origin.orientation.z, message.info.origin.orientation.w),
            },
            'ratios': occupancy_ratios(message.data),
        }
        output = String()
        output.data = json.dumps(payload, sort_keys=True, separators=(',', ':'))
        self._publisher.publish(output)


def _yaw(z: float, w: float) -> float:
    """Return the planar yaw for the expected planar occupancy-grid origin."""
    from math import atan2
    return atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def main(args=None):
    """Run the evaluation-only map metadata adapter."""
    rclpy.init(args=args)
    node = SlamMapAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
