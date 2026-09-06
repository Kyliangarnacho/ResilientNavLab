"""Publish the Task 1 synthetic pedestrian only on the isolated BRNE topic."""

import rclpy
from rclpy.node import Node

from resilient_nav_interfaces.msg import Pedestrian, PedestrianArray


class BrneSyntheticPedestrianSource(Node):
    """Retain the Task 1 synthetic moving pedestrian without odom or goal input."""

    def __init__(self):
        """Create one deterministic, odom-frame pedestrian publisher."""
        super().__init__('brne_synthetic_pedestrian_source')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('publish_frequency_hz', 10.0)
        self.declare_parameter('pedestrian_id', 1)
        self.declare_parameter('x', 1.20)
        self.declare_parameter('y', 0.50)
        self.declare_parameter('vx', 0.0)
        self.declare_parameter('vy', -0.10)
        frequency = float(self.get_parameter('publish_frequency_hz').value)
        if frequency <= 0.0:
            raise ValueError('publish_frequency_hz must be positive')
        self.publisher = self.create_publisher(PedestrianArray, '/brne/pedestrians', 10)
        self.create_timer(1.0 / frequency, self._publish)

    def _publish(self):
        """Emit one finite pedestrian state for BRNE trajectory prediction."""
        message = PedestrianArray()
        message.header.frame_id = str(self.get_parameter('frame_id').value)
        message.header.stamp = self.get_clock().now().to_msg()
        pedestrian = Pedestrian()
        pedestrian.header = message.header
        pedestrian.id = int(self.get_parameter('pedestrian_id').value)
        pedestrian.pose.position.x = float(self.get_parameter('x').value)
        pedestrian.pose.position.y = float(self.get_parameter('y').value)
        pedestrian.pose.orientation.w = 1.0
        pedestrian.velocity.linear.x = float(self.get_parameter('vx').value)
        pedestrian.velocity.linear.y = float(self.get_parameter('vy').value)
        message.pedestrians.append(pedestrian)
        self.publisher.publish(message)


def main(args=None):
    """Run the isolated source without synthetic odometry or navigation goals."""
    rclpy.init(args=args)
    node = BrneSyntheticPedestrianSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
