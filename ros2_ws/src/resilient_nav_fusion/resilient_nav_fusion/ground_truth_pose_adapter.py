"""Publish Gazebo model pose only to the evaluation ground-truth channel."""

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage

from .ground_truth_pose import ground_truth_pose_from_tf


class GroundTruthPoseAdapter(Node):
    """Fail-closed evaluation adapter; it has no fusion or health connection."""

    def __init__(self):
        super().__init__('ground_truth_pose_adapter')
        self.declare_parameter('input_topic', '/evaluation/gazebo_model_tf')
        self.declare_parameter('output_topic', '/evaluation/ground_truth_pose')
        self.declare_parameter('expected_frame', 'odom')
        self.declare_parameter('expected_child_frame', 'base_footprint')
        self._publisher = self.create_publisher(
            PoseStamped, str(self.get_parameter('output_topic').value), 10
        )
        self.create_subscription(
            TFMessage,
            str(self.get_parameter('input_topic').value),
            self._on_transform,
            qos_profile_sensor_data,
        )

    def _on_transform(self, message: TFMessage) -> None:
        pose = ground_truth_pose_from_tf(
            message,
            expected_frame=str(self.get_parameter('expected_frame').value),
            expected_child_frame=str(
                self.get_parameter('expected_child_frame').value
            ),
        )
        if pose is None:
            self.get_logger().warning('suppressed invalid evaluation ground-truth pose')
            return
        self._publisher.publish(pose)


def main(args=None):
    """Run the evaluation-only Gazebo pose adapter."""
    rclpy.init(args=args)
    node = GroundTruthPoseAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
