"""Read-only TF adapter that exposes SLAM pose only to evaluation topics."""

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class SlamPoseAdapter(Node):
    """Query map to base_footprint without publishing or owning TF."""

    def __init__(self):
        super().__init__('slam_pose_adapter')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('output_topic', '/evaluation/slam_pose')
        self.declare_parameter('query_period_sec', 0.05)
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self, spin_thread=True)
        self._publisher = self.create_publisher(
            PoseStamped, str(self.get_parameter('output_topic').value), 10
        )
        self.create_timer(
            float(self.get_parameter('query_period_sec').value), self._publish_pose
        )

    def _publish_pose(self) -> None:
        try:
            transform = self._buffer.lookup_transform(
                str(self.get_parameter('map_frame').value),
                str(self.get_parameter('base_frame').value),
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return
        message = PoseStamped()
        message.header = transform.header
        message.pose.position.x = transform.transform.translation.x
        message.pose.position.y = transform.transform.translation.y
        message.pose.position.z = transform.transform.translation.z
        message.pose.orientation = transform.transform.rotation
        self._publisher.publish(message)


def main(args=None):
    """Run the evaluation-only TF reader."""
    rclpy.init(args=args)
    node = SlamPoseAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
