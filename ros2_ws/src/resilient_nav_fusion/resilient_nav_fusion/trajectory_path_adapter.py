"""Evaluation-only trajectory Path publisher for Phase 8 visualization."""

from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class TrajectoryPathAdapter(Node):
    """Convert truth/fixed/adaptive output streams into bounded Path topics."""

    def __init__(self):
        super().__init__('trajectory_path_adapter')
        self.declare_parameter('expected_frame', 'odom')
        self.declare_parameter('expected_child_frame', 'base_footprint')
        self.declare_parameter('max_poses', 2000)
        self._expected_frame = str(self.get_parameter('expected_frame').value)
        self._expected_child_frame = str(
            self.get_parameter('expected_child_frame').value
        )
        self._max_poses = int(self.get_parameter('max_poses').value)
        self._paths = {
            name: Path(header=_path_header(self._expected_frame))
            for name in ('ground_truth', 'fixed', 'adaptive')
        }
        self._path_publishers = {
            name: self.create_publisher(Path, f'/evaluation/path/{name}', 10)
            for name in self._paths
        }
        self.create_subscription(
            PoseStamped,
            '/evaluation/ground_truth_pose',
            self._on_truth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            '/odometry/faulted',
            self._on_fixed,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            '/odometry/adaptive',
            self._on_adaptive,
            qos_profile_sensor_data,
        )

    def _on_truth(self, message: PoseStamped) -> None:
        if message.header.frame_id == self._expected_frame:
            self._append_and_publish('ground_truth', message.header, message.pose)

    def _on_fixed(self, message: Odometry) -> None:
        self._on_odometry('fixed', message)

    def _on_adaptive(self, message: Odometry) -> None:
        self._on_odometry('adaptive', message)

    def _on_odometry(self, name: str, message: Odometry) -> None:
        if (
            message.header.frame_id == self._expected_frame
            and message.child_frame_id == self._expected_child_frame
        ):
            self._append_and_publish(name, message.header, message.pose.pose)

    def _append_and_publish(self, name, header, pose) -> None:
        """Deep-copy input fields and retain a bounded, frame-consistent path."""
        path = self._paths[name]
        point = PoseStamped()
        point.header = deepcopy(header)
        point.pose = deepcopy(pose)
        path.header = deepcopy(header)
        path.header.frame_id = self._expected_frame
        path.poses.append(point)
        if len(path.poses) > self._max_poses:
            del path.poses[: len(path.poses) - self._max_poses]
        self._path_publishers[name].publish(path)


def _path_header(frame_id: str):
    """Create the fixed frame header without importing truth into any estimator."""
    header = PoseStamped().header
    header.frame_id = frame_id
    return header


def main(args=None):
    """Run the evaluation-only trajectory overlay adapter."""
    rclpy.init(args=args)
    node = TrajectoryPathAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
