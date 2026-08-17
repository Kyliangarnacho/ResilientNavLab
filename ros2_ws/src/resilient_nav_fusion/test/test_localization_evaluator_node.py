"""ROS message conversion contracts for the evaluation-only adapter."""

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from resilient_nav_fusion.localization_evaluator_node import (
    _odometry_to_timed_pose,
    _pose_stamped_to_timed_pose,
)


def test_pose_stamped_truth_preserves_valid_odom_frame_and_timestamp():
    """The evaluator accepts the isolated Ground Truth Channel message type."""
    message = PoseStamped()
    message.header.frame_id = 'odom'
    message.header.stamp.sec = 17
    message.header.stamp.nanosec = 250_000_000
    message.pose.position.x = 1.25
    message.pose.orientation.w = 1.0

    result = _pose_stamped_to_timed_pose(message, 'odom')

    assert result is not None
    assert result.stamp_sec == 17.25
    assert result.x == 1.25
    assert _pose_stamped_to_timed_pose(message, 'map') is None


def test_odometry_requires_odom_to_base_footprint_and_a_valid_orientation():
    """Estimator inputs with mismatched TF ownership are suppressed."""
    message = Odometry()
    message.header.frame_id = 'odom'
    message.child_frame_id = 'base_footprint'
    message.header.stamp.sec = 3
    message.pose.pose.orientation.w = 1.0

    assert _odometry_to_timed_pose(message, 'odom', 'base_footprint') is not None
    assert _odometry_to_timed_pose(message, 'odom', 'base_link') is None
    message.pose.pose.orientation.w = 0.0
    assert _odometry_to_timed_pose(message, 'odom', 'base_footprint') is None
