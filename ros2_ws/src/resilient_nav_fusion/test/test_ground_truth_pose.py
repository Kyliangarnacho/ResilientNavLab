"""Contracts for the isolated Phase 8 evaluation pose adapter."""

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from resilient_nav_fusion.ground_truth_pose import ground_truth_pose_from_tf


def transform(frame='odom', child='base_footprint', sec=12, nanosec=34):
    """Build a transform with distinct pose values and a valid timestamp."""
    value = TransformStamped()
    value.header.frame_id = frame
    value.child_frame_id = child
    value.header.stamp.sec = sec
    value.header.stamp.nanosec = nanosec
    value.transform.translation.x = 1.5
    value.transform.translation.y = -2.0
    value.transform.rotation.z = 0.25
    value.transform.rotation.w = 0.75
    return value


def test_ground_truth_pose_preserves_type_frame_timestamp_and_pose():
    """Exact Gazebo model TF becomes a stamped odom-frame evaluation pose."""
    source = transform()
    output = ground_truth_pose_from_tf(TFMessage(transforms=[source]))

    assert output is not None
    assert output.__class__.__name__ == 'PoseStamped'
    assert output.header == source.header
    assert output.header.frame_id == 'odom'
    assert output.header.stamp == source.header.stamp
    assert output.pose.position.x == source.transform.translation.x
    assert output.pose.position.y == source.transform.translation.y
    assert output.pose.position.z == source.transform.translation.z
    assert output.pose.orientation == source.transform.rotation


def test_ground_truth_pose_fails_closed_for_missing_ambiguous_or_invalid_frame():
    """No positional guessing or arbitrary TF is allowed on the truth channel."""
    valid = transform()
    wrong = transform(child='base_link')

    assert ground_truth_pose_from_tf(TFMessage(transforms=[wrong])) is None
    assert ground_truth_pose_from_tf(TFMessage(transforms=[valid, valid])) is None
    assert ground_truth_pose_from_tf(
        TFMessage(transforms=[transform(nanosec=1_000_000_000)])
    ) is None
