"""Contracts for the isolated Phase 8 evaluation pose adapter."""

import math

from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import TransformStamped
import pytest
from resilient_nav_fusion.ground_truth_pose import (
    ground_truth_pose_from_tf,
    relative_pose_from_origin,
)
from tf2_msgs.msg import TFMessage


def transform(
    frame='resilient_lab',
    child='resilient_nav_robot',
    sec=12,
    nanosec=34,
):
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
    """Exact Gazebo world-model TF becomes a stamped evaluation pose."""
    source = transform()
    output = ground_truth_pose_from_tf(TFMessage(transforms=[source]))

    assert output is not None
    assert output.__class__.__name__ == 'PoseStamped'
    assert output.header == source.header
    assert output.header.frame_id == 'resilient_lab'
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


def test_world_pose_is_rebased_in_first_pose_frame_with_full_orientation():
    """Rebasing preserves independent 3D motion but removes spawn offset."""
    origin = Pose()
    origin.position.x = 5.0
    origin.position.y = -3.0
    origin.position.z = 0.2
    origin.orientation.z = math.sin(math.pi / 4.0)
    origin.orientation.w = math.cos(math.pi / 4.0)
    source = PoseStamped()
    source.header.frame_id = 'resilient_lab'
    source.header.stamp.sec = 4
    source.pose.position.x = 5.0
    source.pose.position.y = -2.0
    source.pose.position.z = 0.3
    source.pose.orientation.z = 1.0
    source.pose.orientation.w = 0.0

    output = relative_pose_from_origin(source, origin)

    assert output is not None
    assert output.header.frame_id == 'odom'
    assert output.header.stamp == source.header.stamp
    assert output.pose.position.x == pytest.approx(1.0)
    assert output.pose.position.y == pytest.approx(0.0, abs=1.0e-12)
    assert output.pose.position.z == pytest.approx(0.1)
    assert output.pose.orientation.z == pytest.approx(math.sin(math.pi / 4.0))
    assert output.pose.orientation.w == pytest.approx(math.cos(math.pi / 4.0))
