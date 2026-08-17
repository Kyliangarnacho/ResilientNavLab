"""Evaluation-only conversion from Gazebo model TF to a stamped pose."""

from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage


def ground_truth_pose_from_tf(
    message: TFMessage,
    *,
    expected_frame: str = 'odom',
    expected_child_frame: str = 'base_footprint',
) -> PoseStamped | None:
    """Select one exact model transform or fail closed without publishing."""
    matches = [
        transform
        for transform in message.transforms
        if transform.header.frame_id == expected_frame
        and transform.child_frame_id == expected_child_frame
    ]
    if len(matches) != 1:
        return None

    transform = matches[0]
    if not _valid_stamp(transform.header.stamp.sec, transform.header.stamp.nanosec):
        return None
    output = PoseStamped()
    output.header = deepcopy(transform.header)
    output.pose.position.x = transform.transform.translation.x
    output.pose.position.y = transform.transform.translation.y
    output.pose.position.z = transform.transform.translation.z
    output.pose.orientation = deepcopy(transform.transform.rotation)
    return output


def _valid_stamp(sec: int, nanosec: int) -> bool:
    """Accept valid ROS times, including the simulation epoch zero."""
    return sec >= 0 and 0 <= nanosec < 1_000_000_000
