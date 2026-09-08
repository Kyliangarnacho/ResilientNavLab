"""Evaluation-only conversion from Gazebo model TF to a stamped pose."""

from copy import deepcopy
from math import isfinite, sqrt

from geometry_msgs.msg import Pose, PoseStamped
from tf2_msgs.msg import TFMessage


def ground_truth_pose_from_tf(
    message: TFMessage,
    *,
    expected_frame: str = 'resilient_lab',
    expected_child_frame: str = 'resilient_nav_robot',
) -> PoseStamped | None:
    """Select the independent Gazebo world-to-model transform."""
    matches = [
        transform
        for transform in message.transforms
        if transform.header.frame_id == expected_frame
        and transform.child_frame_id == expected_child_frame
    ]
    if len(matches) != 1:
        return None

    transform = matches[0]
    if (
        not _valid_stamp(
            transform.header.stamp.sec,
            transform.header.stamp.nanosec,
        )
        or not _valid_pose(
            transform.transform.translation,
            transform.transform.rotation,
        )
    ):
        return None
    output = PoseStamped()
    output.header = deepcopy(transform.header)
    output.pose.position.x = transform.transform.translation.x
    output.pose.position.y = transform.transform.translation.y
    output.pose.position.z = transform.transform.translation.z
    output.pose.orientation = deepcopy(transform.transform.rotation)
    return output


def relative_pose_from_origin(
    source: PoseStamped,
    origin: Pose,
    *,
    output_frame: str = 'odom',
) -> PoseStamped | None:
    """Express an independent world pose relative to the first valid pose."""
    if (
        not output_frame
        or not _valid_pose(source.pose.position, source.pose.orientation)
        or not _valid_pose(origin.position, origin.orientation)
    ):
        return None

    origin_q = _normalized_quaternion(origin.orientation)
    source_q = _normalized_quaternion(source.pose.orientation)
    inverse_origin = (-origin_q[0], -origin_q[1], -origin_q[2], origin_q[3])
    delta = (
        source.pose.position.x - origin.position.x,
        source.pose.position.y - origin.position.y,
        source.pose.position.z - origin.position.z,
    )
    relative_translation = _rotate_vector(inverse_origin, delta)
    relative_q = _normalized_tuple(_multiply_quaternions(inverse_origin, source_q))

    output = PoseStamped()
    output.header = deepcopy(source.header)
    output.header.frame_id = output_frame
    output.pose.position.x = relative_translation[0]
    output.pose.position.y = relative_translation[1]
    output.pose.position.z = relative_translation[2]
    output.pose.orientation.x = relative_q[0]
    output.pose.orientation.y = relative_q[1]
    output.pose.orientation.z = relative_q[2]
    output.pose.orientation.w = relative_q[3]
    return output


def _valid_stamp(sec: int, nanosec: int) -> bool:
    """Accept valid ROS times, including the simulation epoch zero."""
    return sec >= 0 and 0 <= nanosec < 1_000_000_000


def _valid_pose(position, orientation) -> bool:
    values = (
        position.x,
        position.y,
        position.z,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )
    return all(isfinite(value) for value in values) and sum(
        value * value
        for value in (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
    ) > 1.0e-12


def _normalized_quaternion(value):
    return _normalized_tuple((value.x, value.y, value.z, value.w))


def _normalized_tuple(value):
    norm = sqrt(sum(component * component for component in value))
    return tuple(component / norm for component in value)


def _multiply_quaternions(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _rotate_vector(quaternion, vector):
    vector_q = (vector[0], vector[1], vector[2], 0.0)
    inverse = (
        -quaternion[0],
        -quaternion[1],
        -quaternion[2],
        quaternion[3],
    )
    rotated = _multiply_quaternions(
        _multiply_quaternions(quaternion, vector_q),
        inverse,
    )
    return rotated[:3]
