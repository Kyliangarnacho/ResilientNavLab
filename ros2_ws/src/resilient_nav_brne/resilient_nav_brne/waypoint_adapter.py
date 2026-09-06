"""Small, ROS-free geometry helpers for the BRNE real-data input adapter."""

from __future__ import annotations

from math import atan2, cos, hypot, isfinite, sin
from typing import Iterable, Sequence

import numpy as np


def yaw_from_quaternion(quaternion) -> float | None:
    """Return planar yaw for a finite, non-zero quaternion, else reject it."""
    values = np.asarray(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w], dtype=float
    )
    if not np.isfinite(values).all() or float(np.linalg.norm(values)) <= 1e-6:
        return None
    return atan2(
        2.0 * (values[3] * values[2] + values[0] * values[1]),
        1.0 - 2.0 * (values[1] ** 2 + values[2] ** 2),
    )


def transform_points_se2(
    points: Iterable[Sequence[float]], translation_x: float, translation_y: float,
    yaw: float,
) -> np.ndarray | None:
    """Apply one finite map-to-odom planar transform to all Path positions."""
    array = np.asarray(list(points), dtype=float)
    if array.ndim != 2 or array.shape[0] == 0 or array.shape[1] != 2:
        return None
    values = np.asarray([translation_x, translation_y, yaw], dtype=float)
    if not np.isfinite(array).all() or not np.isfinite(values).all():
        return None
    rotation = np.array([[cos(yaw), -sin(yaw)], [sin(yaw), cos(yaw)]])
    return array @ rotation.T + values[:2]


def select_local_waypoint(
    transformed_path: Sequence[Sequence[float]], robot_xy: Sequence[float],
    lookahead_distance: float, maximum_nearest_distance: float,
) -> tuple[int, np.ndarray, float] | None:
    """Pick a forward arc-length waypoint only when the current Path is nearby."""
    points = np.asarray(transformed_path, dtype=float)
    robot = np.asarray(robot_xy, dtype=float)
    if (
        points.ndim != 2 or points.shape[0] == 0 or points.shape[1] != 2
        or robot.shape != (2,) or not np.isfinite(points).all()
        or not np.isfinite(robot).all() or not isfinite(lookahead_distance)
        or not isfinite(maximum_nearest_distance) or lookahead_distance <= 0.0
        or maximum_nearest_distance <= 0.0
    ):
        return None
    nearest_index = int(np.argmin(np.linalg.norm(points - robot, axis=1)))
    nearest_distance = float(np.linalg.norm(points[nearest_index] - robot))
    if nearest_distance > maximum_nearest_distance:
        return None
    selected_index = nearest_index
    traversed = 0.0
    for index in range(nearest_index + 1, len(points)):
        traversed += float(np.linalg.norm(points[index] - points[index - 1]))
        selected_index = index
        if traversed >= lookahead_distance:
            break
    return selected_index, points[selected_index].copy(), nearest_distance
