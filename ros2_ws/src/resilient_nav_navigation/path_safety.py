"""Pure full-footprint safety checks for Nav2 global paths.

The Navfn output is a path for the robot origin.  These helpers provide a
deterministic, project-side acceptance check against the raw Nav2 Costmap
without changing either Navfn or Nav2's runtime collision checker.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from costmap_contract import padded_footprint


LETHAL_COST = 254
UNKNOWN_COST = 255


@dataclass(frozen=True)
class RawCostmapGrid:
    """ROS-independent view of a nav2_msgs/Costmap grid."""

    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    data: tuple[int, ...]

    @classmethod
    def from_message(cls, message) -> 'RawCostmapGrid':
        """Copy the fields used by the checker from a Costmap ROS message."""
        metadata = message.metadata
        grid = cls(
            resolution=float(metadata.resolution),
            width=int(metadata.size_x),
            height=int(metadata.size_y),
            origin_x=float(metadata.origin.position.x),
            origin_y=float(metadata.origin.position.y),
            data=tuple(int(value) for value in message.data),
        )
        grid.validate()
        return grid

    def validate(self) -> None:
        if (
            not math.isfinite(self.resolution)
            or self.resolution <= 0.0
            or self.width <= 0
            or self.height <= 0
            or len(self.data) != self.width * self.height
        ):
            raise ValueError('raw Costmap metadata/data are inconsistent')

    def value_at(self, column: int, row: int) -> int:
        return self.data[row * self.width + column]


def normalize_angle(angle: float) -> float:
    """Normalize an angle into [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _interpolate_angle(start: float, end: float, fraction: float) -> float:
    return normalize_angle(start + normalize_angle(end - start) * fraction)


def _transform_polygon(
    polygon: Sequence[tuple[float, float]], x: float, y: float, yaw: float
) -> tuple[tuple[float, float], ...]:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return tuple(
        (x + point_x * cosine - point_y * sine, y + point_x * sine + point_y * cosine)
        for point_x, point_y in polygon
    )


def _project(
    points: Sequence[tuple[float, float]], axis: tuple[float, float]
) -> tuple[float, float]:
    values = [point[0] * axis[0] + point[1] * axis[1] for point in points]
    return min(values), max(values)


def _polygons_intersect(
    first: Sequence[tuple[float, float]], second: Sequence[tuple[float, float]]
) -> bool:
    """Return exact convex-polygon overlap using the separating-axis theorem."""
    for polygon in (first, second):
        for start, end in zip(polygon, polygon[1:] + polygon[:1]):
            edge_x, edge_y = end[0] - start[0], end[1] - start[1]
            magnitude = math.hypot(edge_x, edge_y)
            if magnitude == 0.0:
                continue
            axis = (-edge_y / magnitude, edge_x / magnitude)
            first_min, first_max = _project(first, axis)
            second_min, second_max = _project(second, axis)
            if first_max < second_min or second_max < first_min:
                return False
    return True


def _cell_square(grid: RawCostmapGrid, column: int, row: int) -> tuple[tuple[float, float], ...]:
    x = grid.origin_x + column * grid.resolution
    y = grid.origin_y + row * grid.resolution
    return ((x, y), (x + grid.resolution, y), (x + grid.resolution, y + grid.resolution), (x, y + grid.resolution))


def _candidate_cells(
    grid: RawCostmapGrid, polygon: Sequence[tuple[float, float]]
) -> tuple[range, range, bool]:
    min_x = min(point[0] for point in polygon)
    max_x = max(point[0] for point in polygon)
    min_y = min(point[1] for point in polygon)
    max_y = max(point[1] for point in polygon)
    start_column = math.floor((min_x - grid.origin_x) / grid.resolution)
    end_column = math.floor((max_x - grid.origin_x) / grid.resolution)
    start_row = math.floor((min_y - grid.origin_y) / grid.resolution)
    end_row = math.floor((max_y - grid.origin_y) / grid.resolution)
    out_of_bounds = (
        start_column < 0
        or start_row < 0
        or end_column >= grid.width
        or end_row >= grid.height
    )
    return (
        range(max(0, start_column), min(grid.width - 1, end_column) + 1),
        range(max(0, start_row), min(grid.height - 1, end_row) + 1),
        out_of_bounds,
    )


def _pose_evidence(
    grid: RawCostmapGrid, footprint: Sequence[tuple[float, float]], x: float, y: float, yaw: float
) -> dict[str, object]:
    polygon = _transform_polygon(footprint, x, y, yaw)
    columns, rows, out_of_bounds = _candidate_cells(grid, polygon)
    evidence: dict[str, object] = {
        'pose': {'x': x, 'y': y, 'yaw': yaw},
        'out_of_bounds': out_of_bounds,
        'lethal_cell_count': 0,
        'unknown_cell_count': 0,
        'inscribed_cell_count': 0,
    }
    for row in rows:
        for column in columns:
            if not _polygons_intersect(polygon, _cell_square(grid, column, row)):
                continue
            value = grid.value_at(column, row)
            if value == LETHAL_COST:
                evidence['lethal_cell_count'] += 1
            elif value == UNKNOWN_COST:
                evidence['unknown_cell_count'] += 1
            elif value == 253:
                evidence['inscribed_cell_count'] += 1
    return evidence


def _append_pose(
    poses: list[tuple[float, float, float]], x: float, y: float, yaw: float
) -> None:
    candidate = (x, y, normalize_angle(yaw))
    if not poses or any(abs(first - second) > 1e-12 for first, second in zip(poses[-1], candidate)):
        poses.append(candidate)


def _swept_poses(
    points: Sequence[tuple[float, float]],
    initial_yaw: float,
    goal_yaw: float,
    translation_spacing: float,
    yaw_spacing: float,
) -> list[tuple[float, float, float]]:
    clean_points = [points[0]]
    for point in points[1:]:
        if math.dist(clean_points[-1], point) > 1e-9:
            clean_points.append(point)
    if not clean_points:
        raise ValueError('cannot sweep an empty path')
    if not all(math.isfinite(value) for point in clean_points for value in point):
        raise ValueError('path contains a non-finite coordinate')
    tangents: list[float] = []
    for start, end in zip(clean_points, clean_points[1:]):
        tangents.append(math.atan2(end[1] - start[1], end[0] - start[0]))
    poses: list[tuple[float, float, float]] = []
    if not tangents:
        tangents = [initial_yaw]
    first_tangent = tangents[0]
    turn_count = max(1, math.ceil(abs(normalize_angle(first_tangent - initial_yaw)) / yaw_spacing))
    for index in range(turn_count + 1):
        _append_pose(poses, clean_points[0][0], clean_points[0][1], _interpolate_angle(initial_yaw, first_tangent, index / turn_count))
    for index, (start, end) in enumerate(zip(clean_points, clean_points[1:])):
        tangent = tangents[index]
        length = math.dist(start, end)
        count = max(1, math.ceil(length / translation_spacing))
        for step in range(1, count + 1):
            fraction = step / count
            _append_pose(
                poses,
                start[0] + (end[0] - start[0]) * fraction,
                start[1] + (end[1] - start[1]) * fraction,
                tangent,
            )
        if index + 1 < len(tangents):
            next_tangent = tangents[index + 1]
            turn_count = max(1, math.ceil(abs(normalize_angle(next_tangent - tangent)) / yaw_spacing))
            for step in range(1, turn_count + 1):
                _append_pose(poses, end[0], end[1], _interpolate_angle(tangent, next_tangent, step / turn_count))
    final_tangent = tangents[-1]
    turn_count = max(1, math.ceil(abs(normalize_angle(goal_yaw - final_tangent)) / yaw_spacing))
    for index in range(1, turn_count + 1):
        _append_pose(poses, clean_points[-1][0], clean_points[-1][1], _interpolate_angle(final_tangent, goal_yaw, index / turn_count))
    return poses


def full_footprint_path_sweep(
    grid: RawCostmapGrid,
    points: Sequence[tuple[float, float]],
    initial_yaw: float,
    goal_yaw: float,
) -> dict[str, object]:
    """Validate every swept padded-footprint pose against raw Nav2 costs.

    Translation increments are at most half a Costmap cell.  Rotation samples
    make the outermost footprint vertex travel at most half a cell, including
    the terminal orientation required by FollowPath's goal checker.
    """
    grid.validate()
    footprint = padded_footprint()
    max_radius = max(math.hypot(x, y) for x, y in footprint)
    translation_spacing = grid.resolution / 2.0
    yaw_spacing = translation_spacing / max_radius
    poses = _swept_poses(points, initial_yaw, goal_yaw, translation_spacing, yaw_spacing)
    total_lethal = total_unknown = total_inscribed = 0
    first_violation = None
    for index, (x, y, yaw) in enumerate(poses):
        evidence = _pose_evidence(grid, footprint, x, y, yaw)
        total_lethal += evidence['lethal_cell_count']
        total_unknown += evidence['unknown_cell_count']
        total_inscribed += evidence['inscribed_cell_count']
        if first_violation is None and (
            evidence['out_of_bounds']
            or evidence['lethal_cell_count']
            or evidence['unknown_cell_count']
        ):
            first_violation = {'sweep_index': index, **evidence}
    return {
        'safe': first_violation is None,
        'sweep_pose_count': len(poses),
        'translation_spacing_m': translation_spacing,
        'yaw_spacing_rad': yaw_spacing,
        'lethal_intersection_count': total_lethal,
        'unknown_intersection_count': total_unknown,
        'inscribed_intersection_count': total_inscribed,
        'first_violation': first_violation,
    }
