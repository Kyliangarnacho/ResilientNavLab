"""Pure, shared contracts for the Phase 10 Costmap smoke tools."""

from __future__ import annotations

import math
from collections.abc import Iterable, Sequence


PHYSICAL_FOOTPRINT = (
    (-0.35, -0.175),
    (-0.10, -0.215),
    (0.10, -0.215),
    (0.15, -0.175),
    (0.15, 0.175),
    (0.10, 0.215),
    (-0.10, 0.215),
    (-0.35, 0.175),
)
FOOTPRINT_PADDING = 0.01
LETHAL_COST_THRESHOLD = 99


def close(actual: float, expected: float, tolerance: float = 1e-6) -> bool:
    """Return whether two finite geometry values agree within tolerance."""
    return math.isfinite(actual) and abs(actual - expected) <= tolerance


def padded_footprint(
    footprint: Sequence[tuple[float, float]] = PHYSICAL_FOOTPRINT,
    padding: float = FOOTPRINT_PADDING,
) -> tuple[tuple[float, float], ...]:
    """Match Nav2's per-coordinate footprint-padding convention."""
    return tuple(
        (
            x + math.copysign(padding, x),
            y + math.copysign(padding, y),
        )
        for x, y in footprint
    )


def yaw_from_quaternion(quaternion) -> float:
    """Extract planar yaw from an object exposing x/y/z/w quaternion fields."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def transform_polygon(
    polygon: Sequence[tuple[float, float]], transform
) -> tuple[tuple[float, float], ...]:
    """Apply a planar Transform message to a footprint polygon."""
    yaw = yaw_from_quaternion(transform.rotation)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return tuple(
        (
            transform.translation.x + x * cosine - y * sine,
            transform.translation.y + x * sine + y * cosine,
        )
        for x, y in polygon
    )


def cost_distribution(values: Iterable[int]) -> dict[str, int]:
    """Classify published OccupancyGrid values without hiding 99-cost cells."""
    values = tuple(values)
    return {
        'unknown': sum(value == -1 for value in values),
        'free': sum(value == 0 for value in values),
        'inflated': sum(0 < value < LETHAL_COST_THRESHOLD for value in values),
        'lethal_like': sum(value >= LETHAL_COST_THRESHOLD for value in values),
        'cost_99': sum(value == 99 for value in values),
        'cost_100': sum(value == 100 for value in values),
    }


def grid_cell_index(
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
    x: float,
    y: float,
) -> int:
    """Return the flattened index for a world-frame grid coordinate."""
    column = math.floor((x - origin_x) / resolution)
    row = math.floor((y - origin_y) / resolution)
    if not (0 <= column < width and 0 <= row < height):
        raise ValueError(f'watch point ({x}, {y}) is outside the costmap')
    return row * width + column


def rolling_window_center(
    origin_x: float, origin_y: float, width: int, height: int, resolution: float
) -> tuple[float, float]:
    """Return the geometric centre of an OccupancyGrid window."""
    return (
        origin_x + width * resolution / 2.0,
        origin_y + height * resolution / 2.0,
    )
