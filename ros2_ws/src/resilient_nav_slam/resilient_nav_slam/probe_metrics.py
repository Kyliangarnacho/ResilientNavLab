"""Pure metrics used by the read-only Phase 9 SLAM probe."""

from collections.abc import Sequence
import math


def scan_rate_hz(receipt_times_sec: Sequence[float]) -> float | None:
    """Return the average receive rate over the supplied monotonic timestamps."""
    if len(receipt_times_sec) < 2:
        return None

    duration_sec = receipt_times_sec[-1] - receipt_times_sec[0]
    if duration_sec <= 0.0:
        return None
    return (len(receipt_times_sec) - 1) / duration_sec


def occupancy_ratios(cells: Sequence[int]) -> dict[str, float | None]:
    """Classify OccupancyGrid cells using ROS's -1/0..100 convention."""
    total = len(cells)
    if total == 0:
        return {'unknown': None, 'free': None, 'occupied': None}

    unknown = sum(cell < 0 for cell in cells)
    free = sum(0 <= cell < 50 for cell in cells)
    occupied = sum(cell >= 50 for cell in cells)
    return {
        'unknown': unknown / total,
        'free': free / total,
        'occupied': occupied / total,
    }


def freshness_seconds(now_nanoseconds: int, stamp_nanoseconds: int) -> float | None:
    """Return age from ROS clock time, or ``None`` for an unset transform stamp."""
    if stamp_nanoseconds <= 0:
        return None
    return max(0.0, (now_nanoseconds - stamp_nanoseconds) / 1_000_000_000.0)


def stamp_nanoseconds(stamp) -> int:
    """Convert a ROS builtin time-like object to nanoseconds without ROS imports."""
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return the planar yaw of a quaternion without depending on ROS helpers."""
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
