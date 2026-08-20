"""Unit tests for the ROS-independent SLAM probe metrics."""

import math

import pytest

from resilient_nav_slam.probe_metrics import (
    freshness_seconds,
    occupancy_ratios,
    quaternion_yaw,
    scan_rate_hz,
)


def test_scan_rate_uses_the_elapsed_receive_window():
    assert scan_rate_hz([10.0, 10.2, 10.4]) == pytest.approx(5.0)
    assert scan_rate_hz([10.0]) is None
    assert scan_rate_hz([10.0, 10.0]) is None


def test_occupancy_ratios_classify_unknown_free_and_occupied_cells():
    assert occupancy_ratios([-1, 0, 49, 50, 100]) == {
        'unknown': 0.2,
        'free': 0.4,
        'occupied': 0.4,
    }
    assert occupancy_ratios([]) == {
        'unknown': None,
        'free': None,
        'occupied': None,
    }


def test_freshness_uses_ros_time_and_rejects_zero_stamps():
    assert freshness_seconds(6_000_000_000, 4_500_000_000) == 1.5
    assert freshness_seconds(4_000_000_000, 4_500_000_000) == 0.0
    assert freshness_seconds(4_000_000_000, 0) is None


def test_quaternion_yaw_returns_planar_heading():
    """The probe reports TF yaw without creating a ROS-side transform."""
    assert quaternion_yaw(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)
    assert quaternion_yaw(0.0, 0.0, 1.0, 0.0) == pytest.approx(math.pi)
