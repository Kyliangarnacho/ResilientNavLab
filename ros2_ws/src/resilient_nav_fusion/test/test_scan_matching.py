"""Focused tests for the ROS-free LiDAR scan-matching core."""

from math import cos, sin

import numpy as np

import pytest

from resilient_nav_fusion.scan_matching import (
    PlanarScanMatcher,
    ScanMatcherConfig,
    laser_scan_points,
)


def test_scan_points_filter_invalid_ranges_and_bound_work():
    """Only finite, in-range beams enter a bounded match set."""
    points = laser_scan_points(
        [float('nan'), 1.0, float('inf'), 2.0, 8.0, 1.5],
        angle_min=-0.5,
        angle_increment=0.2,
        range_min=0.1,
        range_max=5.0,
        max_points=3,
    )

    assert points is not None
    assert points.shape == (3, 2)
    assert np.all(np.isfinite(points))


def test_match_recovers_adjacent_scan_motion():
    """ICP recovers a small known rigid motion between adjacent scans."""
    random = np.random.default_rng(12)
    previous = random.uniform((-3.0, -2.0), (4.0, 3.0), size=(160, 2))
    expected_angle = 0.04
    expected_translation = np.array((0.06, -0.015))
    rotation = np.array((
        (cos(expected_angle), -sin(expected_angle)),
        (sin(expected_angle), cos(expected_angle)),
    ))
    current = (previous - expected_translation) @ rotation

    result = PlanarScanMatcher().match(previous, current)

    assert result is not None
    assert result.translation_previous_frame == pytest.approx(
        expected_translation, abs=1.0e-6
    )
    assert result.rotation_rad == pytest.approx(expected_angle, abs=1.0e-6)
    assert result.rmse_m < 1.0e-6
    assert result.inlier_ratio == pytest.approx(1.0)


def test_point_to_line_match_avoids_near_zero_scan_sampling_minimum():
    """Ordered surfaces recover a sub-beam translation without wheel hints."""
    front_wall = np.column_stack((
        np.full(80, 2.0),
        np.linspace(-2.0, 2.0, 80),
    ))
    side_wall = np.column_stack((
        np.linspace(2.0, -2.0, 80),
        np.full(80, 2.0),
    ))
    previous = np.vstack((front_wall, side_wall))
    expected_angle = 0.015
    expected_translation = np.array((0.022, -0.004))
    rotation = np.array((
        (cos(expected_angle), -sin(expected_angle)),
        (sin(expected_angle), cos(expected_angle)),
    ))
    current = (previous - expected_translation) @ rotation

    result = PlanarScanMatcher().match(previous, current)

    assert result is not None
    assert result.translation_previous_frame == pytest.approx(
        expected_translation, abs=0.003
    )
    assert result.rotation_rad == pytest.approx(expected_angle, abs=0.003)


def test_match_rejects_motion_outside_declared_bound():
    """A converged but implausibly large scan jump fails closed."""
    random = np.random.default_rng(7)
    previous = random.uniform((-2.0, -2.0), (2.0, 2.0), size=(120, 2))
    current = previous - np.array((0.30, 0.0))
    matcher = PlanarScanMatcher(ScanMatcherConfig(
        max_translation_m=0.10,
        max_correspondence_distance_m=0.50,
    ))

    assert matcher.match(previous, current) is None


def test_match_rejects_insufficient_geometry():
    """Repeated coincident points cannot produce a trusted motion."""
    matcher = PlanarScanMatcher()
    points = np.zeros((50, 2))

    assert matcher.match(points, points) is None


def test_robust_match_rejects_new_obstacle_points_before_motion_fit():
    """A suddenly appeared wall-like cluster cannot drag the ego-motion fit."""
    random = np.random.default_rng(42)
    previous = random.uniform((-3.0, -2.0), (4.0, 3.0), size=(200, 2))
    expected_angle = 0.03
    expected_translation = np.array((0.05, -0.01))
    rotation = np.array((
        (cos(expected_angle), -sin(expected_angle)),
        (sin(expected_angle), cos(expected_angle)),
    ))
    current_static = (previous - expected_translation) @ rotation

    # This wall-like cluster appears only in the current scan.  It represents
    # either a pedestrian surface or a newly introduced static obstacle.
    obstacle_previous_frame = np.column_stack((
        np.full(80, 1.0),
        np.linspace(-0.8, 0.8, 80),
    ))
    obstacle_current_frame = (
        obstacle_previous_frame - expected_translation
    ) @ rotation
    current = np.vstack((current_static, obstacle_current_frame))
    matcher = PlanarScanMatcher(ScanMatcherConfig(
        max_points=300,
        trim_fraction=0.95,
        max_rmse_m=0.08,
    ))

    result = matcher.match(previous, current)

    assert result is not None
    assert result.translation_previous_frame == pytest.approx(
        expected_translation, abs=0.01
    )
    assert result.rotation_rad == pytest.approx(expected_angle, abs=0.01)
    assert result.rmse_m < 0.02
    assert result.inlier_ratio == pytest.approx(200.0 / 280.0)


def test_robust_match_rejects_a_pedestrian_moving_across_adjacent_scans():
    """A transient cluster present in several frames remains an outlier."""
    front_wall = np.column_stack((
        np.full(100, 2.5),
        np.linspace(-2.5, 2.5, 100),
    ))
    side_wall = np.column_stack((
        np.linspace(2.5, -2.5, 100),
        np.full(100, 2.5),
    ))
    static_points = np.vstack((front_wall, side_wall))
    old_pedestrian = np.column_stack((
        np.full(80, 1.0),
        np.linspace(-0.8, 0.8, 80),
    ))
    previous = np.vstack((static_points, old_pedestrian))
    expected_angle = 0.03
    expected_translation = np.array((0.05, -0.01))
    rotation = np.array((
        (cos(expected_angle), -sin(expected_angle)),
        (sin(expected_angle), cos(expected_angle)),
    ))
    current_static = (static_points - expected_translation) @ rotation
    moved_pedestrian = np.column_stack((
        np.full(80, 1.3),
        np.linspace(-0.8, 0.8, 80),
    ))
    current = np.vstack((
        current_static,
        (moved_pedestrian - expected_translation) @ rotation,
    ))

    result = PlanarScanMatcher(ScanMatcherConfig(
        max_points=400,
        trim_fraction=0.80,
        max_rmse_m=0.08,
    )).match(previous, current)

    assert result is not None
    assert result.translation_previous_frame == pytest.approx(
        expected_translation, abs=0.01
    )
    assert result.rotation_rad == pytest.approx(expected_angle, abs=0.01)
    assert result.rmse_m < 0.02
    assert result.inlier_ratio > 0.55


@pytest.mark.parametrize(
    'kwargs',
    [
        {'min_points': 2},
        {'trim_fraction': 0.0},
        {'min_inlier_ratio': 1.1},
        {'robust_mad_scale': 0.0},
        {'robust_residual_floor_m': 0.0},
        {'max_rmse_m': 0.0},
    ],
)
def test_invalid_matcher_configuration_is_rejected(kwargs):
    """Invalid quality gates cannot silently weaken matching."""
    with pytest.raises(ValueError):
        ScanMatcherConfig(**kwargs)
