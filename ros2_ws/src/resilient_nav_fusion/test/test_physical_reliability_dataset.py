"""Unit tests for the offline physical-reliability dataset contract."""

import numpy as np
import pytest

from resilient_nav_fusion.physical_reliability_dataset import (
    DatasetConfig,
    FEATURE_COLUMNS,
    aggregate_residual,
    ground_truth_motion,
    scan_observability,
)


def test_window_contract_rejects_sizes_outside_declared_range():
    with pytest.raises(ValueError, match='0.3-0.5'):
        DatasetConfig(window_sec=0.2)


def test_residual_aggregation_preserves_sign_and_persistence():
    stats = aggregate_residual([-0.2, 0.1, 0.0, 0.3], threshold=0.15)

    assert stats['mean'] == pytest.approx(0.05)
    assert stats['abs_mean'] == pytest.approx(0.15)
    assert stats['abs_max'] == pytest.approx(0.3)
    assert stats['persistence'] == pytest.approx(0.5)


def test_independent_pose_derivative_returns_body_motion():
    poses = [
        (index * 0.02, index * 0.02 * 0.25, 0.0, 0.0)
        for index in range(30)
    ]

    truth = ground_truth_motion(poses, derivative_radius_sec=0.05)

    assert np.median(truth.vx_body) == pytest.approx(0.25, abs=1.0e-8)
    assert np.max(np.abs(truth.yaw_rate)) == pytest.approx(0.0, abs=1.0e-8)


def test_scan_observability_distinguishes_one_wall_from_two_axes():
    horizontal = np.column_stack((np.linspace(-2.0, 2.0, 100), np.ones(100)))
    vertical = np.column_stack((np.ones(100), np.linspace(1.0, 3.0, 100)))
    corner = np.vstack((horizontal, vertical))

    assert scan_observability(horizontal) < 0.02
    assert scan_observability(corner) > 0.20


def test_feature_allowlist_cannot_contain_ground_truth_or_scenario_answers():
    normalized = ' '.join(FEATURE_COLUMNS).lower()

    for forbidden in ('ground_truth', 'scenario', 'fault', 'parameters_yaml'):
        assert forbidden not in normalized
