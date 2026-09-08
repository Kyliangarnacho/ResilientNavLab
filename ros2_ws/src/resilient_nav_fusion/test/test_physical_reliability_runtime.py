"""Focused tests for online schema-v2 RF reliability inference."""

from pathlib import Path

import pytest

from resilient_nav_fusion.physical_reliability_runtime import (
    FEATURE_COLUMNS,
    ImuSample,
    LidarSample,
    OnlineFeatureWindow,
    ReliabilityModelBundle,
    WheelSample,
    lidar_quality_reliability,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = PACKAGE_ROOT / 'models' / 'physical_reliability_rf_v2'


def populated_window():
    window = OnlineFeatureWindow()
    for index in range(21):
        stamp = index * 0.02
        window.add_wheel(WheelSample(stamp, 0.22, 0.10, 0.0025))
    for index in range(41):
        stamp = index * 0.01
        window.add_imu(ImuSample(stamp, 0.095, 0.03, 0.01, 0.002, 0.003))
    for index in range(1, 5):
        stamp = index * 0.10
        window.add_lidar(LidarSample(
            stamp,
            True,
            0.21,
            0.09,
            0.0030,
            0.025,
            0.78,
            0.24,
        ))
    return window


def test_online_window_matches_exact_model_feature_contract():
    features = populated_window().feature_vector(0.4)

    assert features is not None
    assert tuple(features) == FEATURE_COLUMNS
    assert len(features) == 44
    assert features['wheel_lidar_vx_residual_abs_mean'] == pytest.approx(0.01)
    assert features['icp_valid_fraction'] == 1.0


def test_selected_joblib_bundle_outputs_three_continuous_probabilities():
    features = populated_window().feature_vector(0.4)
    scores = ReliabilityModelBundle(MODEL_DIR).predict(features, 0.85)

    assert scores.rf_ready
    assert 0.0 <= scores.wheel_translation <= 1.0
    assert 0.0 <= scores.wheel_rotation <= 1.0
    assert 0.0 <= scores.imu_yaw_rate <= 1.0
    assert scores.lidar_translation == 0.85


def test_lidar_quality_remains_an_independent_gate():
    good = LidarSample(1.0, True, rmse_m=0.02, inlier_ratio=0.80, observability=0.25)
    poor = LidarSample(1.0, True, rmse_m=0.09, inlier_ratio=0.80, observability=0.25)

    assert lidar_quality_reliability(
        good, max_rmse_m=0.08, min_inlier_ratio=0.45, min_observability=0.05
    ) > 0.5
    assert lidar_quality_reliability(
        poor, max_rmse_m=0.08, min_inlier_ratio=0.45, min_observability=0.05
    ) == 0.0


def test_runtime_module_contains_no_experiment_answer_inputs():
    source = (
        PACKAGE_ROOT
        / 'resilient_nav_fusion'
        / 'physical_reliability_runtime.py'
    ).read_text(encoding='utf-8').lower()

    for forbidden in (
        'faultstatus',
        'scenario_id',
        'scenario_seed',
        'parameters_yaml',
        'ground_truth',
        'label_audit',
    ):
        assert forbidden not in source
