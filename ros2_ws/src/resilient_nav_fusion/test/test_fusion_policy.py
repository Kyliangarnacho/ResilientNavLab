"""Unit tests for the ROS-free Phase 8 health-aware fusion policy."""

import pytest

from resilient_nav_fusion.fusion_policy import (
    FusionPolicy,
    FusionPolicyConfig,
    FusionState,
    HealthState,
    MeasurementHealth,
    ReliabilityScores,
)


def decision(wheel, imu, policy=None):
    """Build or reuse a policy and return its next deterministic decision."""
    return (policy or FusionPolicy()).decide(wheel, imu)


def test_healthy_measurements_are_nominal_with_unit_covariance():
    result = decision(HealthState.HEALTHY, HealthState.HEALTHY)

    assert result.state is FusionState.NOMINAL
    assert result.accepted_measurements == ('wheel_velocity', 'imu_yaw_rate')
    assert result.rejected_measurements == (
        'lidar_velocity', 'wheel_yaw_rate'
    )
    assert result.covariance_scales == {
        'wheel_velocity': 1.0,
        'wheel_rotation': 1.0,
        'lidar_velocity': None,
        'imu_yaw_rate': 1.0,
        'wheel_yaw_rate': None,
    }
    assert not result.wheel_yaw_fallback_enabled
    assert result.confidence == 1.0


def test_degraded_measurement_uses_only_explicit_covariance_scale():
    config = FusionPolicyConfig(imu_degraded_covariance_scale=6.0)
    result = decision(HealthState.HEALTHY, HealthState.DEGRADED, FusionPolicy(config))

    assert result.state is FusionState.DEGRADED
    assert result.accepted_measurements == ('wheel_velocity', 'imu_yaw_rate')
    assert result.covariance_scales['wheel_velocity'] == 1.0
    assert result.covariance_scales['imu_yaw_rate'] == 6.0
    assert 'imu_yaw_rate_accepted_degraded' in result.reasons
    assert result.confidence == pytest.approx(0.75)


def test_imu_fault_enables_wheel_yaw_fallback_immediately():
    result = decision(HealthState.HEALTHY, HealthState.FAULT)

    assert result.state is FusionState.DEGRADED
    assert result.accepted_measurements == ('wheel_velocity', 'wheel_yaw_rate')
    assert result.rejected_measurements == ('lidar_velocity', 'imu_yaw_rate')
    assert result.covariance_scales['imu_yaw_rate'] is None
    assert result.covariance_scales['wheel_yaw_rate'] == 8.0
    assert result.wheel_yaw_fallback_enabled
    assert result.confidence == pytest.approx(0.5)


def test_wheel_fault_keeps_only_imu_yaw_rate_without_fallback():
    result = decision(HealthState.FAULT, HealthState.HEALTHY)

    assert result.state is FusionState.DEGRADED
    assert result.accepted_measurements == ('imu_yaw_rate',)
    assert result.rejected_measurements == (
        'wheel_velocity', 'lidar_velocity', 'wheel_yaw_rate'
    )
    assert result.covariance_scales['wheel_velocity'] is None
    assert not result.wheel_yaw_fallback_enabled
    assert result.confidence == pytest.approx(0.5)


def test_both_faults_enter_hold_without_covariance_scales():
    result = decision(HealthState.FAULT, HealthState.FAULT)

    assert result.state is FusionState.HOLD
    assert not result.accepted_measurements
    assert result.rejected_measurements == (
        'wheel_velocity', 'lidar_velocity', 'imu_yaw_rate', 'wheel_yaw_rate'
    )
    assert all(scale is None for scale in result.covariance_scales.values())
    assert result.confidence == 0.0


def test_wheel_fault_uses_lidar_translation_and_keeps_imu_yaw_rate():
    result = FusionPolicy().decide(
        HealthState.FAULT,
        HealthState.HEALTHY,
        HealthState.HEALTHY,
    )

    assert result.state is FusionState.DEGRADED
    assert result.accepted_measurements == ('lidar_velocity', 'imu_yaw_rate')
    assert result.rejected_measurements == (
        'wheel_velocity', 'wheel_yaw_rate'
    )
    assert result.lidar_translation_fallback_enabled
    assert not result.wheel_yaw_fallback_enabled
    assert 'lidar_velocity_accepted_healthy' in result.reasons


def test_lidar_is_inactive_while_wheel_is_usable():
    result = FusionPolicy().decide(
        HealthState.HEALTHY,
        HealthState.HEALTHY,
        HealthState.FAULT,
    )

    assert result.state is FusionState.NOMINAL
    assert result.accepted_measurements == ('wheel_velocity', 'imu_yaw_rate')
    assert not result.lidar_translation_fallback_enabled
    assert 'lidar_velocity_inactive_wheel_available' in result.reasons


def test_both_unknowns_remain_unknown_and_fail_closed():
    result = decision(HealthState.UNKNOWN, HealthState.UNKNOWN)

    assert result.state is FusionState.UNKNOWN
    assert not result.accepted_measurements
    assert any('rejected_unknown' in reason for reason in result.reasons)
    assert result.confidence == 0.0


def test_startup_provisional_measurements_bypass_recovery_confirmation():
    policy = FusionPolicy(FusionPolicyConfig(recovery_confirmation_cycles=2))
    policy.decide(HealthState.UNKNOWN, HealthState.UNKNOWN)

    provisional = policy.decide(
        MeasurementHealth(HealthState.PROVISIONAL, confidence=0.5),
        MeasurementHealth(HealthState.PROVISIONAL, confidence=0.5),
    )
    confirmed = policy.decide(HealthState.HEALTHY, HealthState.HEALTHY)

    assert provisional.state is FusionState.DEGRADED
    assert provisional.accepted_measurements == (
        'wheel_velocity', 'imu_yaw_rate'
    )
    assert provisional.covariance_scales['wheel_velocity'] == 1.0
    assert provisional.covariance_scales['imu_yaw_rate'] == 1.0
    assert 'wheel_velocity_accepted_provisional' in provisional.reasons
    assert confirmed.state is FusionState.NOMINAL


def test_first_confirmed_health_after_unknown_is_not_fault_recovery():
    policy = FusionPolicy(FusionPolicyConfig(recovery_confirmation_cycles=2))
    policy.decide(HealthState.UNKNOWN, HealthState.UNKNOWN)
    policy.decide(HealthState.UNKNOWN, HealthState.UNKNOWN)

    confirmed = policy.decide(HealthState.HEALTHY, HealthState.HEALTHY)

    assert confirmed.state is FusionState.NOMINAL
    assert confirmed.accepted_measurements == (
        'wheel_velocity', 'imu_yaw_rate'
    )


def test_recovery_requires_consecutive_healthy_cycles_after_fault():
    policy = FusionPolicy(FusionPolicyConfig(recovery_confirmation_cycles=2))
    assert policy.decide(HealthState.HEALTHY, HealthState.FAULT).wheel_yaw_fallback_enabled

    first_recovery = policy.decide(HealthState.HEALTHY, HealthState.HEALTHY)
    assert first_recovery.state is FusionState.DEGRADED
    assert first_recovery.accepted_measurements == (
        'wheel_velocity', 'wheel_yaw_rate'
    )
    assert 'imu_yaw_rate_recovery_pending' in first_recovery.reasons

    recovered = policy.decide(HealthState.HEALTHY, HealthState.HEALTHY)
    assert recovered.state is FusionState.NOMINAL
    assert recovered.accepted_measurements == ('wheel_velocity', 'imu_yaw_rate')
    assert not recovered.wheel_yaw_fallback_enabled


def test_rapid_fault_healthy_jitter_never_recovers_on_one_sample():
    policy = FusionPolicy(FusionPolicyConfig(recovery_confirmation_cycles=2))
    policy.decide(HealthState.HEALTHY, HealthState.HEALTHY)
    policy.decide(HealthState.HEALTHY, HealthState.FAULT)

    assert policy.decide(HealthState.HEALTHY, HealthState.HEALTHY).wheel_yaw_fallback_enabled
    assert policy.decide(HealthState.HEALTHY, HealthState.FAULT).wheel_yaw_fallback_enabled
    assert policy.decide(HealthState.HEALTHY, HealthState.HEALTHY).wheel_yaw_fallback_enabled
    assert policy.decide(HealthState.HEALTHY, HealthState.HEALTHY).state is FusionState.NOMINAL


def test_measurement_health_and_config_validate_input_ranges():
    with pytest.raises(ValueError, match='confidence'):
        MeasurementHealth(HealthState.HEALTHY, confidence=1.1)
    with pytest.raises(ValueError, match='covariance'):
        FusionPolicyConfig(wheel_degraded_covariance_scale=0.9)
    with pytest.raises(ValueError, match='covariance'):
        FusionPolicyConfig(lidar_degraded_covariance_scale=0.9)
    with pytest.raises(ValueError, match='confirmation'):
        FusionPolicyConfig(recovery_confirmation_cycles=0)


def test_rf_probability_continuously_scales_independent_components():
    result = FusionPolicy().decide(
        HealthState.HEALTHY,
        HealthState.HEALTHY,
        HealthState.HEALTHY,
        ReliabilityScores(
            wheel_translation=0.50,
            wheel_rotation=0.80,
            imu_yaw_rate=0.95,
            lidar_translation=0.90,
            rf_ready=True,
        ),
    )

    assert result.covariance_scales['wheel_velocity'] == pytest.approx(3.61)
    assert result.covariance_scales['wheel_rotation'] == pytest.approx(1.41015625)
    assert result.covariance_scales['imu_yaw_rate'] == 1.0
    assert not result.lidar_translation_fallback_enabled
    assert result.state is FusionState.DEGRADED


def test_high_rf_reliability_keeps_nominal_state_with_continuous_scaling():
    """Small non-unit scales are not a navigation-level degradation."""
    result = FusionPolicy().decide(
        HealthState.HEALTHY,
        HealthState.HEALTHY,
        HealthState.HEALTHY,
        ReliabilityScores(
            wheel_translation=0.85,
            wheel_rotation=0.90,
            imu_yaw_rate=0.88,
            lidar_translation=0.90,
            rf_ready=True,
        ),
    )

    assert result.covariance_scales['wheel_velocity'] > 1.0
    assert result.covariance_scales['wheel_rotation'] > 1.0
    assert result.covariance_scales['imu_yaw_rate'] > 1.0
    assert result.state is FusionState.NOMINAL


def test_reliability_below_nominal_state_threshold_remains_degraded():
    """The status tolerance must not hide meaningfully weak evidence."""
    result = FusionPolicy().decide(
        HealthState.HEALTHY,
        HealthState.HEALTHY,
        HealthState.HEALTHY,
        ReliabilityScores(
            wheel_translation=0.79,
            wheel_rotation=0.95,
            imu_yaw_rate=0.95,
            lidar_translation=0.90,
            rf_ready=True,
        ),
    )

    assert result.state is FusionState.DEGRADED


def test_extremely_low_rf_score_uses_fallback_without_validation_threshold():
    result = FusionPolicy().decide(
        HealthState.HEALTHY,
        HealthState.HEALTHY,
        HealthState.HEALTHY,
        ReliabilityScores(
            wheel_translation=0.08,
            wheel_rotation=0.90,
            imu_yaw_rate=0.04,
            lidar_translation=0.80,
            rf_ready=True,
        ),
    )

    assert result.lidar_translation_fallback_enabled
    assert result.wheel_yaw_fallback_enabled
    assert 'lidar_velocity' in result.accepted_measurements
    assert 'imu_yaw_rate' in result.rejected_measurements


def test_warming_window_is_neutral_and_does_not_delay_healthy_startup():
    result = FusionPolicy().decide(
        HealthState.PROVISIONAL,
        HealthState.PROVISIONAL,
        HealthState.UNKNOWN,
        ReliabilityScores(rf_ready=False),
    )

    assert result.covariance_scales['wheel_velocity'] == 1.0
    assert result.covariance_scales['wheel_rotation'] == 1.0
    assert result.covariance_scales['imu_yaw_rate'] == 1.0
    assert 'rf_window_warming_neutral_weight' in result.reasons
