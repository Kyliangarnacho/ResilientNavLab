"""Focused message-boundary tests for the Phase 8 measurement adapter."""

from copy import deepcopy
from math import isnan, nan

import pytest

from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from resilient_nav_interfaces.msg import SensorHealth
from sensor_msgs.msg import Imu

from resilient_nav_fusion.fusion_policy import (
    FusionPolicy,
    FusionPolicyConfig,
    HealthState,
    ReliabilityScores,
)
from resilient_nav_fusion.measurement_adapter import (
    IMU_ANGULAR_Z_COVARIANCE_INDEX,
    WHEEL_LINEAR_X_COVARIANCE_INDEX,
    WHEEL_YAW_POSE_COVARIANCE_INDEX,
    WHEEL_YAW_RATE_COVARIANCE_INDEX,
    adapt_lidar_measurement,
    adapt_imu_measurement,
    adapt_wheel_measurements,
    fusion_status_from_decision,
    imu_measurement_health_from_sensor,
    lidar_admission_from_measurement,
    measurement_health_from_sensor,
    provisional_imu_health,
    provisional_wheel_health,
    sensor_health_has_observation,
)


def wheel_odometry():
    """Build wheel odometry with distinct covariance entries for index checks."""
    message = Odometry()
    message.header.stamp.sec = 12
    message.header.stamp.nanosec = 345
    message.header.frame_id = 'odom'
    message.child_frame_id = 'base_footprint'
    message.twist.twist.linear.x = 0.4
    message.twist.twist.angular.z = 0.3
    message.pose.covariance = [float(index + 101) for index in range(36)]
    message.twist.covariance = [float(index + 1) for index in range(36)]
    return message


def imu_message():
    """Build IMU data with distinct angular-velocity covariance entries."""
    message = Imu()
    message.header.stamp.sec = 7
    message.header.frame_id = 'imu_link'
    message.angular_velocity.z = 0.6
    message.angular_velocity_covariance = [float(index + 1) for index in range(9)]
    return message


def lidar_velocity():
    """Build a scan-matched forward-velocity message."""
    message = TwistWithCovarianceStamped()
    message.header.stamp.sec = 8
    message.header.frame_id = 'lidar_link'
    message.twist.twist.linear.x = 0.25
    message.twist.covariance[WHEEL_LINEAR_X_COVARIANCE_INDEX] = 0.04
    return message


def sensor_health(state, confidence=1.0):
    """Build a health message including forbidden transport metadata to discard."""
    message = SensorHealth()
    message.header.stamp.sec = 9
    message.sensor = 'wheel'
    message.source_topic = '/faulted/wheel/odometry'
    message.state = state
    message.confidence = confidence
    message.window_start.sec = 4
    message.window_end.sec = 9
    message.sample_count = 6
    return message


def test_degraded_outputs_inflate_only_actual_measurement_covariances():
    policy = FusionPolicy(
        FusionPolicyConfig(
            wheel_degraded_covariance_scale=4.0,
            imu_degraded_covariance_scale=5.0,
        )
    )
    wheel_decision = policy.decide(HealthState.DEGRADED, HealthState.HEALTHY)
    wheel_source = wheel_odometry()
    wheel_before = deepcopy(wheel_source)
    wheel_output, fallback = adapt_wheel_measurements(wheel_source, wheel_decision)

    assert wheel_output is not None
    assert fallback is None
    assert wheel_output.twist.covariance[WHEEL_LINEAR_X_COVARIANCE_INDEX] == 4.0
    assert wheel_output.pose.covariance[WHEEL_YAW_POSE_COVARIANCE_INDEX] == 544.0
    assert list(wheel_output.twist.covariance[1:]) == list(
        wheel_before.twist.covariance[1:]
    )
    assert list(wheel_output.pose.covariance[:-1]) == list(
        wheel_before.pose.covariance[:-1]
    )
    assert wheel_source == wheel_before

    imu_decision = FusionPolicy(
        FusionPolicyConfig(imu_degraded_covariance_scale=5.0)
    ).decide(HealthState.HEALTHY, HealthState.DEGRADED)
    imu_source = imu_message()
    imu_before = deepcopy(imu_source)
    imu_output = adapt_imu_measurement(imu_source, imu_decision)

    assert imu_output is not None
    assert imu_output.angular_velocity_covariance[
        IMU_ANGULAR_Z_COVARIANCE_INDEX
    ] == 45.0
    assert list(imu_output.angular_velocity_covariance[:-1]) == list(
        imu_before.angular_velocity_covariance[:-1]
    )
    assert imu_source == imu_before


def test_rf_weights_wheel_translation_and_rotation_covariance_independently():
    decision = FusionPolicy().decide(
        HealthState.HEALTHY,
        HealthState.HEALTHY,
        HealthState.HEALTHY,
        ReliabilityScores(
            wheel_translation=0.50,
            wheel_rotation=0.95,
            imu_yaw_rate=0.95,
            lidar_translation=0.90,
            rf_ready=True,
        ),
    )

    output, fallback = adapt_wheel_measurements(wheel_odometry(), decision)

    assert fallback is None
    assert output.twist.covariance[WHEEL_LINEAR_X_COVARIANCE_INDEX] == pytest.approx(3.61)
    assert output.pose.covariance[WHEEL_YAW_POSE_COVARIANCE_INDEX] == 136.0


def test_fault_and_unknown_decisions_suppress_primary_measurements():
    wheel_source = wheel_odometry()
    imu_source = imu_message()

    fault = FusionPolicy().decide(HealthState.FAULT, HealthState.FAULT)
    assert adapt_wheel_measurements(wheel_source, fault) == (None, None)
    assert adapt_imu_measurement(imu_source, fault) is None

    unknown = FusionPolicy().decide(HealthState.UNKNOWN, HealthState.UNKNOWN)
    assert adapt_wheel_measurements(wheel_source, unknown) == (None, None)
    assert adapt_imu_measurement(imu_source, unknown) is None


def test_imu_failure_creates_isolated_wheel_yaw_rate_fallback():
    source = wheel_odometry()
    before = deepcopy(source)
    decision = FusionPolicy().decide(HealthState.HEALTHY, HealthState.FAULT)
    wheel_output, fallback = adapt_wheel_measurements(source, decision)

    assert wheel_output is not None
    assert fallback is not None
    assert fallback.header.stamp == source.header.stamp
    assert fallback.header.frame_id == source.child_frame_id
    assert fallback.twist.twist.angular.z == source.twist.twist.angular.z
    assert fallback.twist.covariance[WHEEL_YAW_RATE_COVARIANCE_INDEX] == 288.0
    assert list(fallback.twist.covariance[:-1]) == list(
        source.twist.covariance[:-1]
    )
    assert source == before


def test_wheel_failure_selects_lidar_velocity_without_mutation():
    source = lidar_velocity()
    before = deepcopy(source)
    decision = FusionPolicy(
        FusionPolicyConfig(lidar_degraded_covariance_scale=5.0)
    ).decide(
        HealthState.FAULT,
        HealthState.HEALTHY,
        HealthState.DEGRADED,
    )

    output = adapt_lidar_measurement(source, decision)

    assert output is not None
    assert output.twist.twist.linear.x == 0.25
    assert output.twist.covariance[WHEEL_LINEAR_X_COVARIANCE_INDEX] == 0.20
    assert source == before


def test_lidar_velocity_is_suppressed_while_wheel_is_healthy():
    decision = FusionPolicy().decide(
        HealthState.HEALTHY,
        HealthState.HEALTHY,
        HealthState.HEALTHY,
    )

    assert adapt_lidar_measurement(lidar_velocity(), decision) is None


def test_illegal_corresponding_covariances_fail_closed_without_mutation():
    wheel = wheel_odometry()
    wheel.twist.covariance[WHEEL_LINEAR_X_COVARIANCE_INDEX] = nan
    wheel_before = deepcopy(wheel)
    primary, fallback = adapt_wheel_measurements(
        wheel,
        FusionPolicy().decide(HealthState.HEALTHY, HealthState.HEALTHY),
    )
    assert primary is None
    assert fallback is None
    assert wheel.header == wheel_before.header
    assert wheel.child_frame_id == wheel_before.child_frame_id
    assert isnan(wheel.twist.covariance[WHEEL_LINEAR_X_COVARIANCE_INDEX])
    assert list(wheel.twist.covariance[1:]) == list(
        wheel_before.twist.covariance[1:]
    )

    fallback_source = wheel_odometry()
    fallback_source.twist.covariance[WHEEL_YAW_RATE_COVARIANCE_INDEX] = -1.0
    primary, fallback = adapt_wheel_measurements(
        fallback_source,
        FusionPolicy().decide(HealthState.HEALTHY, HealthState.FAULT),
    )
    assert primary is not None
    assert fallback is None

    invalid_yaw_pose = wheel_odometry()
    invalid_yaw_pose.pose.covariance[WHEEL_YAW_POSE_COVARIANCE_INDEX] = nan
    primary, fallback = adapt_wheel_measurements(
        invalid_yaw_pose,
        FusionPolicy().decide(HealthState.HEALTHY, HealthState.HEALTHY),
    )
    assert primary is None
    assert fallback is None

    imu = imu_message()
    imu.angular_velocity_covariance[IMU_ANGULAR_Z_COVARIANCE_INDEX] = -1.0
    assert adapt_imu_measurement(
        imu,
        FusionPolicy().decide(HealthState.HEALTHY, HealthState.HEALTHY),
    ) is None

    unknown_imu_covariance = imu_message()
    unknown_imu_covariance.angular_velocity_covariance = [0.0] * 9
    assert adapt_imu_measurement(
        unknown_imu_covariance,
        FusionPolicy().decide(HealthState.HEALTHY, HealthState.HEALTHY),
    ) is None

    unavailable_imu = imu_message()
    unavailable_imu.angular_velocity_covariance[0] = -1.0
    assert adapt_imu_measurement(
        unavailable_imu,
        FusionPolicy().decide(HealthState.HEALTHY, HealthState.HEALTHY),
    ) is None


def test_policy_reasons_transfer_structurally_to_fusion_status():
    decision = FusionPolicy().decide(HealthState.HEALTHY, HealthState.FAULT)
    health = sensor_health(SensorHealth.FAULT)
    status = fusion_status_from_decision(decision, health)

    assert tuple(status.reasons) == decision.reasons
    assert status.accepted_measurements == list(decision.accepted_measurements)
    assert status.rejected_measurements == list(decision.rejected_measurements)
    assert status.header == health.header
    assert status.window_start == health.window_start
    assert status.window_end == health.window_end


def test_sensor_health_allowlist_drops_source_topic_and_rejects_bad_confidence():
    health = sensor_health(SensorHealth.DEGRADED, confidence=0.8)
    sanitized = measurement_health_from_sensor(health)

    assert sanitized.state is HealthState.DEGRADED
    assert sanitized.confidence == 0.8
    assert not hasattr(sanitized, 'source_topic')
    assert measurement_health_from_sensor(
        sensor_health(SensorHealth.HEALTHY, confidence=float('nan'))
    ).state is HealthState.UNKNOWN


def test_imu_remains_degraded_usable_when_only_wheel_cross_check_is_unavailable():
    health = sensor_health(SensorHealth.UNKNOWN, confidence=0.9)
    health.sensor = 'imu'
    health.detected_fault = 'none'
    health.reasons = ['wheel_reference_unavailable']

    sanitized = imu_measurement_health_from_sensor(health)
    decision = FusionPolicy().decide(
        HealthState.FAULT,
        sanitized,
        HealthState.HEALTHY,
    )

    assert sanitized.state is HealthState.DEGRADED
    assert sanitized.confidence == 0.9
    assert FusionPolicy.IMU_YAW_RATE in decision.accepted_measurements
    assert FusionPolicy.LIDAR_VELOCITY in decision.accepted_measurements
    assert FusionPolicy.WHEEL_VELOCITY in decision.rejected_measurements


def test_imu_baseline_calibration_is_provisional_at_startup():
    health = sensor_health(SensorHealth.UNKNOWN, confidence=0.9)
    health.sensor = 'imu'
    health.detected_fault = 'unknown'
    health.reasons = ['imu_yaw_rate_bias_baseline_calibrating']

    assert (
        imu_measurement_health_from_sensor(health).state
        is HealthState.PROVISIONAL
    )


def test_only_exact_wheel_or_imu_startup_reason_is_provisional():
    health = sensor_health(SensorHealth.UNKNOWN, confidence=0.5)
    health.sensor = 'wheel'
    health.detected_fault = 'none'
    health.reasons = ['startup_provisional_valid']

    assert measurement_health_from_sensor(health).state is HealthState.PROVISIONAL

    health.sensor = 'lidar'
    assert measurement_health_from_sensor(health).state is HealthState.UNKNOWN

    health.sensor = 'wheel'
    health.detected_fault = 'freeze'
    assert measurement_health_from_sensor(health).state is HealthState.UNKNOWN


def test_no_message_timer_does_not_disable_raw_startup_admission():
    no_data = sensor_health(SensorHealth.UNKNOWN, confidence=0.0)
    no_data.sample_count = 0
    no_data.reasons = ['no_messages_received']
    first_sample = sensor_health(SensorHealth.UNKNOWN, confidence=0.5)
    first_sample.sample_count = 1
    first_sample.reasons = ['startup_provisional_valid']

    assert sensor_health_has_observation(no_data) is False
    assert sensor_health_has_observation(first_sample) is True


def test_raw_startup_checks_accept_legal_wheel_and_imu_without_health_status():
    wheel = provisional_wheel_health(wheel_odometry(), 12.0, 0.5, 0.05)
    imu = provisional_imu_health(imu_message(), 7.0, 0.5, 0.05)

    assert wheel is not None
    assert wheel.state is HealthState.PROVISIONAL
    assert imu is not None
    assert imu.state is HealthState.PROVISIONAL


def test_raw_startup_checks_reject_stale_future_and_malformed_measurements():
    stale_wheel = wheel_odometry()
    future_imu = imu_message()
    invalid_wheel = wheel_odometry()
    invalid_wheel.twist.twist.linear.x = float('nan')

    assert provisional_wheel_health(stale_wheel, 13.0, 0.5, 0.05) is None
    assert provisional_imu_health(future_imu, 6.0, 0.5, 0.05) is None
    assert provisional_wheel_health(invalid_wheel, 12.0, 0.5, 0.05) is None


def test_lidar_fallback_uses_local_post_icp_admission_not_sensor_health():
    valid = lidar_admission_from_measurement(
        lidar_velocity(), 8.0, 0.5, 0.05
    )
    stale = lidar_admission_from_measurement(
        lidar_velocity(), 9.0, 0.5, 0.05
    )
    malformed = lidar_velocity()
    malformed.twist.twist.linear.y = float('nan')

    assert valid is not None
    assert valid.state is HealthState.HEALTHY
    assert stale is None
    assert lidar_admission_from_measurement(
        malformed, 8.0, 0.5, 0.05
    ) is None
