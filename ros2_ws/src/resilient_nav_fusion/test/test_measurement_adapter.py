"""Focused message-boundary tests for the Phase 8 measurement adapter."""

from copy import deepcopy
from math import isnan, nan

from nav_msgs.msg import Odometry
from resilient_nav_interfaces.msg import SensorHealth
from sensor_msgs.msg import Imu

from resilient_nav_fusion.fusion_policy import (
    FusionPolicy,
    FusionPolicyConfig,
    HealthState,
)
from resilient_nav_fusion.measurement_adapter import (
    IMU_ANGULAR_Z_COVARIANCE_INDEX,
    WHEEL_LINEAR_X_COVARIANCE_INDEX,
    WHEEL_YAW_RATE_COVARIANCE_INDEX,
    adapt_imu_measurement,
    adapt_wheel_measurements,
    fusion_status_from_decision,
    measurement_health_from_sensor,
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
    assert list(wheel_output.twist.covariance[1:]) == list(
        wheel_before.twist.covariance[1:]
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
