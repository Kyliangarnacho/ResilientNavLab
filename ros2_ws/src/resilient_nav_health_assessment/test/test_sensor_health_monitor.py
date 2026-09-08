from pathlib import Path

from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import QoSReliabilityPolicy
from resilient_nav_health_assessment.sensor_health_monitor import (
    HealthEvaluator,
    make_sensor_health,
    make_unknown_sensor_health,
    SensorHealthMonitor,
)
from resilient_nav_interfaces.msg import SensorHealth
from sensor_msgs.msg import Imu, LaserScan


def make_evaluator(**overrides):
    parameters = {
        'window_duration_sec': 5.0,
        'min_samples': 3,
        'stale_timeout_sec': 0.5,
        'delay_warning_sec': 0.3,
        'delay_fault_sec': 0.5,
        'delay_confirmation_cycles': 3,
        'command_linear_threshold_mps': 0.05,
        'command_angular_threshold_rad_s': 0.1,
        'freeze_duration_sec': 1.0,
        'wheel_pose_span_threshold_m': 0.01,
        'wheel_linear_span_threshold_mps': 0.01,
        'wheel_angular_span_threshold_rad_s': 0.02,
    }
    parameters.update(overrides)
    return HealthEvaluator(**parameters)


def add_imu_samples(evaluator, received_times, stamp_offset=0.0):
    for received_sec in received_times:
        evaluator.add_imu(received_sec, received_sec - stamp_offset)


def add_wheel_samples(evaluator, samples):
    for received_sec, yaw_rad, linear_x, angular_z in samples:
        evaluator.add_wheel(
            received_sec,
            received_sec,
            pose_x=0.0,
            pose_y=0.0,
            yaw_rad=yaw_rad,
            linear_x=linear_x,
            angular_z=angular_z,
        )


def add_static_wheel_samples(evaluator, received_times):
    add_wheel_samples(
        evaluator,
        [(received_sec, 0.0, 0.0, 0.0) for received_sec in received_times],
    )


def add_scan_samples(
    evaluator, received_times, ranges, angle_increment_rad=0.01, stamp_offset=0.0,
):
    for received_sec in received_times:
        evaluator.add_scan(
            received_sec,
            received_sec - stamp_offset,
            ranges,
            angle_increment_rad,
        )


def scan_ranges(total_beams, nan_indices=(), inf_indices=()):
    ranges = [1.0] * total_beams
    for index in nan_indices:
        ranges[index] = float('nan')
    for index in inf_indices:
        ranges[index] = float('inf')
    return ranges


def add_imu_yaw_rate_samples(evaluator, samples):
    for received_sec, stamp_sec, angular_z in samples:
        evaluator.add_imu(received_sec, stamp_sec, angular_z)


def add_wheel_yaw_rate_samples(evaluator, samples):
    for received_sec, stamp_sec, angular_z in samples:
        evaluator.add_wheel(
            received_sec,
            stamp_sec,
            pose_x=0.0,
            pose_y=0.0,
            yaw_rad=0.0,
            linear_x=0.0,
            angular_z=angular_z,
        )


def paired_yaw_rate_samples(
    imu_z, wheel_z, count=10, stamp_offset=0.0, start_sec=1.0,
):
    return [
        (
            start_sec + index * 0.04,
            start_sec + index * 0.04 + stamp_offset,
            imu_z,
        )
        for index in range(count)
    ], [
        (start_sec + index * 0.04, start_sec + index * 0.04, wheel_z)
        for index in range(count)
    ]


def test_sensor_health_constants_are_generated():
    assert SensorHealth.UNKNOWN == 0
    assert SensorHealth.HEALTHY == 1
    assert SensorHealth.DEGRADED == 2
    assert SensorHealth.FAULT == 3


def test_make_unknown_sensor_health_message():
    stamp = Time(sec=12, nanosec=345)

    msg = make_unknown_sensor_health('imu', '/imu/data', stamp)

    assert msg.header.stamp == stamp
    assert msg.header.frame_id == ''
    assert msg.sensor == 'imu'
    assert msg.source_topic == '/imu/data'
    assert msg.state == SensorHealth.UNKNOWN
    assert msg.health_score == -1.0
    assert msg.confidence == 0.0
    assert msg.detected_fault == 'unknown'
    assert msg.reasons == ['detector_not_implemented']
    assert msg.metric_names == []
    assert list(msg.metric_values) == []
    assert msg.window_start == stamp
    assert msg.window_end == stamp
    assert msg.sample_count == 0


def test_no_data_is_unknown():
    decision = make_evaluator().evaluate_imu(10.0)

    assert decision.state == SensorHealth.UNKNOWN
    assert decision.health_score == -1.0
    assert decision.detected_fault == 'unknown'
    assert decision.reasons == ['no_messages_received']
    assert decision.confidence == 0.0


def test_fresh_samples_are_healthy_with_timing_metrics():
    evaluator = make_evaluator()
    add_imu_samples(evaluator, [1.0, 1.1, 1.2])

    decision = evaluator.evaluate_imu(1.3)

    assert decision.state == SensorHealth.HEALTHY
    assert decision.health_score == 1.0
    assert decision.confidence == 1.0
    assert decision.detected_fault == 'none'
    assert decision.metric_names[:3] == [
        'message_age_sec', 'stamp_age_sec', 'interarrival_sec'
    ]
    assert decision.metric_values[:3] == pytest.approx([0.1, 0.1, 0.1])


def test_stale_samples_are_a_fault():
    evaluator = make_evaluator()
    add_imu_samples(evaluator, [1.0, 1.1, 1.2])

    decision = evaluator.evaluate_imu(1.8)

    assert decision.state == SensorHealth.FAULT
    assert decision.health_score == 0.0
    assert decision.detected_fault == 'stale'
    assert decision.reasons == ['latest_message_exceeded_stale_timeout']


def test_normal_header_jitter_remains_healthy():
    evaluator = make_evaluator()
    for received_sec, stamp_age_sec in [
        (1.0, 0.02),
        (1.1, 0.04),
        (1.2, 0.03),
    ]:
        evaluator.add_imu(received_sec, received_sec - stamp_age_sec)

    decision = evaluator.evaluate_imu(1.23)

    assert decision.state == SensorHealth.HEALTHY
    assert decision.detected_fault == 'none'


def test_header_delay_requires_consecutive_confirmation():
    warning_evaluator = make_evaluator()
    add_imu_samples(warning_evaluator, [1.0, 1.1, 1.2], stamp_offset=0.35)
    first_warning = warning_evaluator.evaluate_imu(1.2)
    warning_evaluator.add_imu(1.3, 0.95)
    second_warning = warning_evaluator.evaluate_imu(1.3)
    warning_evaluator.add_imu(1.4, 1.05)
    warning = warning_evaluator.evaluate_imu(1.4)

    fault_evaluator = make_evaluator()
    add_imu_samples(fault_evaluator, [1.0, 1.1, 1.2], stamp_offset=0.6)
    first_fault = fault_evaluator.evaluate_imu(1.2)
    fault_evaluator.add_imu(1.3, 0.7)
    second_fault = fault_evaluator.evaluate_imu(1.3)
    fault_evaluator.add_imu(1.4, 0.8)
    fault = fault_evaluator.evaluate_imu(1.4)

    assert first_warning.state == SensorHealth.DEGRADED
    assert first_warning.reasons == ['header_delay_confirmation_pending']
    assert second_warning.state == SensorHealth.DEGRADED
    assert warning.state == SensorHealth.DEGRADED
    assert warning.health_score == 0.5
    assert warning.detected_fault == 'delay'
    assert first_fault.state == SensorHealth.DEGRADED
    assert first_fault.reasons == ['header_delay_confirmation_pending']
    assert second_fault.state == SensorHealth.DEGRADED
    assert fault.state == SensorHealth.FAULT
    assert fault.health_score == 0.0
    assert fault.detected_fault == 'delay'


def test_static_wheel_without_motion_command_is_not_freeze():
    evaluator = make_evaluator()
    add_static_wheel_samples(evaluator, [0.0, 0.5, 1.2])

    decision = evaluator.evaluate_wheel(1.2)

    assert decision.state == SensorHealth.HEALTHY
    assert decision.detected_fault == 'none'
    assert 'freeze' not in decision.reasons[0]


def test_persistent_motion_command_with_static_wheel_is_freeze_fault():
    evaluator = make_evaluator()
    evaluator.set_command(0.0, linear_x=0.2, angular_z=0.0)
    add_static_wheel_samples(evaluator, [0.2, 0.5, 1.2])

    decision = evaluator.evaluate_wheel(1.2)

    assert decision.state == SensorHealth.FAULT
    assert decision.health_score == 0.0
    assert decision.detected_fault == 'freeze'
    assert decision.reasons == [
        'motion_command_persisted_while_wheel_state_was_static'
    ]
    assert 'wheel_pose_span_m' in decision.metric_names
    assert 'wheel_yaw_span_rad' in decision.metric_names
    assert 'wheel_linear_span_mps' in decision.metric_names
    assert 'wheel_angular_span_rad_s' in decision.metric_names


def test_uniform_rotation_with_changing_yaw_is_healthy():
    evaluator = make_evaluator()
    evaluator.set_command(0.0, linear_x=0.0, angular_z=0.4)
    add_wheel_samples(
        evaluator,
        [
            (0.2, 0.0, 0.0, 0.4),
            (0.5, 0.12, 0.0, 0.4),
            (1.2, 0.40, 0.0, 0.4),
        ],
    )

    decision = evaluator.evaluate_wheel(1.2)

    assert decision.state == SensorHealth.HEALTHY
    assert decision.detected_fault == 'none'
    metrics = dict(zip(decision.metric_names, decision.metric_values))
    assert metrics['wheel_angular_span_rad_s'] == 0.0
    assert metrics['wheel_yaw_span_rad'] == pytest.approx(0.4)


def test_frozen_rotation_with_nonzero_angular_velocity_is_a_freeze_fault():
    evaluator = make_evaluator()
    evaluator.set_command(0.0, linear_x=0.0, angular_z=0.4)
    add_wheel_samples(
        evaluator,
        [
            (0.2, 0.0, 0.0, 0.4),
            (0.5, 0.0, 0.0, 0.4),
            (1.2, 0.0, 0.0, 0.4),
        ],
    )

    decision = evaluator.evaluate_wheel(1.2)

    assert decision.state == SensorHealth.FAULT
    assert decision.detected_fault == 'freeze'
    metrics = dict(zip(decision.metric_names, decision.metric_values))
    assert metrics['wheel_angular_span_rad_s'] == 0.0
    assert metrics['wheel_yaw_span_rad'] == 0.0


def test_rotation_across_pi_boundary_is_not_mistaken_for_a_freeze():
    evaluator = make_evaluator()
    evaluator.set_command(0.0, linear_x=0.0, angular_z=0.4)
    add_wheel_samples(
        evaluator,
        [
            (0.2, 3.00, 0.0, 0.4),
            (0.5, 3.12, 0.0, 0.4),
            (1.2, -3.08, 0.0, 0.4),
        ],
    )

    decision = evaluator.evaluate_wheel(1.2)

    assert decision.state == SensorHealth.HEALTHY
    metrics = dict(zip(decision.metric_names, decision.metric_values))
    assert metrics['wheel_yaw_span_rad'] == pytest.approx(0.2031853072)


def test_continuous_motion_commands_do_not_reset_freeze_timer():
    evaluator = make_evaluator()
    evaluator.set_command(0.0, linear_x=0.2, angular_z=0.0)
    evaluator.set_command(0.3, linear_x=0.2, angular_z=0.0)
    evaluator.set_command(0.8, linear_x=0.2, angular_z=0.0)
    add_static_wheel_samples(evaluator, [0.2, 0.5, 1.2])

    decision = evaluator.evaluate_wheel(1.2)

    assert decision.state == SensorHealth.FAULT
    assert decision.detected_fault == 'freeze'


def test_wheel_freeze_enters_degraded_on_short_horizon_before_fault():
    evaluator = make_evaluator()
    evaluator.set_command(0.0, linear_x=0.2, angular_z=0.0)
    moving = [
        (time_sec, time_sec * 0.2, 0.2, 0.0)
        for time_sec in [3.6, 4.0, 4.4, 4.8]
    ]
    frozen = [
        (time_sec, 0.96, 0.2, 0.0)
        for time_sec in [5.0, 5.1, 5.2, 5.3, 5.4, 5.5]
    ]
    add_wheel_samples(evaluator, moving + frozen)

    degraded = evaluator.evaluate_wheel(5.5)
    for time_sec in [5.6, 5.7, 5.8, 5.9, 6.0, 6.1]:
        add_wheel_samples(evaluator, [(time_sec, 0.96, 0.2, 0.0)])
    fault = evaluator.evaluate_wheel(6.1)

    assert degraded.state == SensorHealth.DEGRADED
    assert degraded.detected_fault == 'freeze'
    assert degraded.reasons == ['short_horizon_wheel_progress_missing']
    assert fault.state == SensorHealth.FAULT
    assert fault.detected_fault == 'freeze'


def test_normal_arc_progress_does_not_trigger_early_freeze_warning():
    evaluator = make_evaluator()
    evaluator.set_command(0.0, linear_x=0.0, angular_z=0.3)
    add_wheel_samples(evaluator, [
        (time_sec, time_sec * 0.3, 0.0, 0.3)
        for time_sec in [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
    ])

    decision = evaluator.evaluate_wheel(0.5)

    assert decision.state == SensorHealth.HEALTHY
    assert decision.detected_fault == 'none'


def test_metric_names_and_values_have_the_same_length():
    evaluator = make_evaluator()
    add_static_wheel_samples(evaluator, [0.0, 0.5, 1.2])
    decision = evaluator.evaluate_wheel(1.2)
    msg = make_sensor_health('wheel', '/faulted/wheel/odometry', Time(), decision)

    assert len(decision.metric_names) == len(decision.metric_values)
    assert len(msg.metric_names) == len(msg.metric_values)


def test_scan_without_data_or_with_insufficient_samples_is_unknown():
    evaluator = make_evaluator()

    no_data = evaluator.evaluate_scan(1.0)
    add_scan_samples(evaluator, [1.1], scan_ranges(100))
    insufficient = evaluator.evaluate_scan(1.1)

    assert no_data.state == SensorHealth.UNKNOWN
    assert no_data.reasons == ['no_messages_received']
    assert insufficient.state == SensorHealth.UNKNOWN
    assert insufficient.reasons == ['startup_provisional_valid']
    assert insufficient.detected_fault == 'none'


def test_normal_finite_scan_is_healthy_with_complete_metrics():
    evaluator = make_evaluator()
    add_scan_samples(evaluator, [1.0, 1.1, 1.2], scan_ranges(100))

    decision = evaluator.evaluate_scan(1.2)

    metrics = dict(zip(decision.metric_names, decision.metric_values))
    assert decision.state == SensorHealth.HEALTHY
    assert decision.detected_fault == 'none'
    assert metrics['scan_total_beams'] == 100.0
    assert metrics['scan_nan_count'] == 0.0
    assert metrics['scan_nan_ratio'] == 0.0
    assert metrics['scan_finite_count'] == 100.0
    assert metrics['scan_inf_count'] == 0.0
    assert metrics['scan_longest_nan_sector_beams'] == 0.0
    assert metrics['scan_longest_nan_sector_width_rad'] == 0.0
    assert len(decision.metric_names) == len(decision.metric_values)


def test_ordinary_inf_ranges_do_not_trigger_sector_blindness():
    evaluator = make_evaluator()
    ranges = scan_ranges(100, inf_indices=range(20, 80))
    add_scan_samples(evaluator, [1.0, 1.1, 1.2], ranges)

    decision = evaluator.evaluate_scan(1.2)

    metrics = dict(zip(decision.metric_names, decision.metric_values))
    assert decision.state == SensorHealth.HEALTHY
    assert decision.detected_fault == 'none'
    assert metrics['scan_nan_count'] == 0.0
    assert metrics['scan_inf_count'] == 60.0


def test_isolated_nan_beams_do_not_trigger_sector_blindness():
    evaluator = make_evaluator()
    ranges = scan_ranges(100, nan_indices=[10, 30, 50, 70])
    add_scan_samples(evaluator, [1.0, 1.1, 1.2], ranges)

    decision = evaluator.evaluate_scan(1.2)

    metrics = dict(zip(decision.metric_names, decision.metric_values))
    assert decision.state == SensorHealth.HEALTHY
    assert decision.detected_fault == 'none'
    assert metrics['scan_nan_ratio'] == pytest.approx(0.04)
    assert metrics['scan_longest_nan_sector_beams'] == 1.0


def test_middle_nan_sector_reports_angular_width_and_fault():
    evaluator = make_evaluator(scan_confirmation_cycles=1)
    ranges = scan_ranges(100, nan_indices=range(40, 60))
    add_scan_samples(
        evaluator, [1.0, 1.1, 1.2], ranges, angle_increment_rad=0.05,
    )

    decision = evaluator.evaluate_scan(1.2)

    metrics = dict(zip(decision.metric_names, decision.metric_values))
    assert decision.state == SensorHealth.FAULT
    assert decision.detected_fault == 'sector_blindness'
    assert decision.reasons == [
        'long_contiguous_nan_sector_exceeded_fault_threshold'
    ]
    assert metrics['scan_nan_ratio'] == pytest.approx(0.20)
    assert metrics['scan_longest_nan_sector_beams'] == 20.0
    assert metrics['scan_longest_nan_sector_width_rad'] == pytest.approx(1.0)


def test_nan_sector_wraps_across_scan_range_ends():
    evaluator = make_evaluator(scan_confirmation_cycles=1)
    ranges = scan_ranges(
        100,
        nan_indices=list(range(0, 10)) + list(range(90, 100)),
    )
    add_scan_samples(
        evaluator, [1.0, 1.1, 1.2], ranges, angle_increment_rad=0.05,
    )

    decision = evaluator.evaluate_scan(1.2)

    metrics = dict(zip(decision.metric_names, decision.metric_values))
    assert decision.state == SensorHealth.FAULT
    assert metrics['scan_longest_nan_sector_beams'] == 20.0
    assert metrics['scan_longest_nan_sector_width_rad'] == pytest.approx(1.0)


def test_scan_sector_requires_confirmation_and_recovers_after_clean_scans():
    evaluator = make_evaluator()
    blind_ranges = scan_ranges(100, nan_indices=range(40, 60))
    add_scan_samples(
        evaluator, [1.0, 1.1, 1.2], blind_ranges, angle_increment_rad=0.05,
    )

    first = evaluator.evaluate_scan(1.2)
    second = evaluator.evaluate_scan(1.2)
    fault = evaluator.evaluate_scan(1.2)
    add_scan_samples(evaluator, [1.3, 1.4, 1.5], scan_ranges(100), 0.05)
    first_recovery = evaluator.evaluate_scan(1.5)
    second_recovery = evaluator.evaluate_scan(1.5)
    recovered = evaluator.evaluate_scan(1.5)

    assert first.state == SensorHealth.HEALTHY
    assert second.state == SensorHealth.HEALTHY
    assert fault.state == SensorHealth.FAULT
    assert first_recovery.state == SensorHealth.FAULT
    assert first_recovery.reasons == ['scan_sector_blindness_recovery_pending']
    assert second_recovery.state == SensorHealth.FAULT
    assert recovered.state == SensorHealth.HEALTHY
    assert recovered.detected_fault == 'none'


def test_scan_stale_and_delay_timing_take_precedence():
    stale_evaluator = make_evaluator()
    add_scan_samples(stale_evaluator, [1.0, 1.1, 1.2], scan_ranges(100))
    stale = stale_evaluator.evaluate_scan(1.8)

    delay_evaluator = make_evaluator()
    add_scan_samples(
        delay_evaluator, [1.0, 1.1, 1.2], scan_ranges(100), stamp_offset=0.6,
    )
    for _ in range(3):
        delay = delay_evaluator.evaluate_scan(1.2)

    assert stale.state == SensorHealth.FAULT
    assert stale.detected_fault == 'stale'
    assert delay.state == SensorHealth.FAULT
    assert delay.detected_fault == 'delay'


def test_stationary_matching_yaw_rates_remain_healthy():
    evaluator = make_evaluator()
    imu_samples, wheel_samples = paired_yaw_rate_samples(0.0, 0.0)
    add_imu_yaw_rate_samples(evaluator, imu_samples)
    add_wheel_yaw_rate_samples(evaluator, wheel_samples)

    wheel_decision = evaluator.evaluate_wheel(1.36)
    decision = evaluator.evaluate_imu(1.36, wheel_decision)

    assert decision.state == SensorHealth.HEALTHY
    assert decision.detected_fault == 'none'
    metrics = dict(zip(decision.metric_names, decision.metric_values))
    assert metrics['imu_wheel_pair_count'] == 10.0
    assert metrics['imu_wheel_residual_mean_rad_s'] == 0.0


def test_matching_constant_turn_yaw_rates_remain_healthy():
    evaluator = make_evaluator()
    imu_samples, wheel_samples = paired_yaw_rate_samples(0.4, 0.4)
    add_imu_yaw_rate_samples(evaluator, imu_samples)
    add_wheel_yaw_rate_samples(evaluator, wheel_samples)

    decision = evaluator.evaluate_imu(1.36, evaluator.evaluate_wheel(1.36))

    assert decision.state == SensorHealth.HEALTHY
    assert decision.detected_fault == 'none'


def test_nonzero_nominal_residual_is_calibrated_as_healthy():
    evaluator = make_evaluator()
    imu_samples, wheel_samples = paired_yaw_rate_samples(0.3628, 0.4)
    add_imu_yaw_rate_samples(evaluator, imu_samples)
    add_wheel_yaw_rate_samples(evaluator, wheel_samples)

    decision = evaluator.evaluate_imu(1.36, evaluator.evaluate_wheel(1.36))

    metrics = dict(zip(decision.metric_names, decision.metric_values))
    assert decision.state == SensorHealth.HEALTHY
    assert decision.detected_fault == 'none'
    assert metrics['imu_wheel_residual_median_rad_s'] == pytest.approx(-0.0372)
    assert metrics['imu_wheel_nominal_residual_baseline_rad_s'] == pytest.approx(
        -0.0372
    )
    assert metrics['imu_wheel_corrected_residual_median_rad_s'] == 0.0
    assert metrics['imu_wheel_baseline_calibrated'] == 1.0


def test_imu_bias_is_unknown_while_nominal_baseline_is_calibrating():
    evaluator = make_evaluator(imu_bias_baseline_calibration_min_pairs=20)
    imu_samples, wheel_samples = paired_yaw_rate_samples(0.3628, 0.4)
    add_imu_yaw_rate_samples(evaluator, imu_samples)
    add_wheel_yaw_rate_samples(evaluator, wheel_samples)

    calibrating = evaluator.evaluate_imu(1.36, evaluator.evaluate_wheel(1.36))
    next_imu, next_wheel = paired_yaw_rate_samples(
        0.3628, 0.4, start_sec=1.4,
    )
    add_imu_yaw_rate_samples(evaluator, next_imu)
    add_wheel_yaw_rate_samples(evaluator, next_wheel)
    calibrated = evaluator.evaluate_imu(1.76, evaluator.evaluate_wheel(1.76))

    metrics = dict(zip(calibrating.metric_names, calibrating.metric_values))
    assert calibrating.state == SensorHealth.UNKNOWN
    assert calibrating.detected_fault == 'unknown'
    assert calibrating.reasons == ['imu_yaw_rate_bias_baseline_calibrating']
    assert metrics['imu_wheel_baseline_calibration_pair_count'] == 10.0
    assert metrics['imu_wheel_baseline_calibrated'] == 0.0
    assert calibrated.state == SensorHealth.HEALTHY


@pytest.mark.parametrize('bias', [0.15, -0.15])
def test_persistent_signed_yaw_rate_bias_becomes_a_fault(bias):
    evaluator = make_evaluator()
    imu_samples, wheel_samples = paired_yaw_rate_samples(0.3628, 0.4)
    add_imu_yaw_rate_samples(evaluator, imu_samples)
    add_wheel_yaw_rate_samples(evaluator, wheel_samples)
    calibration = evaluator.evaluate_imu(1.36, evaluator.evaluate_wheel(1.36))
    imu_samples, wheel_samples = paired_yaw_rate_samples(
        0.3628 + bias, 0.4, start_sec=7.0,
    )
    add_imu_yaw_rate_samples(evaluator, imu_samples)
    add_wheel_yaw_rate_samples(evaluator, wheel_samples)

    first = evaluator.evaluate_imu(7.36, evaluator.evaluate_wheel(7.36))
    second = evaluator.evaluate_imu(7.36, evaluator.evaluate_wheel(7.36))
    decision = evaluator.evaluate_imu(7.36, evaluator.evaluate_wheel(7.36))

    assert calibration.state == SensorHealth.HEALTHY
    assert first.state == SensorHealth.DEGRADED
    assert first.reasons == ['imu_yaw_rate_bias_confirmation_pending']
    assert second.state == SensorHealth.DEGRADED
    assert decision.state == SensorHealth.FAULT
    assert decision.detected_fault == 'bias'
    metrics = dict(zip(decision.metric_names, decision.metric_values))
    assert metrics['imu_wheel_residual_mean_rad_s'] == pytest.approx(-0.0372 + bias)
    assert metrics['imu_wheel_residual_median_rad_s'] == pytest.approx(-0.0372 + bias)
    assert metrics['imu_wheel_corrected_residual_mean_rad_s'] == pytest.approx(bias)
    assert metrics['imu_wheel_corrected_residual_median_rad_s'] == pytest.approx(bias)
    assert metrics['imu_wheel_residual_stddev_rad_s'] == 0.0
    assert metrics['imu_wheel_residual_mad_rad_s'] == 0.0


def test_persistent_warning_level_yaw_rate_bias_becomes_degraded():
    evaluator = make_evaluator()
    imu_samples, wheel_samples = paired_yaw_rate_samples(0.3628, 0.4)
    add_imu_yaw_rate_samples(evaluator, imu_samples)
    add_wheel_yaw_rate_samples(evaluator, wheel_samples)
    evaluator.evaluate_imu(1.36, evaluator.evaluate_wheel(1.36))
    imu_samples, wheel_samples = paired_yaw_rate_samples(
        0.4628, 0.4, start_sec=7.0,
    )
    add_imu_yaw_rate_samples(evaluator, imu_samples)
    add_wheel_yaw_rate_samples(evaluator, wheel_samples)

    for _ in range(3):
        decision = evaluator.evaluate_imu(7.36, evaluator.evaluate_wheel(7.36))

    assert decision.state == SensorHealth.DEGRADED
    assert decision.detected_fault == 'bias'
    assert decision.reasons == [
        'imu_yaw_rate_residual_exceeded_bias_warning_threshold'
    ]


def test_nominal_baseline_does_not_drift_during_a_bias_fault():
    evaluator = make_evaluator()
    imu_samples, wheel_samples = paired_yaw_rate_samples(0.3628, 0.4)
    add_imu_yaw_rate_samples(evaluator, imu_samples)
    add_wheel_yaw_rate_samples(evaluator, wheel_samples)
    evaluator.evaluate_imu(1.36, evaluator.evaluate_wheel(1.36))
    imu_samples, wheel_samples = paired_yaw_rate_samples(
        0.5128, 0.4, start_sec=7.0,
    )
    add_imu_yaw_rate_samples(evaluator, imu_samples)
    add_wheel_yaw_rate_samples(evaluator, wheel_samples)

    for _ in range(3):
        decision = evaluator.evaluate_imu(7.36, evaluator.evaluate_wheel(7.36))

    metrics = dict(zip(decision.metric_names, decision.metric_values))
    assert decision.state == SensorHealth.FAULT
    assert metrics['imu_wheel_nominal_residual_baseline_rad_s'] == pytest.approx(
        -0.0372
    )
    assert metrics['imu_wheel_corrected_residual_median_rad_s'] == pytest.approx(
        0.15
    )


def test_single_yaw_rate_residual_spike_does_not_trigger_bias():
    evaluator = make_evaluator()
    imu_samples, wheel_samples = paired_yaw_rate_samples(0.4, 0.4)
    imu_samples[-1] = (imu_samples[-1][0], imu_samples[-1][1], 0.55)
    add_imu_yaw_rate_samples(evaluator, imu_samples)
    add_wheel_yaw_rate_samples(evaluator, wheel_samples)

    for _ in range(3):
        decision = evaluator.evaluate_imu(1.36, evaluator.evaluate_wheel(1.36))

    assert decision.state == SensorHealth.HEALTHY
    assert decision.detected_fault == 'none'


def test_insufficient_imu_wheel_pairs_reports_reference_status():
    evaluator = make_evaluator()
    imu_samples, wheel_samples = paired_yaw_rate_samples(0.55, 0.4, count=3)
    add_imu_yaw_rate_samples(evaluator, imu_samples)
    add_wheel_yaw_rate_samples(evaluator, wheel_samples)

    decision = evaluator.evaluate_imu(1.08, evaluator.evaluate_wheel(1.08))

    assert decision.state == SensorHealth.UNKNOWN
    assert decision.detected_fault == 'none'
    assert decision.reasons == ['imu_wheel_pairs_insufficient']


def test_unpairable_header_timestamps_report_reference_status():
    evaluator = make_evaluator(imu_wheel_max_pairing_time_diff_sec=0.001)
    imu_samples, wheel_samples = paired_yaw_rate_samples(
        0.55, 0.4, stamp_offset=0.01
    )
    add_imu_yaw_rate_samples(evaluator, imu_samples)
    add_wheel_yaw_rate_samples(evaluator, wheel_samples)

    decision = evaluator.evaluate_imu(1.36, evaluator.evaluate_wheel(1.36))

    assert decision.state == SensorHealth.UNKNOWN
    assert decision.detected_fault == 'none'
    assert decision.reasons == ['imu_wheel_timestamp_pairs_unavailable']


def test_legal_first_sample_is_provisional_but_invalid_value_faults_immediately():
    provisional_evaluator = make_evaluator()
    provisional_evaluator.add_imu(1.0, 1.0, angular_z=0.2)

    provisional = provisional_evaluator.evaluate_imu(1.0)

    invalid_evaluator = make_evaluator()
    invalid_evaluator.add_imu(1.0, 1.0, angular_z=float('nan'))
    invalid = invalid_evaluator.evaluate_imu(1.0)

    assert provisional.state == SensorHealth.UNKNOWN
    assert provisional.detected_fault == 'none'
    assert provisional.reasons == ['startup_provisional_valid']
    assert invalid.state == SensorHealth.FAULT
    assert invalid.detected_fault == 'invalid'
    assert invalid.reasons == ['non_finite_imu_yaw_rate']


def test_future_header_stamp_is_invalid_before_startup_acceptance():
    evaluator = make_evaluator(future_stamp_tolerance_sec=0.05)
    evaluator.add_imu(1.0, 1.2, angular_z=0.2)

    decision = evaluator.evaluate_imu(1.0)

    assert decision.state == SensorHealth.FAULT
    assert decision.detected_fault == 'invalid'
    assert decision.reasons == ['header_stamp_is_in_the_future']


def test_imu_wheel_pairing_does_not_reuse_a_wheel_observation():
    evaluator = make_evaluator()
    add_imu_yaw_rate_samples(
        evaluator,
        [(1.0 + index * 0.04, 1.2, 0.55) for index in range(10)],
    )
    add_wheel_yaw_rate_samples(
        evaluator,
        [(1.0, 1.0, 0.4), (1.1, 1.1, 0.4), (1.2, 1.2, 0.4)],
    )

    decision = evaluator.evaluate_imu(1.36, evaluator.evaluate_wheel(1.36))

    metrics = dict(zip(decision.metric_names, decision.metric_values))
    assert decision.state == SensorHealth.UNKNOWN
    assert decision.detected_fault == 'none'
    assert decision.reasons == ['imu_wheel_pairs_insufficient']
    assert metrics['imu_wheel_pair_count'] == 1.0


def test_wheel_delay_reference_cannot_be_reported_as_imu_bias():
    evaluator = make_evaluator()
    imu_samples, _ = paired_yaw_rate_samples(0.55, 0.4)
    add_imu_yaw_rate_samples(evaluator, imu_samples)
    add_wheel_yaw_rate_samples(
        evaluator,
        [(1.0, 0.4, 0.4), (1.1, 0.5, 0.4), (1.2, 0.6, 0.4)],
    )

    for now_sec in [1.2, 1.3, 1.36]:
        wheel_decision = evaluator.evaluate_wheel(now_sec)
    decision = evaluator.evaluate_imu(1.36, wheel_decision)

    assert wheel_decision.state == SensorHealth.FAULT
    assert wheel_decision.detected_fault == 'delay'
    assert decision.state == SensorHealth.UNKNOWN
    assert decision.detected_fault == 'none'
    assert decision.reasons == ['wheel_reference_unavailable']


def test_abnormal_wheel_reference_does_not_report_imu_bias():
    evaluator = make_evaluator()
    evaluator.set_command(0.0, linear_x=0.2, angular_z=0.0)
    imu_samples, _ = paired_yaw_rate_samples(0.55, 0.4)
    add_imu_yaw_rate_samples(evaluator, imu_samples)
    add_static_wheel_samples(evaluator, [1.0, 1.2, 1.36])

    wheel_decision = evaluator.evaluate_wheel(1.36)
    decision = evaluator.evaluate_imu(1.36, wheel_decision)

    assert wheel_decision.detected_fault == 'freeze'
    assert decision.state == SensorHealth.UNKNOWN
    assert decision.detected_fault == 'none'
    assert decision.reasons == ['wheel_reference_unavailable']
    assert len(decision.metric_names) == len(decision.metric_values)


def test_imu_and_wheel_subscriptions_use_best_effort_qos():
    calls = []

    class FakeMonitor:
        _sources = {
            'imu': '/faulted/imu/data',
            'wheel': '/faulted/wheel/odometry',
            'scan': '/faulted/scan',
        }

        def _on_imu(self, msg):
            pass

        def _on_wheel(self, msg):
            pass

        def _on_scan(self, msg):
            pass

        def create_subscription(self, message_type, topic, callback, qos):
            calls.append((message_type, topic, qos))

    SensorHealthMonitor._create_sensor_subscriptions(FakeMonitor())

    subscriptions = {topic: (message_type, qos) for message_type, topic, qos in calls}
    assert subscriptions['/faulted/imu/data'][0] is Imu
    assert subscriptions['/faulted/wheel/odometry'][0] is Odometry
    assert subscriptions['/faulted/scan'][0] is LaserScan
    for _, qos in subscriptions.values():
        assert qos.reliability == QoSReliabilityPolicy.BEST_EFFORT


def test_monitor_destroy_node_preserves_rclpy_publisher_lifecycle(
    monkeypatch, tmp_path,
):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    rclpy.init()
    node = SensorHealthMonitor()
    try:
        assert set(node._health_publishers) == {'imu', 'wheel', 'scan'}
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_monitor_does_not_use_fault_injection_truth_status():
    source = Path(__file__).parents[1] / (
        'resilient_nav_health_assessment/sensor_health_monitor.py'
    )
    source_text = source.read_text(encoding='utf-8')

    assert 'FaultStatus' not in source_text
    assert '/fault_injection/status' not in source_text
