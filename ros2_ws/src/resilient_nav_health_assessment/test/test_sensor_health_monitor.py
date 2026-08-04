from pathlib import Path

from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
import pytest
from rclpy.qos import QoSReliabilityPolicy
from resilient_nav_health_assessment.sensor_health_monitor import (
    HealthEvaluator,
    make_sensor_health,
    make_unknown_sensor_health,
    SensorHealthMonitor,
)
from resilient_nav_interfaces.msg import SensorHealth
from sensor_msgs.msg import Imu


def make_evaluator():
    return HealthEvaluator(
        window_duration_sec=5.0,
        min_samples=3,
        stale_timeout_sec=0.5,
        delay_warning_sec=0.3,
        delay_fault_sec=0.5,
        delay_confirmation_cycles=3,
        command_linear_threshold_mps=0.05,
        command_angular_threshold_rad_s=0.1,
        freeze_duration_sec=1.0,
        wheel_pose_span_threshold_m=0.01,
        wheel_linear_span_threshold_mps=0.01,
        wheel_angular_span_threshold_rad_s=0.02,
    )


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
    assert decision.metric_names == [
        'message_age_sec', 'stamp_age_sec', 'interarrival_sec'
    ]
    assert decision.metric_values == pytest.approx([0.1, 0.1, 0.1])


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

    assert first_warning.state == SensorHealth.HEALTHY
    assert second_warning.state == SensorHealth.HEALTHY
    assert warning.state == SensorHealth.DEGRADED
    assert warning.health_score == 0.5
    assert warning.detected_fault == 'delay'
    assert first_fault.state == SensorHealth.HEALTHY
    assert second_fault.state == SensorHealth.HEALTHY
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


def test_metric_names_and_values_have_the_same_length():
    evaluator = make_evaluator()
    add_static_wheel_samples(evaluator, [0.0, 0.5, 1.2])
    decision = evaluator.evaluate_wheel(1.2)
    msg = make_sensor_health('wheel', '/faulted/wheel/odometry', Time(), decision)

    assert len(decision.metric_names) == len(decision.metric_values)
    assert len(msg.metric_names) == len(msg.metric_values)


def test_imu_and_wheel_subscriptions_use_best_effort_qos():
    calls = []

    class FakeMonitor:
        _sources = {
            'imu': '/faulted/imu/data',
            'wheel': '/faulted/wheel/odometry',
        }

        def _on_imu(self, msg):
            pass

        def _on_wheel(self, msg):
            pass

        def create_subscription(self, message_type, topic, callback, qos):
            calls.append((message_type, topic, qos))

    SensorHealthMonitor._create_sensor_subscriptions(FakeMonitor())

    subscriptions = {topic: (message_type, qos) for message_type, topic, qos in calls}
    assert subscriptions['/faulted/imu/data'][0] is Imu
    assert subscriptions['/faulted/wheel/odometry'][0] is Odometry
    for _, qos in subscriptions.values():
        assert qos.reliability == QoSReliabilityPolicy.BEST_EFFORT


def test_monitor_does_not_use_fault_injection_truth_status():
    source = Path(__file__).parents[1] / (
        'resilient_nav_health_assessment/sensor_health_monitor.py'
    )
    source_text = source.read_text(encoding='utf-8')

    assert 'FaultStatus' not in source_text
    assert '/fault_injection/status' not in source_text
