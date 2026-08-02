from copy import deepcopy
import inspect

import pytest
from resilient_nav_fault_injection.imu_bias import (
    FaultStatusChangeTracker,
    get_imu_z_gyro_bias_state,
    inject_imu_z_gyro_bias,
    make_imu_z_gyro_bias_status,
    seconds_to_stamp,
    validate_time_window,
)
from resilient_nav_interfaces.msg import FaultStatus
from sensor_msgs.msg import Imu


def make_imu_message(time_sec=2.0):
    msg = Imu()
    whole_seconds = int(time_sec)
    msg.header.stamp.sec = whole_seconds
    msg.header.stamp.nanosec = int((time_sec - whole_seconds) * 1e9)
    msg.header.frame_id = 'imu_link'
    msg.orientation.x = 0.1
    msg.orientation.y = 0.2
    msg.orientation.z = 0.3
    msg.orientation.w = 0.9
    msg.orientation_covariance = [
        0.1, 0.0, 0.0,
        0.0, 0.2, 0.0,
        0.0, 0.0, 0.3,
    ]
    msg.angular_velocity.x = 1.0
    msg.angular_velocity.y = -2.0
    msg.angular_velocity.z = 3.0
    msg.angular_velocity_covariance = [
        1.0, 0.1, 0.2,
        0.3, 2.0, 0.4,
        0.5, 0.6, 3.0,
    ]
    msg.linear_acceleration.x = 4.0
    msg.linear_acceleration.y = -5.0
    msg.linear_acceleration.z = 6.0
    msg.linear_acceleration_covariance = [
        4.0, 0.7, 0.8,
        0.9, 5.0, 1.0,
        1.1, 1.2, 6.0,
    ]
    return msg


def test_input_object_is_unchanged():
    msg = make_imu_message()
    original = deepcopy(msg)

    inject_imu_z_gyro_bias(
        msg,
        enabled=True,
        bias_rad_s=0.25,
        start_time_sec=1.0,
        end_time_sec=3.0,
    )

    assert msg == original


def test_injector_pure_function_has_no_fault_default_values():
    signature = inspect.signature(inject_imu_z_gyro_bias)

    assert signature.parameters['enabled'].default is inspect.Parameter.empty
    assert signature.parameters['bias_rad_s'].default is inspect.Parameter.empty
    assert (
        signature.parameters['start_time_sec'].default
        is inspect.Parameter.empty
    )
    assert (
        signature.parameters['end_time_sec'].default
        is inspect.Parameter.empty
    )


def test_before_start_time_is_not_injected():
    msg = make_imu_message(time_sec=0.999999999)

    faulted = inject_imu_z_gyro_bias(
        msg,
        enabled=True,
        bias_rad_s=0.25,
        start_time_sec=1.0,
        end_time_sec=3.0,
    )

    assert faulted == msg
    assert faulted is not msg


def test_at_start_time_is_injected():
    msg = make_imu_message(time_sec=1.0)

    faulted = inject_imu_z_gyro_bias(
        msg,
        enabled=True,
        bias_rad_s=0.25,
        start_time_sec=1.0,
        end_time_sec=3.0,
    )

    assert faulted.angular_velocity.z == 3.25


def test_inside_time_window_is_injected():
    msg = make_imu_message(time_sec=2.0)

    faulted = inject_imu_z_gyro_bias(
        msg,
        enabled=True,
        bias_rad_s=0.25,
        start_time_sec=1.0,
        end_time_sec=3.0,
    )

    assert faulted.angular_velocity.z == 3.25


def test_at_end_time_is_not_injected():
    msg = make_imu_message(time_sec=3.0)

    faulted = inject_imu_z_gyro_bias(
        msg,
        enabled=True,
        bias_rad_s=0.25,
        start_time_sec=1.0,
        end_time_sec=3.0,
    )

    assert faulted == msg
    assert faulted is not msg


def test_disabled_returns_complete_passthrough_copy():
    msg = make_imu_message()

    faulted = inject_imu_z_gyro_bias(
        msg,
        enabled=False,
        bias_rad_s=0.25,
        start_time_sec=1.0,
        end_time_sec=3.0,
    )

    assert faulted == msg
    assert faulted is not msg


def test_other_fields_and_covariances_are_unchanged():
    msg = make_imu_message()

    faulted = inject_imu_z_gyro_bias(
        msg,
        enabled=True,
        bias_rad_s=0.25,
        start_time_sec=1.0,
        end_time_sec=3.0,
    )

    assert faulted.header == msg.header
    assert faulted.orientation == msg.orientation
    assert faulted.linear_acceleration == msg.linear_acceleration
    assert faulted.angular_velocity.x == msg.angular_velocity.x
    assert faulted.angular_velocity.y == msg.angular_velocity.y
    assert list(faulted.orientation_covariance) == list(
        msg.orientation_covariance
    )
    assert list(faulted.angular_velocity_covariance) == list(
        msg.angular_velocity_covariance
    )
    assert (
        list(faulted.linear_acceleration_covariance)
        == list(msg.linear_acceleration_covariance)
    )


@pytest.mark.parametrize(
    ('start_time_sec', 'end_time_sec'),
    [
        (-0.1, 3.0),
        (1.0, 1.0),
        (2.0, 1.0),
    ],
)
def test_invalid_time_window_is_rejected(start_time_sec, end_time_sec):
    with pytest.raises(ValueError):
        validate_time_window(start_time_sec, end_time_sec)


@pytest.mark.parametrize(
    ('enabled', 'time_sec', 'expected_state'),
    [
        (False, 0.5, FaultStatus.CANCELLED),
        (False, 1.5, FaultStatus.CANCELLED),
        (True, 0.999999999, FaultStatus.SCHEDULED),
        (True, 1.0, FaultStatus.ACTIVE),
        (True, 2.5, FaultStatus.ACTIVE),
        (True, 3.0, FaultStatus.ENDED),
        (True, 4.0, FaultStatus.ENDED),
    ],
)
def test_fault_status_state_rules(enabled, time_sec, expected_state):
    assert (
        get_imu_z_gyro_bias_state(
            enabled=enabled,
            msg_time_sec=time_sec,
            start_time_sec=1.0,
            end_time_sec=3.0,
        )
        == expected_state
    )


@pytest.mark.parametrize(
    ('time_sec', 'expected_sec', 'expected_nanosec'),
    [
        (0.0, 0, 0),
        (1.25, 1, 250000000),
        (2.000000001, 2, 1),
        (9.9999999996, 10, 0),
    ],
)
def test_seconds_to_stamp_conversion(
    time_sec,
    expected_sec,
    expected_nanosec,
):
    stamp = seconds_to_stamp(time_sec)

    assert stamp.sec == expected_sec
    assert stamp.nanosec == expected_nanosec


def test_fault_status_fields_use_input_time_and_node_parameters():
    msg = make_imu_message(time_sec=1.25)

    status = make_imu_z_gyro_bias_status(
        msg,
        enabled=True,
        bias_rad_s=-0.125,
        start_time_sec=1.25,
        end_time_sec=3.5,
        scenario_id='scenario_a',
        scenario_seed=42,
        event_id='event_b',
        source_topic='/custom/imu',
        faulted_topic='/custom/faulted_imu',
    )

    assert status.header.stamp == msg.header.stamp
    assert status.header.frame_id == ''
    assert status.scenario_id == 'scenario_a'
    assert status.scenario_seed == 42
    assert status.event_id == 'event_b'
    assert status.source_topic == '/custom/imu'
    assert status.faulted_topic == '/custom/faulted_imu'
    assert status.sensor == 'imu'
    assert status.model == 'z_gyro_bias'
    assert status.start_time.sec == 1
    assert status.start_time.nanosec == 250000000
    assert status.end_time.sec == 3
    assert status.end_time.nanosec == 500000000
    assert status.state == FaultStatus.ACTIVE
    assert status.severity == pytest.approx(0.125)
    assert list(status.affected_fields) == ['angular_velocity.z']
    assert 'enabled: true' in status.parameters_yaml
    assert 'bias_rad_s: -0.125' in status.parameters_yaml


def test_fault_status_uses_input_imu_header_stamp_for_state():
    msg = make_imu_message(time_sec=3.0)

    status = make_imu_z_gyro_bias_status(
        msg,
        enabled=True,
        bias_rad_s=0.25,
        start_time_sec=1.0,
        end_time_sec=3.0,
        scenario_id='scenario_a',
        scenario_seed=42,
        event_id='event_b',
        source_topic='/custom/imu',
        faulted_topic='/custom/faulted_imu',
    )

    assert status.header.stamp.sec == 3
    assert status.header.stamp.nanosec == 0
    assert status.state == FaultStatus.ENDED


def test_fault_status_change_tracker_filters_repeated_states():
    tracker = FaultStatusChangeTracker()
    scheduled_1 = FaultStatus()
    scheduled_1.state = FaultStatus.SCHEDULED
    scheduled_2 = FaultStatus()
    scheduled_2.state = FaultStatus.SCHEDULED
    active = FaultStatus()
    active.state = FaultStatus.ACTIVE
    ended = FaultStatus()
    ended.state = FaultStatus.ENDED

    assert tracker.first_or_changed(scheduled_1) is scheduled_1
    assert tracker.first_or_changed(scheduled_2) is None
    assert tracker.first_or_changed(active) is active
    assert tracker.first_or_changed(active) is None
    assert tracker.first_or_changed(ended) is ended
