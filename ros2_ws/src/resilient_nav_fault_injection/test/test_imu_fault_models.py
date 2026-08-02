from copy import deepcopy
import random

import pytest
from resilient_nav_fault_injection.imu_fault_models import (
    apply_imu_fault,
    BIAS_MODEL,
    DROPOUT_MODEL,
    FIXED_DELAY_MODEL,
    GAUSSIAN_NOISE_MODEL,
    ImuFixedDelayQueue,
    make_imu_fault_status,
    validate_delay_sec,
    validate_dropout_probability,
    validate_model,
    validate_noise_sigma,
)
from resilient_nav_interfaces.msg import FaultStatus
from sensor_msgs.msg import Imu


def make_imu_message(time_sec=6.0, angular_velocity_z=3.0):
    msg = Imu()
    whole_seconds = int(time_sec)
    msg.header.stamp.sec = whole_seconds
    msg.header.stamp.nanosec = int((time_sec - whole_seconds) * 1e9)
    msg.header.frame_id = 'imu_link'
    msg.orientation.x = 0.1
    msg.orientation.y = 0.2
    msg.orientation.z = 0.3
    msg.orientation.w = 0.9
    msg.angular_velocity.x = 1.0
    msg.angular_velocity.y = -2.0
    msg.angular_velocity.z = angular_velocity_z
    msg.linear_acceleration.x = 4.0
    msg.linear_acceleration.y = -5.0
    msg.linear_acceleration.z = 6.0
    return msg


def inject_noise_sequence(seed, times, *, enabled=True, sigma=0.05):
    rng = random.Random(seed)
    values = []
    for time_sec in times:
        faulted = apply_imu_fault(
            make_imu_message(time_sec=time_sec),
            model=GAUSSIAN_NOISE_MODEL,
            enabled=enabled,
            bias_rad_s=0.15,
            noise_sigma_rad_s=sigma,
            start_time_sec=5.0,
            end_time_sec=15.0,
            noise_rng=rng,
        )
        values.append(faulted.angular_velocity.z)
    return values


def inject_dropout_sequence(
    seed,
    times,
    *,
    enabled=True,
    probability=0.5,
):
    rng = random.Random(seed)
    published = []
    for time_sec in times:
        faulted = apply_imu_fault(
            make_imu_message(time_sec=time_sec),
            model=DROPOUT_MODEL,
            enabled=enabled,
            bias_rad_s=0.15,
            noise_sigma_rad_s=0.05,
            dropout_probability=probability,
            start_time_sec=5.0,
            end_time_sec=15.0,
            noise_rng=rng,
        )
        published.append(faulted is not None)
    return published


def test_bias_model_result_is_unchanged():
    faulted = apply_imu_fault(
        make_imu_message(time_sec=6.0),
        model=BIAS_MODEL,
        enabled=True,
        bias_rad_s=0.25,
        noise_sigma_rad_s=0.05,
        start_time_sec=5.0,
        end_time_sec=15.0,
        noise_rng=random.Random(1),
    )

    assert faulted.angular_velocity.z == 3.25


def test_gaussian_noise_sigma_zero_is_passthrough_copy():
    msg = make_imu_message(time_sec=6.0)

    faulted = apply_imu_fault(
        msg,
        model=GAUSSIAN_NOISE_MODEL,
        enabled=True,
        bias_rad_s=0.15,
        noise_sigma_rad_s=0.0,
        start_time_sec=5.0,
        end_time_sec=15.0,
        noise_rng=random.Random(20260803),
    )

    assert faulted == msg
    assert faulted is not msg


def test_dropout_probability_zero_publishes_all_messages():
    published = inject_dropout_sequence(
        20260803,
        [5.0, 6.0, 7.0, 14.999],
        probability=0.0,
    )

    assert published == [True, True, True, True]


def test_dropout_probability_one_drops_all_active_window_messages():
    results = [
        apply_imu_fault(
            make_imu_message(time_sec=time_sec),
            model=DROPOUT_MODEL,
            enabled=True,
            bias_rad_s=0.15,
            noise_sigma_rad_s=0.05,
            dropout_probability=1.0,
            start_time_sec=5.0,
            end_time_sec=15.0,
            noise_rng=random.Random(20260803),
        )
        for time_sec in [5.0, 6.0, 14.999]
    ]

    assert results == [None, None, None]


def test_same_seed_produces_same_dropout_sequence():
    times = [5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 14.999]

    first = inject_dropout_sequence(20260803, times)
    second = inject_dropout_sequence(20260803, times)

    assert first == second


def test_different_seed_produces_different_dropout_sequence():
    times = [5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 14.999]

    first = inject_dropout_sequence(20260803, times)
    second = inject_dropout_sequence(20260804, times)

    assert first != second


def test_window_outside_messages_do_not_consume_dropout_sequence():
    with_outside_samples = inject_dropout_sequence(
        20260803,
        [4.0, 4.999, 5.0, 15.0, 16.0, 6.0],
        probability=0.5,
    )
    active_only = inject_dropout_sequence(
        20260803,
        [5.0, 6.0],
        probability=0.5,
    )

    assert with_outside_samples[0] is True
    assert with_outside_samples[1] is True
    assert with_outside_samples[2] == active_only[0]
    assert with_outside_samples[3] is True
    assert with_outside_samples[4] is True
    assert with_outside_samples[5] == active_only[1]


def test_disabled_dropout_is_passthrough_and_does_not_consume_sequence():
    disabled_values = inject_dropout_sequence(
        20260803,
        [5.0, 6.0],
        enabled=False,
        probability=0.5,
    )
    active_values = inject_dropout_sequence(
        20260803,
        [5.0, 6.0],
        probability=0.5,
    )

    assert disabled_values == [True, True]
    assert active_values != disabled_values


def test_dropout_passthrough_returns_deep_copy_and_preserves_input_object():
    msg = make_imu_message(time_sec=6.0)
    original = deepcopy(msg)

    faulted = apply_imu_fault(
        msg,
        model=DROPOUT_MODEL,
        enabled=True,
        bias_rad_s=0.15,
        noise_sigma_rad_s=0.05,
        dropout_probability=0.0,
        start_time_sec=5.0,
        end_time_sec=15.0,
        noise_rng=random.Random(20260803),
    )

    assert msg == original
    assert faulted == msg
    assert faulted is not msg


def test_same_seed_produces_same_noise_sequence():
    times = [5.0, 6.0, 7.0, 14.999]

    first = inject_noise_sequence(20260803, times)
    second = inject_noise_sequence(20260803, times)

    assert first == second


def test_different_seed_produces_different_noise_sequence():
    times = [5.0, 6.0, 7.0, 14.999]

    first = inject_noise_sequence(20260803, times)
    second = inject_noise_sequence(20260804, times)

    assert first != second


def test_window_outside_messages_do_not_consume_noise_sequence():
    with_outside_samples = inject_noise_sequence(
        20260803,
        [4.0, 4.999, 5.0, 15.0, 16.0, 6.0],
    )
    active_only = inject_noise_sequence(20260803, [5.0, 6.0])

    assert with_outside_samples[0] == 3.0
    assert with_outside_samples[1] == 3.0
    assert with_outside_samples[2] == active_only[0]
    assert with_outside_samples[3] == 3.0
    assert with_outside_samples[4] == 3.0
    assert with_outside_samples[5] == active_only[1]


def test_disabled_noise_is_passthrough_and_does_not_consume_sequence():
    disabled_values = inject_noise_sequence(
        20260803,
        [5.0, 6.0],
        enabled=False,
    )
    active_values = inject_noise_sequence(20260803, [5.0, 6.0])

    assert disabled_values == [3.0, 3.0]
    assert active_values != disabled_values


def test_gaussian_noise_only_changes_z_and_preserves_input_object():
    msg = make_imu_message(time_sec=6.0)
    original = deepcopy(msg)

    faulted = apply_imu_fault(
        msg,
        model=GAUSSIAN_NOISE_MODEL,
        enabled=True,
        bias_rad_s=0.15,
        noise_sigma_rad_s=0.05,
        start_time_sec=5.0,
        end_time_sec=15.0,
        noise_rng=random.Random(20260803),
    )

    assert msg == original
    assert faulted.header == msg.header
    assert faulted.orientation == msg.orientation
    assert faulted.angular_velocity.x == msg.angular_velocity.x
    assert faulted.angular_velocity.y == msg.angular_velocity.y
    assert faulted.linear_acceleration == msg.linear_acceleration
    assert faulted.angular_velocity.z != msg.angular_velocity.z


def test_invalid_sigma_is_rejected():
    with pytest.raises(ValueError):
        validate_noise_sigma(-0.001)

    with pytest.raises(ValueError):
        apply_imu_fault(
            make_imu_message(),
            model=GAUSSIAN_NOISE_MODEL,
            enabled=True,
            bias_rad_s=0.0,
            noise_sigma_rad_s=-0.001,
            start_time_sec=5.0,
            end_time_sec=15.0,
            noise_rng=random.Random(1),
        )


def test_invalid_dropout_probability_is_rejected():
    for invalid_probability in (-0.001, 1.001):
        with pytest.raises(ValueError):
            validate_dropout_probability(invalid_probability)

        with pytest.raises(ValueError):
            apply_imu_fault(
                make_imu_message(),
                model=DROPOUT_MODEL,
                enabled=True,
                bias_rad_s=0.0,
                noise_sigma_rad_s=0.05,
                dropout_probability=invalid_probability,
                start_time_sec=5.0,
                end_time_sec=15.0,
                noise_rng=random.Random(1),
            )


def test_invalid_model_is_rejected():
    with pytest.raises(ValueError):
        validate_model('delay')


def test_fixed_delay_active_message_is_queued_without_immediate_output():
    queue = ImuFixedDelayQueue(delay_sec=0.50)
    immediate = queue.handle_message(
        make_imu_message(time_sec=6.0),
        enabled=True,
        current_ros_time_sec=100.0,
        start_time_sec=5.0,
        end_time_sec=15.0,
    )

    assert immediate is None
    assert len(queue) == 1


def test_fixed_delay_outputs_at_release_time():
    queue = ImuFixedDelayQueue(delay_sec=0.50)
    msg = make_imu_message(time_sec=6.0)
    queue.handle_message(
        msg,
        enabled=True,
        current_ros_time_sec=100.0,
        start_time_sec=5.0,
        end_time_sec=15.0,
    )

    released = queue.pop_ready(100.50)

    assert released == [msg]
    assert len(queue) == 0


def test_fixed_delay_does_not_output_before_release_time():
    queue = ImuFixedDelayQueue(delay_sec=0.50)
    queue.handle_message(
        make_imu_message(time_sec=6.0),
        enabled=True,
        current_ros_time_sec=100.0,
        start_time_sec=5.0,
        end_time_sec=15.0,
    )

    assert queue.pop_ready(100.49) == []
    assert len(queue) == 1


def test_fixed_delay_preserves_input_order_for_multiple_messages():
    queue = ImuFixedDelayQueue(delay_sec=0.50)
    first = make_imu_message(time_sec=6.0, angular_velocity_z=1.0)
    second = make_imu_message(time_sec=6.1, angular_velocity_z=2.0)

    queue.handle_message(
        first,
        enabled=True,
        current_ros_time_sec=100.0,
        start_time_sec=5.0,
        end_time_sec=15.0,
    )
    queue.handle_message(
        second,
        enabled=True,
        current_ros_time_sec=100.1,
        start_time_sec=5.0,
        end_time_sec=15.0,
    )

    released = queue.pop_ready(100.6)

    assert released == [first, second]


def test_fixed_delay_preserves_header_stamp_and_message_content():
    queue = ImuFixedDelayQueue(delay_sec=0.50)
    msg = make_imu_message(time_sec=6.0, angular_velocity_z=3.25)
    original = deepcopy(msg)

    queue.handle_message(
        msg,
        enabled=True,
        current_ros_time_sec=100.0,
        start_time_sec=5.0,
        end_time_sec=15.0,
    )
    msg.angular_velocity.z = 9.0
    released = queue.pop_ready(100.5)

    assert released == [original]
    assert released[0].header.stamp == original.header.stamp
    assert released[0] is not msg


def test_fixed_delay_window_outside_message_is_passthrough_and_not_queued():
    queue = ImuFixedDelayQueue(delay_sec=0.50)
    msg = make_imu_message(time_sec=15.0)

    immediate = queue.handle_message(
        msg,
        enabled=True,
        current_ros_time_sec=100.0,
        start_time_sec=5.0,
        end_time_sec=15.0,
    )

    assert immediate == msg
    assert immediate is not msg
    assert len(queue) == 0


def test_fixed_delay_disabled_message_is_passthrough_and_not_queued():
    queue = ImuFixedDelayQueue(delay_sec=0.50)
    msg = make_imu_message(time_sec=6.0)

    immediate = queue.handle_message(
        msg,
        enabled=False,
        current_ros_time_sec=100.0,
        start_time_sec=5.0,
        end_time_sec=15.0,
    )

    assert immediate == msg
    assert immediate is not msg
    assert len(queue) == 0


def test_fixed_delay_zero_can_release_immediately():
    queue = ImuFixedDelayQueue(delay_sec=0.0)
    msg = make_imu_message(time_sec=6.0)

    immediate = queue.handle_message(
        msg,
        enabled=True,
        current_ros_time_sec=100.0,
        start_time_sec=5.0,
        end_time_sec=15.0,
    )

    assert immediate is None
    assert queue.pop_ready(100.0) == [msg]


def test_negative_fixed_delay_is_rejected():
    with pytest.raises(ValueError):
        validate_delay_sec(-0.001)

    with pytest.raises(ValueError):
        ImuFixedDelayQueue(delay_sec=-0.001)


def test_fixed_delay_drains_queued_messages_after_window_has_ended():
    queue = ImuFixedDelayQueue(delay_sec=0.50)
    msg = make_imu_message(time_sec=14.9)
    queue.handle_message(
        msg,
        enabled=True,
        current_ros_time_sec=100.0,
        start_time_sec=5.0,
        end_time_sec=15.0,
    )

    ended_status = make_imu_fault_status(
        make_imu_message(time_sec=16.0),
        model=FIXED_DELAY_MODEL,
        enabled=True,
        bias_rad_s=0.15,
        noise_sigma_rad_s=0.05,
        delay_sec=0.50,
        start_time_sec=5.0,
        end_time_sec=15.0,
        scenario_id='imu_delay_demo',
        scenario_seed=20260803,
        event_id='imu_delay_001',
        source_topic='/imu/data',
        faulted_topic='/faulted/imu/data',
    )

    assert ended_status.state == FaultStatus.ENDED
    assert queue.pop_ready(100.50) == [msg]


def test_fixed_delay_active_fault_status_does_not_depend_on_release():
    queue = ImuFixedDelayQueue(delay_sec=0.50)
    msg = make_imu_message(time_sec=6.5)
    queue.handle_message(
        msg,
        enabled=True,
        current_ros_time_sec=100.0,
        start_time_sec=5.0,
        end_time_sec=15.0,
    )

    status = make_imu_fault_status(
        msg,
        model=FIXED_DELAY_MODEL,
        enabled=True,
        bias_rad_s=0.15,
        noise_sigma_rad_s=0.05,
        delay_sec=0.50,
        start_time_sec=5.0,
        end_time_sec=15.0,
        scenario_id='imu_delay_demo',
        scenario_seed=20260803,
        event_id='imu_delay_001',
        source_topic='/imu/data',
        faulted_topic='/faulted/imu/data',
    )

    assert len(queue) == 1
    assert queue.pop_ready(100.49) == []
    assert status.state == FaultStatus.ACTIVE


def test_fixed_delay_fault_status_fields_are_correct():
    msg = make_imu_message(time_sec=6.5)

    status = make_imu_fault_status(
        msg,
        model=FIXED_DELAY_MODEL,
        enabled=True,
        bias_rad_s=0.15,
        noise_sigma_rad_s=0.05,
        delay_sec=0.75,
        start_time_sec=5.0,
        end_time_sec=15.0,
        scenario_id='imu_delay_demo',
        scenario_seed=20260803,
        event_id='imu_delay_001',
        source_topic='/imu/data',
        faulted_topic='/faulted/imu/data',
    )

    assert status.header.stamp == msg.header.stamp
    assert status.scenario_id == 'imu_delay_demo'
    assert status.scenario_seed == 20260803
    assert status.event_id == 'imu_delay_001'
    assert status.source_topic == '/imu/data'
    assert status.faulted_topic == '/faulted/imu/data'
    assert status.sensor == 'imu'
    assert status.model == 'fixed_delay'
    assert status.state == FaultStatus.ACTIVE
    assert status.severity == pytest.approx(0.75)
    assert list(status.affected_fields) == ['message_delivery_time']
    assert 'model: fixed_delay' in status.parameters_yaml
    assert 'delay_sec: 0.75' in status.parameters_yaml


def test_noise_fault_status_fields_are_correct():
    msg = make_imu_message(time_sec=6.5)

    status = make_imu_fault_status(
        msg,
        model=GAUSSIAN_NOISE_MODEL,
        enabled=True,
        bias_rad_s=0.15,
        noise_sigma_rad_s=0.075,
        start_time_sec=5.0,
        end_time_sec=15.0,
        scenario_id='imu_noise_demo',
        scenario_seed=20260803,
        event_id='imu_noise_001',
        source_topic='/imu/data',
        faulted_topic='/faulted/imu/data',
    )

    assert status.header.stamp == msg.header.stamp
    assert status.scenario_id == 'imu_noise_demo'
    assert status.scenario_seed == 20260803
    assert status.event_id == 'imu_noise_001'
    assert status.source_topic == '/imu/data'
    assert status.faulted_topic == '/faulted/imu/data'
    assert status.sensor == 'imu'
    assert status.model == 'gaussian_noise'
    assert status.state == FaultStatus.ACTIVE
    assert status.severity == pytest.approx(0.075)
    assert list(status.affected_fields) == ['angular_velocity.z']
    assert 'model: gaussian_noise' in status.parameters_yaml
    assert 'sigma: 0.075' in status.parameters_yaml
    assert 'seed: 20260803' in status.parameters_yaml


def test_dropout_fault_status_fields_are_correct_when_message_is_dropped():
    msg = make_imu_message(time_sec=6.5)
    dropped = apply_imu_fault(
        msg,
        model=DROPOUT_MODEL,
        enabled=True,
        bias_rad_s=0.15,
        noise_sigma_rad_s=0.05,
        dropout_probability=1.0,
        start_time_sec=5.0,
        end_time_sec=15.0,
        noise_rng=random.Random(20260803),
    )

    status = make_imu_fault_status(
        msg,
        model=DROPOUT_MODEL,
        enabled=True,
        bias_rad_s=0.15,
        noise_sigma_rad_s=0.05,
        dropout_probability=1.0,
        start_time_sec=5.0,
        end_time_sec=15.0,
        scenario_id='imu_dropout_demo',
        scenario_seed=20260803,
        event_id='imu_dropout_001',
        source_topic='/imu/data',
        faulted_topic='/faulted/imu/data',
    )

    assert dropped is None
    assert status.header.stamp == msg.header.stamp
    assert status.scenario_id == 'imu_dropout_demo'
    assert status.scenario_seed == 20260803
    assert status.event_id == 'imu_dropout_001'
    assert status.source_topic == '/imu/data'
    assert status.faulted_topic == '/faulted/imu/data'
    assert status.sensor == 'imu'
    assert status.model == 'dropout'
    assert status.state == FaultStatus.ACTIVE
    assert status.severity == pytest.approx(1.0)
    assert list(status.affected_fields) == ['message_delivery']
    assert 'model: dropout' in status.parameters_yaml
    assert 'dropout_probability: 1.0' in status.parameters_yaml
    assert 'seed: 20260803' in status.parameters_yaml
