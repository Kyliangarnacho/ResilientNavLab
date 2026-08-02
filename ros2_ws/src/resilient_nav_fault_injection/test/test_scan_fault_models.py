from copy import deepcopy
import math

import pytest
from resilient_nav_fault_injection.imu_fault_models import (
    FaultStatusChangeTracker,
)
from resilient_nav_fault_injection.scan_fault_models import (
    apply_scan_fault,
    is_angle_in_sector,
    make_scan_fault_status,
    SECTOR_BLINDNESS_MODEL,
    validate_model,
    validate_sector_width,
)
from resilient_nav_interfaces.msg import FaultStatus
from sensor_msgs.msg import LaserScan


def make_scan_message(
    time_sec=6.0,
    *,
    angle_min=-1.0,
    angle_increment=0.5,
    ranges=None,
    intensities=None,
):
    msg = LaserScan()
    whole_seconds = int(time_sec)
    msg.header.stamp.sec = whole_seconds
    msg.header.stamp.nanosec = int((time_sec - whole_seconds) * 1e9)
    msg.header.frame_id = 'lidar_link'
    msg.angle_min = angle_min
    msg.angle_max = angle_min + 4.0 * angle_increment
    msg.angle_increment = angle_increment
    msg.time_increment = 0.01
    msg.scan_time = 0.1
    msg.range_min = 0.05
    msg.range_max = 8.0
    msg.ranges = ranges if ranges is not None else [1.0, 2.0, 3.0, 4.0, 5.0]
    msg.intensities = intensities if intensities is not None else [
        10.0,
        20.0,
        30.0,
        40.0,
        50.0,
    ]
    return msg


def apply_sector_blindness(
    msg,
    *,
    enabled=True,
    center=0.0,
    width=1.0,
):
    return apply_scan_fault(
        msg,
        model=SECTOR_BLINDNESS_MODEL,
        enabled=enabled,
        sector_center_rad=center,
        sector_width_rad=width,
        start_time_sec=5.0,
        end_time_sec=15.0,
    )


def assert_nan(value):
    assert math.isnan(value)


def test_before_window_is_passthrough_copy():
    msg = make_scan_message(time_sec=4.999)

    faulted = apply_sector_blindness(msg)

    assert faulted == msg
    assert faulted is not msg


def test_active_only_changes_ranges_inside_sector():
    msg = make_scan_message(time_sec=6.0)

    faulted = apply_sector_blindness(msg, center=0.0, width=1.0)

    assert faulted.ranges[0] == 1.0
    assert_nan(faulted.ranges[1])
    assert_nan(faulted.ranges[2])
    assert_nan(faulted.ranges[3])
    assert faulted.ranges[4] == 5.0


def test_sector_boundaries_are_inclusive():
    assert is_angle_in_sector(-0.5, 0.0, 1.0)
    assert is_angle_in_sector(0.5, 0.0, 1.0)
    assert not is_angle_in_sector(-0.500001, 0.0, 1.0)
    assert not is_angle_in_sector(0.500001, 0.0, 1.0)


def test_sector_wrapping_across_pi_boundary():
    msg = make_scan_message(
        time_sec=6.0,
        angle_min=math.pi - 0.2,
        angle_increment=0.2,
        ranges=[1.0, 2.0, 3.0, 4.0],
        intensities=[10.0, 20.0, 30.0, 40.0],
    )

    faulted = apply_sector_blindness(
        msg,
        center=math.pi,
        width=0.5,
    )

    assert_nan(faulted.ranges[0])
    assert_nan(faulted.ranges[1])
    assert_nan(faulted.ranges[2])
    assert faulted.ranges[3] == 4.0


def test_non_target_beams_and_scan_metadata_are_preserved():
    msg = make_scan_message(time_sec=6.0)

    faulted = apply_sector_blindness(msg, center=0.0, width=0.25)

    assert list(faulted.ranges[:2]) == [1.0, 2.0]
    assert list(faulted.ranges[3:]) == [4.0, 5.0]
    assert faulted.header == msg.header
    assert faulted.angle_min == msg.angle_min
    assert faulted.angle_max == msg.angle_max
    assert faulted.angle_increment == msg.angle_increment
    assert faulted.range_min == msg.range_min
    assert faulted.range_max == msg.range_max


def test_matching_intensities_are_zeroed_inside_sector():
    msg = make_scan_message(time_sec=6.0)

    faulted = apply_sector_blindness(msg, center=0.0, width=1.0)

    assert faulted.intensities[0] == 10.0
    assert list(faulted.intensities[1:4]) == [0.0, 0.0, 0.0]
    assert faulted.intensities[4] == 50.0


def test_empty_intensities_are_valid():
    msg = make_scan_message(time_sec=6.0, intensities=[])

    faulted = apply_sector_blindness(msg, center=0.0, width=1.0)

    assert list(faulted.intensities) == []
    assert_nan(faulted.ranges[2])


def test_mismatched_intensities_are_preserved():
    msg = make_scan_message(time_sec=6.0, intensities=[10.0, 20.0])

    faulted = apply_sector_blindness(msg, center=0.0, width=1.0)

    assert list(faulted.intensities) == [10.0, 20.0]


def test_disabled_is_passthrough_copy():
    msg = make_scan_message(time_sec=6.0)

    faulted = apply_sector_blindness(msg, enabled=False)

    assert faulted == msg
    assert faulted is not msg


def test_input_object_is_not_modified():
    msg = make_scan_message(time_sec=6.0)
    original = deepcopy(msg)

    apply_sector_blindness(msg)

    assert msg == original


@pytest.mark.parametrize('invalid_width', [-0.1, 0.0, 2.0 * math.pi + 0.001])
def test_invalid_width_is_rejected(invalid_width):
    with pytest.raises(ValueError):
        validate_sector_width(invalid_width)

    with pytest.raises(ValueError):
        apply_sector_blindness(make_scan_message(), width=invalid_width)


def test_invalid_model_is_rejected():
    with pytest.raises(ValueError):
        validate_model('freeze')

    with pytest.raises(ValueError):
        apply_scan_fault(
            make_scan_message(),
            model='freeze',
            enabled=True,
            sector_center_rad=0.0,
            sector_width_rad=1.0,
            start_time_sec=5.0,
            end_time_sec=15.0,
        )


def test_full_circle_width_blinds_all_ranges():
    msg = make_scan_message(time_sec=6.0)

    faulted = apply_sector_blindness(msg, width=2.0 * math.pi)

    assert all(math.isnan(value) for value in faulted.ranges)


def test_fault_status_fields_are_correct():
    msg = make_scan_message(time_sec=6.5)

    status = make_scan_fault_status(
        msg,
        model=SECTOR_BLINDNESS_MODEL,
        enabled=True,
        sector_center_rad=0.25,
        sector_width_rad=1.25,
        start_time_sec=5.0,
        end_time_sec=15.0,
        scenario_id='scan_sector_blindness_demo',
        scenario_seed=20260803,
        event_id='scan_sector_blindness_001',
        source_topic='/scan',
        faulted_topic='/faulted/scan',
    )

    assert status.header.stamp == msg.header.stamp
    assert status.header.frame_id == ''
    assert status.scenario_id == 'scan_sector_blindness_demo'
    assert status.scenario_seed == 20260803
    assert status.event_id == 'scan_sector_blindness_001'
    assert status.source_topic == '/scan'
    assert status.faulted_topic == '/faulted/scan'
    assert status.sensor == 'lidar'
    assert status.model == 'sector_blindness'
    assert status.state == FaultStatus.ACTIVE
    assert status.severity == pytest.approx(1.25)
    assert list(status.affected_fields) == ['ranges']
    assert 'center: 0.25' in status.parameters_yaml
    assert 'width: 1.25' in status.parameters_yaml
    assert 'invalid_value: nan' in status.parameters_yaml


def test_fault_status_state_changes_are_filtered():
    tracker = FaultStatusChangeTracker()
    messages = [
        make_scan_message(time_sec=4.0),
        make_scan_message(time_sec=4.5),
        make_scan_message(time_sec=5.0),
        make_scan_message(time_sec=6.0),
        make_scan_message(time_sec=15.0),
    ]
    emitted_states = []

    for msg in messages:
        status = make_scan_fault_status(
            msg,
            model=SECTOR_BLINDNESS_MODEL,
            enabled=True,
            sector_center_rad=0.0,
            sector_width_rad=1.0,
            start_time_sec=5.0,
            end_time_sec=15.0,
            scenario_id='scan_sector_blindness_demo',
            scenario_seed=20260803,
            event_id='scan_sector_blindness_001',
            source_topic='/scan',
            faulted_topic='/faulted/scan',
        )
        changed = tracker.first_or_changed(status)
        if changed is not None:
            emitted_states.append(changed.state)

    assert emitted_states == [
        FaultStatus.SCHEDULED,
        FaultStatus.ACTIVE,
        FaultStatus.ENDED,
    ]
