from copy import deepcopy

from nav_msgs.msg import Odometry
import pytest
from resilient_nav_fault_injection.imu_fault_models import (
    FaultStatusChangeTracker,
    validate_time_window,
)
from resilient_nav_fault_injection.wheel_fault_models import (
    FREEZE_MODEL,
    make_wheel_fault_status,
    validate_model,
    WheelOdometryFreezeModel,
)
from resilient_nav_interfaces.msg import FaultStatus


def make_odometry_message(
    time_sec=6.0,
    *,
    frame_id='odom',
    child_frame_id='base_footprint',
    pose_x=1.0,
    twist_x=0.2,
    covariance_offset=0.0,
):
    msg = Odometry()
    whole_seconds = int(time_sec)
    msg.header.stamp.sec = whole_seconds
    msg.header.stamp.nanosec = int((time_sec - whole_seconds) * 1e9)
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id
    msg.pose.pose.position.x = pose_x
    msg.pose.pose.position.y = pose_x + 0.1
    msg.pose.pose.orientation.z = pose_x + 0.2
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.linear.x = twist_x
    msg.twist.twist.angular.z = twist_x + 0.3
    msg.pose.covariance = [
        covariance_offset + float(index)
        for index in range(36)
    ]
    msg.twist.covariance = [
        covariance_offset + 100.0 + float(index)
        for index in range(36)
    ]
    return msg


def apply_freeze(model, msg, *, enabled=True):
    return model.apply(
        msg,
        model=FREEZE_MODEL,
        enabled=enabled,
        start_time_sec=5.0,
        end_time_sec=15.0,
    )


def test_before_window_is_passthrough_copy():
    model = WheelOdometryFreezeModel()
    msg = make_odometry_message(time_sec=4.999, pose_x=1.0)

    faulted = apply_freeze(model, msg)

    assert faulted == msg
    assert faulted is not msg


def test_first_active_message_establishes_snapshot():
    model = WheelOdometryFreezeModel()
    first = make_odometry_message(time_sec=5.0, pose_x=1.0, twist_x=0.2)
    second = make_odometry_message(time_sec=6.0, pose_x=9.0, twist_x=8.0)

    first_faulted = apply_freeze(model, first)
    second_faulted = apply_freeze(model, second)

    assert first_faulted.pose == first.pose
    assert first_faulted.twist == first.twist
    assert second_faulted.pose == first.pose
    assert second_faulted.twist == first.twist


def test_active_window_pose_twist_and_covariance_stay_frozen():
    model = WheelOdometryFreezeModel()
    snapshot = make_odometry_message(
        time_sec=5.0,
        pose_x=2.0,
        twist_x=0.4,
        covariance_offset=10.0,
    )
    current = make_odometry_message(
        time_sec=7.0,
        pose_x=7.0,
        twist_x=1.4,
        covariance_offset=70.0,
    )

    apply_freeze(model, snapshot)
    faulted = apply_freeze(model, current)

    assert faulted.pose.pose == snapshot.pose.pose
    assert list(faulted.pose.covariance) == list(snapshot.pose.covariance)
    assert faulted.twist.twist == snapshot.twist.twist
    assert list(faulted.twist.covariance) == list(snapshot.twist.covariance)


def test_active_output_header_stamp_tracks_current_input():
    model = WheelOdometryFreezeModel()
    snapshot = make_odometry_message(time_sec=5.0)
    current = make_odometry_message(time_sec=6.25, pose_x=9.0)

    apply_freeze(model, snapshot)
    faulted = apply_freeze(model, current)

    assert faulted.header.stamp == current.header.stamp
    assert faulted.header.stamp != snapshot.header.stamp


def test_frame_ids_track_current_input_semantics():
    model = WheelOdometryFreezeModel()
    snapshot = make_odometry_message(
        time_sec=5.0,
        frame_id='odom',
        child_frame_id='base_footprint',
    )
    current = make_odometry_message(
        time_sec=6.0,
        frame_id='map_like_odom',
        child_frame_id='base_link_like',
        pose_x=9.0,
    )

    apply_freeze(model, snapshot)
    faulted = apply_freeze(model, current)

    assert faulted.header.frame_id == 'map_like_odom'
    assert faulted.child_frame_id == 'base_link_like'


def test_after_window_returns_live_data_and_resets_snapshot():
    model = WheelOdometryFreezeModel()
    snapshot = make_odometry_message(time_sec=5.0, pose_x=1.0)
    ended = make_odometry_message(time_sec=15.0, pose_x=15.0)
    reentered = make_odometry_message(time_sec=5.5, pose_x=5.5)
    later_active = make_odometry_message(time_sec=6.0, pose_x=6.0)

    apply_freeze(model, snapshot)
    ended_faulted = apply_freeze(model, ended)
    reentered_faulted = apply_freeze(model, reentered)
    later_faulted = apply_freeze(model, later_active)

    assert ended_faulted == ended
    assert ended_faulted.pose == ended.pose
    assert reentered_faulted.pose == reentered.pose
    assert later_faulted.pose == reentered.pose


def test_disabled_is_passthrough_and_resets_snapshot():
    model = WheelOdometryFreezeModel()
    snapshot = make_odometry_message(time_sec=5.0, pose_x=1.0)
    disabled = make_odometry_message(time_sec=6.0, pose_x=6.0)
    reenabled = make_odometry_message(time_sec=7.0, pose_x=7.0)
    later_active = make_odometry_message(time_sec=8.0, pose_x=8.0)

    apply_freeze(model, snapshot)
    disabled_faulted = apply_freeze(model, disabled, enabled=False)
    reenabled_faulted = apply_freeze(model, reenabled)
    later_faulted = apply_freeze(model, later_active)

    assert disabled_faulted == disabled
    assert reenabled_faulted.pose == reenabled.pose
    assert later_faulted.pose == reenabled.pose


def test_input_object_is_not_modified():
    model = WheelOdometryFreezeModel()
    msg = make_odometry_message(time_sec=5.0)
    original = deepcopy(msg)

    apply_freeze(model, msg)

    assert msg == original


@pytest.mark.parametrize(
    ('start_time_sec', 'end_time_sec'),
    [
        (-0.1, 3.0),
        (1.0, 1.0),
        (2.0, 1.0),
    ],
)
def test_invalid_time_window_is_rejected(start_time_sec, end_time_sec):
    model = WheelOdometryFreezeModel()

    with pytest.raises(ValueError):
        validate_time_window(start_time_sec, end_time_sec)

    with pytest.raises(ValueError):
        model.apply(
            make_odometry_message(),
            model=FREEZE_MODEL,
            enabled=True,
            start_time_sec=start_time_sec,
            end_time_sec=end_time_sec,
        )


def test_invalid_model_is_rejected():
    model = WheelOdometryFreezeModel()

    with pytest.raises(ValueError):
        validate_model('bias')

    with pytest.raises(ValueError):
        model.apply(
            make_odometry_message(),
            model='bias',
            enabled=True,
            start_time_sec=5.0,
            end_time_sec=15.0,
        )


def test_fault_status_fields_are_correct():
    msg = make_odometry_message(time_sec=6.5)

    status = make_wheel_fault_status(
        msg,
        model=FREEZE_MODEL,
        enabled=True,
        start_time_sec=5.0,
        end_time_sec=15.0,
        scenario_id='wheel_freeze_demo',
        scenario_seed=20260803,
        event_id='wheel_freeze_001',
        source_topic='/wheel/odometry',
        faulted_topic='/faulted/wheel/odometry',
    )

    assert status.header.stamp == msg.header.stamp
    assert status.header.frame_id == ''
    assert status.scenario_id == 'wheel_freeze_demo'
    assert status.scenario_seed == 20260803
    assert status.event_id == 'wheel_freeze_001'
    assert status.source_topic == '/wheel/odometry'
    assert status.faulted_topic == '/faulted/wheel/odometry'
    assert status.sensor == 'wheel_odometry'
    assert status.model == 'freeze'
    assert status.state == FaultStatus.ACTIVE
    assert status.severity == pytest.approx(1.0)
    assert 'pose' in list(status.affected_fields)
    assert 'twist' in list(status.affected_fields)
    assert 'model: freeze' in status.parameters_yaml
    assert 'enabled: true' in status.parameters_yaml


def test_fault_status_state_changes_are_filtered():
    tracker = FaultStatusChangeTracker()
    messages = [
        make_odometry_message(time_sec=4.0),
        make_odometry_message(time_sec=4.5),
        make_odometry_message(time_sec=5.0),
        make_odometry_message(time_sec=6.0),
        make_odometry_message(time_sec=15.0),
    ]
    emitted_states = []

    for msg in messages:
        status = make_wheel_fault_status(
            msg,
            model=FREEZE_MODEL,
            enabled=True,
            start_time_sec=5.0,
            end_time_sec=15.0,
            scenario_id='wheel_freeze_demo',
            scenario_seed=20260803,
            event_id='wheel_freeze_001',
            source_topic='/wheel/odometry',
            faulted_topic='/faulted/wheel/odometry',
        )
        changed = tracker.first_or_changed(status)
        if changed is not None:
            emitted_states.append(changed.state)

    assert emitted_states == [
        FaultStatus.SCHEDULED,
        FaultStatus.ACTIVE,
        FaultStatus.ENDED,
    ]
