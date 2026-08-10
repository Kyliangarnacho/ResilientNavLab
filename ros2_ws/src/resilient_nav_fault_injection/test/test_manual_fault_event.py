"""Unit tests for the manual FaultStatus truth event publisher."""

import pytest
from resilient_nav_fault_injection.manual_fault_event import (
    make_manual_fault_status,
    ManualFaultConfiguration,
    ManualFaultTimeline,
)
from resilient_nav_interfaces.msg import FaultStatus


def configuration(**overrides):
    """Return a valid configurable camera truth event."""
    values = {
        'sensor': ' camera ',
        'model': ' freeze ',
        'event_id': 'camera_freeze_001',
        'severity': 0.8,
        'start_delay_sec': 2.0,
        'duration_sec': 5.0,
    }
    values.update(overrides)
    return ManualFaultConfiguration(**values)


def test_configuration_normalizes_labels_and_validates_timing():
    validated = configuration().validated()

    assert validated.sensor == 'camera'
    assert validated.model == 'freeze'
    assert validated.event_id == 'camera_freeze_001'
    assert validated.severity == 0.8


@pytest.mark.parametrize(
    ('overrides', 'message'),
    [
        ({'sensor': ''}, 'sensor'),
        ({'model': ''}, 'model'),
        ({'event_id': ''}, 'event_id'),
        ({'severity': 1.1}, 'severity'),
        ({'start_delay_sec': -0.1}, 'start_delay_sec'),
        ({'duration_sec': 0.0}, 'duration_sec'),
    ],
)
def test_invalid_configuration_is_rejected(overrides, message):
    with pytest.raises(ValueError, match=message):
        configuration(**overrides).validated()


def test_timeline_emits_scheduled_active_and_ended_once():
    timeline = ManualFaultTimeline(10.0, 12.0, 17.0)

    assert timeline.states_due(10.0) == [FaultStatus.SCHEDULED]
    assert timeline.states_due(11.9) == []
    assert timeline.states_due(12.0) == [FaultStatus.ACTIVE]
    assert timeline.states_due(20.0) == [FaultStatus.ENDED]
    assert timeline.states_due(30.0) == []
    assert timeline.complete is True


def test_late_timer_does_not_drop_active_before_ended():
    timeline = ManualFaultTimeline(1.0, 2.0, 3.0)

    assert timeline.states_due(4.0) == [
        FaultStatus.SCHEDULED,
        FaultStatus.ACTIVE,
        FaultStatus.ENDED,
    ]


def test_status_message_contains_manual_truth_window_without_data_topics():
    validated = configuration().validated()
    message = make_manual_fault_status(
        validated,
        state=FaultStatus.ACTIVE,
        stamp_sec=11.25,
        start_sec=12.0,
        end_sec=17.0,
    )

    assert message.header.stamp.sec == 11
    assert message.header.stamp.nanosec == 250_000_000
    assert message.scenario_id == 'manual_fault_event'
    assert message.event_id == 'camera_freeze_001'
    assert message.sensor == 'camera'
    assert message.model == 'freeze'
    assert message.state == FaultStatus.ACTIVE
    assert message.severity == pytest.approx(0.8)
    assert message.start_time.sec == 12
    assert message.end_time.sec == 17
    assert message.source_topic == ''
    assert message.faulted_topic == ''
    assert 'manual_event: true' in message.parameters_yaml
