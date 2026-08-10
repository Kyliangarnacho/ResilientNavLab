"""Tests for the display-only camera health watcher."""

import pytest
from resilient_nav_health_assessment.camera_health_watch import (
    CameraHealthWatchGate,
    format_camera_health_line,
)
from resilient_nav_interfaces.msg import SensorHealth


def _message(state=SensorHealth.HEALTHY, detected_fault='none'):
    message = SensorHealth()
    message.state = state
    message.health_score = 0.9
    message.confidence = 0.8
    message.detected_fault = detected_fault
    message.metric_names = [
        'message_age_sec',
        'rolling_observed_fps',
        'fingerprint_identical_duration_sec',
        'fault_confirmation_elapsed_sec',
        'recovery_elapsed_sec',
    ]
    message.metric_values = [0.1, 12.5, 1.25, 0.4, 0.0]
    return message


def test_watch_line_contains_required_state_and_metrics():
    line = format_camera_health_line(_message())

    assert line.startswith('state=HEALTHY ')
    assert 'health_score=0.900' in line
    assert 'confidence=0.800' in line
    assert 'detected_fault=none' in line
    assert 'message_age=0.100s' in line
    assert 'rolling_fps=12.500Hz' in line
    assert 'fingerprint_identical_duration=1.250s' in line
    assert 'confirmation_elapsed=0.400s' in line
    assert 'recovery_elapsed=0.000s' in line
    assert '\n' not in line


def test_watch_prints_changes_immediately_and_unchanged_at_low_rate():
    gate = CameraHealthWatchGate(5.0)
    healthy = _message()
    degraded = _message(SensorHealth.DEGRADED, 'freeze')

    assert gate.should_print(healthy, 0.0) is True
    assert gate.should_print(healthy, 1.0) is False
    assert gate.should_print(healthy, 5.0) is True
    assert gate.should_print(degraded, 5.1) is True
    assert gate.should_print(degraded, 6.0) is False


def test_detected_fault_change_is_also_printed_immediately():
    gate = CameraHealthWatchGate(10.0)

    assert gate.should_print(_message(SensorHealth.FAULT, 'freeze'), 0.0)
    assert gate.should_print(_message(SensorHealth.FAULT, 'stale'), 0.1)


def test_missing_metrics_format_as_nan_without_affecting_state():
    message = _message()
    message.metric_names = []
    message.metric_values = []

    line = format_camera_health_line(message)

    assert 'state=HEALTHY' in line
    assert 'message_age=nans' in line
    assert 'rolling_fps=nanHz' in line


def test_invalid_unchanged_period_is_rejected():
    with pytest.raises(ValueError, match='greater than zero'):
        CameraHealthWatchGate(0.0)
