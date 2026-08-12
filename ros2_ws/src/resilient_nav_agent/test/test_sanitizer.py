"""Tests for the fail-closed Agent input Sanitizer."""

import math

import pytest

from resilient_nav_agent.sanitizer import AgentInputSanitizer, SanitizationError
from resilient_nav_agent.schemas import HealthState


def sensor_health_fixture(
    sensor='imu',
    source_topic='/faulted/imu/data',
    state=3,
    detected_fault='bias',
    metric_names=None,
    metric_values=None,
):
    """Build a mapping faithful to the current SensorHealth.msg fields."""
    return {
        'header': {
            'stamp': {'sec': 12, 'nanosec': 100_000_000},
            'frame_id': '',
        },
        'sensor': sensor,
        'source_topic': source_topic,
        'state': state,
        'health_score': 0.0 if state == 3 else 0.5,
        'confidence': 0.9,
        'detected_fault': detected_fault,
        'reasons': ['deterministic_health_rule_triggered', ''],
        'metric_names': metric_names or ['message_age_sec', 'stamp_age_sec'],
        'metric_values': metric_values or [0.02, 0.03],
        'window_start': {'sec': 10, 'nanosec': 0},
        'window_end': {'sec': 12, 'nanosec': 0},
        'sample_count': 20,
    }


@pytest.mark.parametrize(
    'sensor,source_topic,state,fault_hint,names,values,component',
    [
        (
            'imu',
            '/faulted/imu/data',
            3,
            'bias',
            ['imu_wheel_corrected_residual_mean_rad_s'],
            [0.15],
            'imu',
        ),
        (
            'wheel',
            '/faulted/wheel/odometry',
            2,
            'freeze',
            ['wheel_pose_span_m', 'commanded_linear_abs_mps'],
            [0.0, 0.2],
            'wheel',
        ),
        (
            'scan',
            '/faulted/scan',
            3,
            'sector_blindness',
            ['scan_nan_ratio', 'longest_nan_sector_width_rad'],
            [0.1, 1.0],
            'scan',
        ),
        (
            'camera',
            '/camera/c920/image_raw',
            2,
            'underexposed',
            ['message_age_sec', 'mean_gray', 'laplacian_variance'],
            [0.04, 4.0, 8.0],
            'camera',
        ),
    ],
)
def test_current_health_types_convert_to_canonical_observations(
    sensor,
    source_topic,
    state,
    fault_hint,
    names,
    values,
    component,
):
    """IMU, wheel, scan, and camera mappings share one canonical contract."""
    raw = sensor_health_fixture(
        sensor=sensor,
        source_topic=source_topic,
        state=state,
        detected_fault=fault_hint,
        metric_names=names,
        metric_values=values,
    )
    observation = AgentInputSanitizer().sanitize_health(raw)

    assert observation.component == component
    assert observation.detected_fault_hint == fault_hint
    assert observation.metrics == dict(zip(names, values))
    assert observation.reasons == ['deterministic_health_rule_triggered']
    assert 'source_topic' not in observation.model_dump()


def test_faulted_source_is_used_only_for_component_and_removed():
    """Experiment topic names may be read for normalization but never emitted."""
    raw = sensor_health_fixture()
    observation = AgentInputSanitizer().sanitize_health(raw)
    serialized = observation.model_dump_json()

    assert observation.component == 'imu'
    assert '/faulted/' not in serialized
    assert 'source_topic' not in serialized


@pytest.mark.parametrize(
    'field',
    [
        'FaultStatus',
        'fault_status',
        'fault_injection_status',
        'scenario_id',
        'scenario_seed',
        'event_id',
        'faulted_topic',
        'model',
        'start_time',
        'end_time',
        'severity',
        'parameters_yaml',
        'affected_fields',
        'ground_truth',
        'truth_label',
        'expected_fault',
        'expected_state',
        'fault_model_truth',
        'benchmark_answer',
    ],
)
def test_ground_truth_fields_are_rejected(field):
    """Every documented answer-bearing field fails closed."""
    raw = sensor_health_fixture()
    raw[field] = 'sensitive-fixture-value-123'

    with pytest.raises(SanitizationError) as error:
        AgentInputSanitizer().sanitize_health(raw)

    assert error.value.code == 'ground_truth_field'
    assert 'sensitive-fixture-value-123' not in str(error.value)


def test_nested_scenario_seed_is_rejected():
    """Recursive detection must catch truth hidden below a benign key."""
    raw = sensor_health_fixture()
    raw['metadata'] = {'scenario_seed': 17}

    with pytest.raises(SanitizationError) as error:
        AgentInputSanitizer().sanitize_health(raw)

    assert error.value.code == 'ground_truth_field'


def test_fault_injection_status_string_is_rejected():
    """Dangerous strings are rejected even when the field name is allowed."""
    raw = sensor_health_fixture(source_topic='/fault_injection/status')

    with pytest.raises(SanitizationError) as error:
        AgentInputSanitizer().sanitize_health(raw)

    assert error.value.code == 'dangerous_content'


def test_faulted_string_outside_source_topic_is_rejected():
    """A faulted topic leaked into Agent-facing text must fail closed."""
    raw = sensor_health_fixture()
    raw['reasons'] = ['read from /faulted/imu/data']

    with pytest.raises(SanitizationError) as error:
        AgentInputSanitizer().sanitize_health(raw)

    assert error.value.code == 'dangerous_content'


def test_metric_name_value_length_mismatch_is_rejected():
    """Parallel metric arrays must have identical lengths."""
    raw = sensor_health_fixture(
        metric_names=['message_age_sec', 'stamp_age_sec'],
        metric_values=[0.1],
    )

    with pytest.raises(SanitizationError) as error:
        AgentInputSanitizer().sanitize_health(raw)

    assert error.value.code == 'invalid_metrics'


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_non_finite_metric_is_rejected(value):
    """Non-finite metric values cannot enter canonical JSON."""
    raw = sensor_health_fixture(
        metric_names=['message_age_sec'],
        metric_values=[value],
    )

    with pytest.raises(SanitizationError):
        AgentInputSanitizer().sanitize_health(raw)


def test_detector_bias_hint_is_allowed():
    """A deterministic detector hint is observation data, not truth."""
    observation = AgentInputSanitizer().sanitize_health(
        sensor_health_fixture(detected_fault='bias')
    )

    assert observation.detected_fault_hint == 'bias'


def test_validation_error_suppresses_internal_exception_details():
    """Rejected values must surface only the stable domain error."""
    raw = sensor_health_fixture()
    raw['health_score'] = 'sensitive-invalid-score-456'

    with pytest.raises(SanitizationError) as error:
        AgentInputSanitizer().sanitize_health(raw)

    assert error.value.code == 'invalid_health'
    assert 'sensitive-invalid-score-456' not in str(error.value)
    assert error.value.__cause__ is None
    assert error.value.__context__ is None


def test_unknown_score_sentinel_becomes_none():
    """The existing SensorHealth -1 score sentinel becomes JSON null."""
    raw = sensor_health_fixture(
        state=0,
        detected_fault='unknown',
        metric_names=[],
        metric_values=[],
    )
    raw['health_score'] = -1.0
    observation = AgentInputSanitizer().sanitize_health(raw)

    assert observation.state == HealthState.UNKNOWN
    assert observation.health_score is None


@pytest.mark.parametrize('payload', ['data:image/png;base64,AAAA', 'base64,AAAA'])
def test_encoded_reason_payload_is_rejected(payload):
    """Data URLs and Base64 markers cannot be smuggled through reasons."""
    raw = sensor_health_fixture()
    raw['reasons'] = [payload]

    with pytest.raises(SanitizationError) as error:
        AgentInputSanitizer().sanitize_health(raw)

    assert error.value.code == 'encoded_payload'
