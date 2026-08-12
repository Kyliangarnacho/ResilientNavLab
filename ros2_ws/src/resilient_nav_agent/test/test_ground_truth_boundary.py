"""End-to-end tests for the Agent Input versus Benchmark Truth boundary."""

import json

import pytest

from resilient_nav_agent.evidence import build_health_evidence
from resilient_nav_agent.incidents import build_offline_incident
from resilient_nav_agent.sanitizer import AgentInputSanitizer, SanitizationError


FORBIDDEN_TOKENS = (
    'faultstatus',
    '/fault_injection/status',
    '/faulted/',
    'scenario_id',
    'scenario_seed',
    'parameters_yaml',
    'ground_truth',
    'benchmark_answer',
)


def health_mapping(sensor, source_topic, state, detected_fault):
    """Return a current SensorHealth-shaped plain mapping."""
    return {
        'header': {
            'stamp': {'sec': 22, 'nanosec': 0},
            'frame_id': '',
        },
        'sensor': sensor,
        'source_topic': source_topic,
        'state': state,
        'health_score': 0.5 if state == 2 else 1.0,
        'confidence': 0.9,
        'detected_fault': detected_fault,
        'reasons': ['deterministic_monitor_output'],
        'metric_names': ['message_age_sec'],
        'metric_values': [0.03],
        'window_start': {'sec': 20, 'nanosec': 0},
        'window_end': {'sec': 22, 'nanosec': 0},
        'sample_count': 10,
    }


def actual_fault_status_mapping():
    """Mirror every field currently declared by FaultStatus.msg."""
    return {
        'header': {'stamp': {'sec': 20, 'nanosec': 0}, 'frame_id': ''},
        'scenario_id': 'imu_bias_demo',
        'scenario_seed': 7,
        'event_id': 'bias-event-1',
        'source_topic': '/imu/data',
        'faulted_topic': '/faulted/imu/data',
        'sensor': 'imu',
        'model': 'bias',
        'start_time': {'sec': 20, 'nanosec': 0},
        'end_time': {'sec': 30, 'nanosec': 0},
        'state': 1,
        'severity': 1.0,
        'affected_fields': ['angular_velocity.z'],
        'parameters_yaml': 'bias_z: 0.15',
    }


def recursive_forbidden_hits(value):
    """Return forbidden token hits in keys and string leaves."""
    hits = []
    if isinstance(value, dict):
        for key, item in value.items():
            lowered = str(key).casefold()
            hits.extend(token for token in FORBIDDEN_TOKENS if token in lowered)
            hits.extend(recursive_forbidden_hits(item))
    elif isinstance(value, list):
        for item in value:
            hits.extend(recursive_forbidden_hits(item))
    elif isinstance(value, str):
        lowered = value.casefold()
        hits.extend(token for token in FORBIDDEN_TOKENS if token in lowered)
    return hits


def test_actual_fault_status_fields_cannot_pass_as_health():
    """The real FaultStatus field set must fail before canonicalization."""
    with pytest.raises(SanitizationError) as error:
        AgentInputSanitizer().sanitize_health(actual_fault_status_mapping())

    assert error.value.code == 'ground_truth_field'


def test_agent_input_contains_no_ground_truth():
    """Final Health, Evidence, and Incident JSON must have zero truth hits."""
    sanitizer = AgentInputSanitizer()
    trigger = sanitizer.sanitize_health(
        health_mapping('imu', '/faulted/imu/data', 2, 'bias')
    )
    related = sanitizer.sanitize_health(
        health_mapping('camera', '/camera/c920/image_raw', 1, 'none')
    )
    trigger_evidence = build_health_evidence(trigger, 'ev-imu-1')
    related_evidence = build_health_evidence(related, 'ev-camera-1')
    incident = build_offline_incident(
        trigger,
        [related],
        [trigger_evidence.evidence_id, related_evidence.evidence_id],
        incident_id='incident-imu-1',
    )
    payload = {
        'health': [
            trigger.model_dump(mode='json'),
            related.model_dump(mode='json'),
        ],
        'evidence': [
            trigger_evidence.model_dump(mode='json'),
            related_evidence.model_dump(mode='json'),
        ],
        'incident': incident.model_dump(mode='json'),
    }

    assert recursive_forbidden_hits(payload) == []
    assert recursive_forbidden_hits(json.loads(json.dumps(payload))) == []
