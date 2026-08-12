"""Tests for the separated offline Case Builder contracts."""

import json

from pydantic import ValidationError
import pytest

from resilient_nav_agent.benchmark.schemas import BenchmarkTruth
from resilient_nav_agent.offline.case_builder import OfflineCaseBuilder
from resilient_nav_agent.offline.fixtures import reference_robot_cases
from resilient_nav_agent.offline.schemas import OfflineAgentInput, OfflineRobotCase
from resilient_nav_agent.sanitizer import SanitizationError
from resilient_nav_agent.schemas import EvidenceType, HealthState


FORBIDDEN_TOKENS = (
    'faultstatus',
    '/fault_injection/status',
    '/faulted/',
    'scenario_id',
    'scenario_seed',
    'parameters_yaml',
    'benchmark_answer',
)


def forbidden_hits(value):
    """Return truth-token hits in nested keys and string leaves."""
    serialized = json.dumps(value, ensure_ascii=False).casefold()
    return [token for token in FORBIDDEN_TOKENS if token in serialized]


def test_eight_versioned_reference_cases_are_complete():
    """The V1 fixture set contains all requested robot health conditions."""
    cases = reference_robot_cases()

    assert [case.case_id for case in cases] == [
        f'CASE-{index:03d}' for index in range(1, 9)
    ]
    assert sum(case.agent_input.incident is not None for case in cases) == 7
    assert cases[-1].agent_input.incident is None
    assert all(
        case.benchmark_truth.truth_metadata['fixture_kind']
        == 'reference_fixture'
        for case in cases
    )


def test_agent_view_serialization_has_zero_truth_leakage():
    """Only the truth object can retain scenario and fault-model metadata."""
    for case in reference_robot_cases():
        agent_payload = case.agent_view().model_dump(mode='json')
        assert forbidden_hits(agent_payload) == []
        assert forbidden_hits(json.loads(json.dumps(agent_payload))) == []

    first = reference_robot_cases()[0]
    truth_payload = first.benchmark_truth.model_dump(mode='json')
    assert 'scenario_seed' in truth_payload['truth_metadata']
    assert 'parameters_yaml' in truth_payload['truth_metadata']


def test_healthy_case_has_health_evidence_but_no_fault_incident():
    """The control case must not fabricate an incident for uniformity."""
    healthy = reference_robot_cases()[-1].agent_view()

    assert healthy.incident is None
    assert healthy.evidence
    assert all(
        item.state == HealthState.HEALTHY
        for item in healthy.health_observations
    )


def test_camera_quality_hint_is_not_the_underexposed_answer():
    """CASE-006 prevents a direct detected-hint-to-answer implementation."""
    case = reference_robot_cases()[5]

    assert case.agent_input.incident.trigger_fault_hint == 'camera_quality'
    assert case.benchmark_truth.expected_fault_type == 'underexposed'


def test_wheel_case_retains_health_and_motion_evidence_types():
    """Commanded motion is an independent observation for wheel freeze."""
    case = reference_robot_cases()[1]

    assert {item.evidence_type for item in case.agent_input.evidence} >= {
        EvidenceType.HEALTH,
        EvidenceType.MOTION,
    }


def test_raw_truth_in_health_path_fails_closed():
    """Case Builder cannot sanitize a mixed observation/truth object."""
    raw = {
        'sensor': 'imu',
        'source_topic': '/imu/data',
        'state': 3,
        'health_score': 0.0,
        'confidence': 1.0,
        'detected_fault': 'bias',
        'reasons': [],
        'metric_names': [],
        'metric_values': [],
        'window_start_sec': 1.0,
        'window_end_sec': 2.0,
        'sample_count': 3,
        'scenario_id': 'prohibited',
    }
    truth = {
        'expected_incident': True,
        'expected_component': 'imu',
        'expected_fault_type': 'imu_bias',
        'expected_should_diagnose': True,
    }

    with pytest.raises(SanitizationError):
        OfflineCaseBuilder().build(
            case_id='CASE-MIXED',
            raw_health=[raw],
            truth_mapping=truth,
        )


def test_contracts_forbid_undeclared_cross_channel_fields():
    """Neither Agent nor truth contracts accept an accidental merged field."""
    healthy_input = reference_robot_cases()[-1].agent_view()
    with pytest.raises(ValidationError):
        OfflineAgentInput.model_validate({
            **healthy_input.model_dump(mode='json'),
            'benchmark_truth': {},
        })

    truth = reference_robot_cases()[-1].benchmark_truth
    with pytest.raises(ValidationError):
        BenchmarkTruth.model_validate({
            **truth.model_dump(mode='json'),
            'agent_input': {},
        })


def test_case_envelope_cannot_be_used_as_agent_input():
    """The runtime-facing type is distinct from the builder-only envelope."""
    case = reference_robot_cases()[0]

    assert isinstance(case, OfflineRobotCase)
    assert isinstance(case.agent_view(), OfflineAgentInput)
    with pytest.raises(ValidationError):
        OfflineAgentInput.model_validate(case.model_dump(mode='json'))
