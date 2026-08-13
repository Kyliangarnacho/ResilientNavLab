"""Tests for the formal RA-1A Analyzer and Robot routing policy."""

import json
from types import SimpleNamespace

from agent_core import AgentRuntime
from pydantic import ValidationError
import pytest

from resilient_nav_agent.extension import (
    IncidentCategory,
    RobotAnalysis,
    RobotDomainExtension,
)
from resilient_nav_agent.offline.context import OfflineDiagnosisContext
from resilient_nav_agent.offline.fixtures import reference_robot_cases
from resilient_nav_agent.prompts import SYSTEM_PROMPT


FORBIDDEN_TOKENS = (
    'faultstatus',
    '/fault_injection/status',
    '/faulted/',
    'scenario_id',
    'scenario_seed',
    'parameters_yaml',
    'benchmark_answer',
)


def response(content):
    """Return a minimal compatible fake completion response."""
    message = SimpleNamespace(content=content, tool_calls=None)
    return SimpleNamespace(choices=[SimpleNamespace(message=message)])


def forbidden_hits(value):
    """Find prohibited truth tokens in a serializable Agent-side object."""
    payload = json.dumps(value, ensure_ascii=False).casefold()
    return [token for token in FORBIDDEN_TOKENS if token in payload]


def analysis(**overrides):
    """Return a valid route-only RobotAnalysis."""
    values = {
        'incident_category': IncidentCategory.SENSOR_HEALTH,
        'primary_component': 'imu',
        'evidence_sufficient': True,
        'needs_tools': False,
        'needs_more_evidence': False,
        'candidate_checks': [],
        'short_reason': 'Sanitized health evidence is sufficient.',
    }
    values.update(overrides)
    return RobotAnalysis(**values)


def test_system_prompt_v1_contains_diagnosis_and_safety_contract():
    """The prompt states evidence, detector-hint, truth, and control rules."""
    lowered = SYSTEM_PROMPT.casefold()

    assert 'resilientnav robot diagnostic agent' in lowered
    assert 'evidence_id' in lowered
    assert 'detected_fault_hint' in lowered
    assert 'never as a verified answer' in lowered
    assert 'insufficient_evidence' in lowered
    assert 'never control the robot' in lowered
    assert 'diagnosisresult' in lowered


def test_analyzer_schema_cannot_take_final_hypotheses():
    """Analyzer and DiagnosisResult remain separate contracts."""
    with pytest.raises(ValidationError):
        RobotAnalysis.model_validate({
            **analysis().model_dump(mode='json'),
            'hypotheses': [],
        })


def test_formal_routes_are_diagnose_needs_more_evidence_and_blocked():
    """The no-Tool V1 domain exposes exactly the authorized route behavior."""
    extension = RobotDomainExtension()

    assert extension.route(analysis(), None).route == 'diagnose'
    needs_more = extension.route(
        analysis(
            evidence_sufficient=False,
            needs_more_evidence=True,
            primary_component=None,
        ),
        None,
    )
    assert needs_more.route == 'needs_more_evidence'
    assert needs_more.should_answer is True
    blocked = extension.route(analysis(needs_tools=True), None)
    assert blocked.route == 'blocked'
    assert blocked.should_answer is False


def test_agent_context_tool_context_and_trace_have_zero_truth_leakage():
    """All currently implemented Agent-side projections remain truth-free."""
    agent_input = reference_robot_cases()[0].agent_view()
    extension = RobotDomainExtension(agent_input)
    context = OfflineDiagnosisContext.from_agent_input(agent_input)
    calls = []

    def completion(**kwargs):
        calls.append(kwargs)
        if 'response_format' in kwargs:
            return response(json.dumps(analysis().model_dump(mode='json')))
        return response(json.dumps({
            'incident_id': agent_input.incident.incident_id,
            'status': 'insufficient_evidence',
            'primary_hypothesis_id': None,
            'hypotheses': [],
            'missing_evidence': ['A later sanitized residual window.'],
            'recommended_checks': ['Inspect a later read-only metric window.'],
            'summary': 'More evidence is required before diagnosis.',
        }))

    result = AgentRuntime(extension, completion).run(
        'offline-case-001',
        'Diagnose the current sanitized incident.',
    )
    agent_payloads = [
        agent_input.model_dump(mode='json'),
        extension.domain_state_context('offline-case-001', 'diagnose'),
        context.model_dump(mode='json'),
        result.context.model_dump(mode='json'),
        result.trace.model_dump(mode='json'),
    ]

    assert all(forbidden_hits(payload) == [] for payload in agent_payloads)
    assert len(calls) == 2
    assert all('api_key' not in call for call in calls)
