"""Offline smoke tests against the real external agent-core package."""

import json
from pathlib import Path
from types import SimpleNamespace

import agent_core
from agent_core import AgentRuntime
from agent_core.observability import RunStatus

from resilient_nav_agent.extension import RobotDomainExtension


def response(content):
    """Return the minimal compatible chat-completion response shape."""
    message = SimpleNamespace(content=content, tool_calls=None)
    return SimpleNamespace(choices=[SimpleNamespace(message=message)])


class FakeRobotCompletion:
    """Deterministic two-call completion with no network or API client."""

    def __init__(self):
        self.calls = []

    def __call__(self, **kwargs):
        """Return structured analysis first and a short fake answer second."""
        self.calls.append(kwargs)
        if 'response_format' in kwargs:
            return response(json.dumps({
                'incident_category': 'sensor_health',
                'primary_component': 'imu',
                'evidence_sufficient': True,
                'needs_tools': False,
                'needs_more_evidence': False,
                'short_reason': 'Sanitized IMU health evidence is sufficient.',
            }))
        return response('Fake offline diagnosis: inspect the IMU bias evidence.')


def repository_root():
    """Find the ResilientNavLab repository without a machine path constant."""
    for candidate in Path(__file__).resolve().parents:
        if (candidate / '.git').is_dir():
            return candidate
    raise AssertionError('repository root not found')


def test_agent_core_import_resolves_outside_this_repository():
    """The dependency must resolve to the independent sibling repository."""
    core_path = Path(agent_core.__file__).resolve()

    assert not core_path.is_relative_to(repository_root())
    assert core_path.name == '__init__.py'
    assert core_path.parent.name == 'agent_core'


def test_robot_extension_runs_through_agent_runtime_with_fake_completion():
    """The Robot extension must produce a valid external AgentResult."""
    completion = FakeRobotCompletion()
    result = AgentRuntime(RobotDomainExtension(), completion).run(
        'offline-incident-1',
        'Diagnose the sanitized IMU incident.',
    )

    assert result.answer.startswith('Fake offline diagnosis:')
    assert result.route.route == 'diagnose'
    assert result.trace.status == RunStatus.COMPLETED
    assert result.analysis['primary_component'] == 'imu'
    assert result.tool_records == []
    assert len(completion.calls) == 2
    assert all('api_key' not in call for call in completion.calls)


def test_invalid_analysis_uses_safe_fallback_route():
    """Malformed fake analysis must fall back to needs-more-evidence."""
    calls = []

    def completion(**kwargs):
        calls.append(kwargs)
        if 'response_format' in kwargs:
            return response('not-json')
        return response('More read-only health evidence is required.')

    result = AgentRuntime(RobotDomainExtension(), completion).run(
        'offline-incident-2',
        'Diagnose an incomplete sanitized incident.',
    )

    assert result.route.route == 'needs_more_evidence'
    assert result.trace.metadata['analysis_fallback'] is True
    assert result.trace.status == RunStatus.COMPLETED
    assert len(calls) == 2
