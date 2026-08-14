"""No-network contract tests for the explicit RA-1A real-model runner."""

import json

from agent_core.model import CompatibleModelClient

from resilient_nav_agent.benchmark.real import (
    REAL_SMOKE_CASE_IDS,
    RealModelBenchmarkStatus,
    run_real_model_benchmark,
    select_reference_cases,
)
from resilient_nav_agent.benchmark.runner import ReferencePipelineFakeCompletion
from resilient_nav_agent.benchmark.schemas import BenchmarkKind


FORBIDDEN_TOKENS = (
    'faultstatus',
    '/fault_injection/status',
    '/faulted/',
    'scenario_id',
    'scenario_seed',
    'parameters_yaml',
    'benchmark_answer',
    'truth_metadata',
)


def forbidden_hits(value):
    """Return protected truth markers found in a JSON-compatible payload."""
    text = json.dumps(value, ensure_ascii=False, default=str).casefold()
    return [token for token in FORBIDDEN_TOKENS if token in text]


def test_real_runner_uses_required_smoke_case_set():
    """Default selection is the requested representative four-case smoke."""
    cases = select_reference_cases()

    assert tuple(case.case_id for case in cases) == REAL_SMOKE_CASE_IDS


def test_real_runner_blocks_without_model_environment(monkeypatch):
    """Missing configuration must block before any model client is created."""
    from resilient_nav_agent.benchmark import real

    def missing_config(cls):
        raise ValueError('configuration intentionally absent')

    monkeypatch.setattr(real.ModelConfig, 'from_env', classmethod(missing_config))
    outcome = run_real_model_benchmark()

    assert outcome.status == RealModelBenchmarkStatus.BLOCKED
    assert outcome.report is None
    assert outcome.blocker == (
        'Real API execution blocked by missing environment configuration.'
    )
    assert outcome.selected_case_ids == REAL_SMOKE_CASE_IDS


def test_real_runner_uses_agent_view_with_compatible_client_only():
    """Robot client calls carry sanitized input through Analyzer, Tool, and final."""
    captured = []

    def client_factory(agent_input):
        completion = ReferencePipelineFakeCompletion(agent_input)

        def transport(**kwargs):
            captured.append(kwargs)
            return completion(**kwargs)

        return CompatibleModelClient(completion=transport)

    outcome = run_real_model_benchmark(
        case_ids=('CASE-001',),
        client_factory=client_factory,
    )

    assert outcome.status == RealModelBenchmarkStatus.COMPLETED
    assert outcome.report.benchmark_kind == BenchmarkKind.REAL_MODEL
    assert outcome.report.passed_case_count == 1
    assert outcome.report.total_model_requests == 3
    assert all(call['stream'] is False for call in captured)
    assert any('response_format' in call for call in captured)
    assert any('tools' in call for call in captured)
    assert all(forbidden_hits(call) == [] for call in captured)
