"""End-to-end and adversarial tests for the RA-1A offline diagnosis loop."""

import json
from types import SimpleNamespace

from agent_core.tools import ToolExecutionStatus
from pydantic import ValidationError
import pytest

from resilient_nav_agent.benchmark.runner import (
    ReferencePipelineFakeCompletion,
    run_reference_fake_benchmark,
)
from resilient_nav_agent.benchmark.schemas import BenchmarkKind
from resilient_nav_agent.benchmark.scorer import BenchmarkScorer
from resilient_nav_agent.offline.context import OfflineDiagnosisContext
from resilient_nav_agent.offline.fixtures import reference_robot_cases
from resilient_nav_agent.offline.runtime import (
    OfflineDiagnosisRunStatus,
    run_offline_diagnosis,
    StructuredFailureCode,
)
from resilient_nav_agent.tools import create_robot_tool_registry


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


def response(content, tool_calls=None):
    """Return the compatible completion response shape used by Core."""
    message = SimpleNamespace(content=content, tool_calls=tool_calls)
    return SimpleNamespace(choices=[SimpleNamespace(message=message)])


def tool_call(call_id, name, arguments):
    """Return one compatible model Tool call."""
    function = SimpleNamespace(name=name, arguments=json.dumps(arguments))
    return SimpleNamespace(id=call_id, type='function', function=function)


def forbidden_hits(value):
    """Find exact prohibited boundary tokens in a JSON-compatible object."""
    text = json.dumps(value, ensure_ascii=False, default=str).casefold()
    return [token for token in FORBIDDEN_TOKENS if token in text]


def diagnosed_payload(agent_input, *, evidence_ids=None, summary='Bounded result.'):
    """Build one schema-valid diagnosed payload for a scripted completion."""
    incident = agent_input.incident
    assert incident is not None
    references = evidence_ids
    if references is None:
        references = [
            item.evidence_id
            for item in agent_input.evidence
            if item.component == incident.trigger_component
        ]
    return {
        'incident_id': incident.incident_id,
        'status': 'diagnosed',
        'primary_hypothesis_id': 'H-1',
        'hypotheses': [{
            'hypothesis_id': 'H-1',
            'component': incident.trigger_component,
            'cause': incident.trigger_fault_hint or 'unspecified_anomaly',
            'support_level': 'medium',
            'supporting_evidence_ids': references,
            'contradicting_evidence_ids': [],
            'rationale': 'The cited sanitized evidence supports this candidate.',
        }],
        'missing_evidence': [],
        'recommended_checks': ['Inspect another sanitized read-only window.'],
        'summary': summary,
    }


def insufficient_payload(agent_input):
    """Build one strict insufficient-evidence payload."""
    return {
        'incident_id': agent_input.incident.incident_id,
        'status': 'insufficient_evidence',
        'primary_hypothesis_id': None,
        'hypotheses': [],
        'missing_evidence': ['A second independent sanitized observation.'],
        'recommended_checks': ['Inspect a later sanitized read-only window.'],
        'summary': 'The available evidence is insufficient for diagnosis.',
    }


class ScriptedCompletion:
    """Configurable Fake covering final output and Tool failure behavior."""

    def __init__(
        self,
        agent_input,
        final_output,
        *,
        calls=None,
        repair_output=None,
        needs_tools=False,
        needs_more_evidence=False,
    ):
        self.agent_input = agent_input
        self.final_output = final_output
        self.tool_calls = calls
        self.repair_output = repair_output
        self.needs_tools = needs_tools
        self.needs_more_evidence = needs_more_evidence
        self.calls = []

    def __call__(self, **kwargs):
        """Return the response appropriate to the Core execution phase."""
        self.calls.append(kwargs)
        if 'response_format' in kwargs:
            incident = self.agent_input.incident
            return response(json.dumps({
                'incident_category': (
                    'sensor_health' if incident is not None else 'healthy'
                ),
                'primary_component': (
                    incident.trigger_component if incident is not None else None
                ),
                'evidence_sufficient': not self.needs_more_evidence,
                'needs_tools': self.needs_tools,
                'needs_more_evidence': self.needs_more_evidence,
                'candidate_checks': [],
                'short_reason': 'Scripted bounded pipeline behavior.',
            }))
        if 'tools' in kwargs:
            return response(None, self.tool_calls or [])
        is_repair = any(
            item.get('role') == 'system'
            and item.get('content', '').startswith('Repair only')
            for item in kwargs['messages']
        )
        if is_repair:
            return response(self.repair_output or '{}')
        output = (
            json.dumps(self.final_output)
            if isinstance(self.final_output, dict)
            else self.final_output
        )
        return response(output)


def test_robot_tools_are_registered_and_return_bounded_observations():
    """All three Tools read only the immutable sanitized context."""
    agent_input = reference_robot_cases()[1].agent_view()
    context = OfflineDiagnosisContext.from_agent_input(agent_input)
    registry = create_robot_tool_registry(context)

    assert [
        item['function']['name'] for item in registry.definitions()
    ] == [
        'get_incident_health_snapshot',
        'compare_component_health',
        'inspect_metric_window',
    ]
    snapshot = registry.execute(
        tool_call_id='a',
        name='get_incident_health_snapshot',
        arguments_json='{}',
    )
    comparison = registry.execute(
        tool_call_id='b',
        name='compare_component_health',
        arguments_json='{"left_component":"wheel","right_component":"imu"}',
    )
    metric = registry.execute(
        tool_call_id='c',
        name='inspect_metric_window',
        arguments_json=(
            '{"component":"wheel","metric_name":"wheel_pose_span_m"}'
        ),
    )

    assert all(
        item.status == ToolExecutionStatus.SUCCESS
        for item in (snapshot, comparison, metric)
    )
    assert comparison.result['health_score_delta'] == -1.0
    assert metric.result['value'] == 0.0
    assert forbidden_hits([
        snapshot.model_dump(mode='json'),
        comparison.model_dump(mode='json'),
        metric.model_dump(mode='json'),
    ]) == []


def test_tool_context_is_deep_copied_and_tuple_bounded():
    """Tool handlers cannot mutate the input through shared list references."""
    agent_input = reference_robot_cases()[0].agent_view()
    context = OfflineDiagnosisContext.from_agent_input(agent_input)
    original_metric_names = set(context.health_observations[0].metrics)
    agent_input.health_observations[0].metrics['later_mutation'] = 1.0

    assert set(context.health_observations[0].metrics) == original_metric_names
    with pytest.raises(ValidationError):
        context.evidence += ()


@pytest.mark.parametrize(
    'name,arguments,error_type',
    [
        ('missing_tool', '{}', 'unknown_tool'),
        (
            'compare_component_health',
            '{"left_component":"imu","right_component":"imu"}',
            'validation_error',
        ),
        (
            'inspect_metric_window',
            '{"component":"imu","metric_name":"absent_metric"}',
            'execution_error',
        ),
    ],
)
def test_tool_failures_are_safe_and_deterministic(name, arguments, error_type):
    """Unknown, invalid, and unavailable Tool requests fail as records."""
    context = OfflineDiagnosisContext.from_agent_input(
        reference_robot_cases()[0].agent_view()
    )
    record = create_robot_tool_registry(context).execute(
        tool_call_id='bad',
        name=name,
        arguments_json=arguments,
    )

    assert record.status == ToolExecutionStatus.ERROR
    assert record.error_type == error_type
    assert forbidden_hits(record.model_dump(mode='json')) == []


def test_reference_fake_batch_runs_all_cases_and_is_clearly_labeled():
    """The repeatable report covers eight fixtures without claiming intelligence."""
    report = run_reference_fake_benchmark()

    assert report.benchmark_kind == BenchmarkKind.PIPELINE_FAKE
    assert report.case_count == 8
    assert report.diagnostic_case_count == 7
    assert report.healthy_case_count == 1
    assert report.passed_case_count == 8
    assert report.component_accuracy == 1.0
    assert report.fault_type_accuracy == 1.0
    assert report.top_k_coverage == 1.0
    assert report.evidence_reference_validity == 1.0
    assert report.ground_truth_leakage_count == 0
    assert report.false_diagnosis_count == 0
    assert report.total_tool_calls == 6
    assert report.total_model_requests == 22
    assert report.status_counts == {
        'completed': 7,
        'no_diagnosis': 1,
        'insufficient_evidence': 0,
        'blocked': 0,
        'structured_failure': 0,
    }


def test_runner_has_tool_and_no_tool_paths_with_strict_results():
    """Reference behavior exercises Core Tool orchestration and direct output."""
    tool_input = reference_robot_cases()[0].agent_view()
    tool_fake = ReferencePipelineFakeCompletion(tool_input)
    tool_run = run_offline_diagnosis(tool_input, tool_fake)
    direct_input = reference_robot_cases()[3].agent_view()
    direct_fake = ReferencePipelineFakeCompletion(direct_input)
    direct_run = run_offline_diagnosis(direct_input, direct_fake)

    assert tool_run.status == OfflineDiagnosisRunStatus.COMPLETED
    assert len(tool_run.tool_records) == 1
    assert tool_run.model_request_count == 3
    assert direct_run.status == OfflineDiagnosisRunStatus.COMPLETED
    assert direct_run.tool_records == ()
    assert direct_run.model_request_count == 2


def test_malformed_final_json_becomes_structured_failure():
    """Plain model text is never wrapped into a successful diagnosis."""
    agent_input = reference_robot_cases()[0].agent_view()
    run = run_offline_diagnosis(
        agent_input,
        ScriptedCompletion(agent_input, 'not-json'),
    )

    assert run.status == OfflineDiagnosisRunStatus.STRUCTURED_FAILURE
    assert run.structured_failure_code == StructuredFailureCode.INVALID_JSON
    assert run.diagnosis_result is None


def test_malformed_healthy_output_cannot_pass_by_merely_avoiding_diagnosis():
    """Healthy scoring requires an explicit strict no-diagnosis result."""
    case = reference_robot_cases()[-1]
    agent_input = case.agent_view()
    run = run_offline_diagnosis(
        agent_input,
        ScriptedCompletion(agent_input, 'not-json'),
    )
    score = BenchmarkScorer().score(run, case.benchmark_truth)

    assert run.status == OfflineDiagnosisRunStatus.STRUCTURED_FAILURE
    assert score.diagnosis_expectation_match is False
    assert score.passed is False


def test_insufficient_evidence_is_preserved_and_scores_as_not_diagnosed():
    """A valid uncertainty result is distinct from malformed output."""
    case = reference_robot_cases()[0]
    agent_input = case.agent_view()
    run = run_offline_diagnosis(
        agent_input,
        ScriptedCompletion(
            agent_input,
            insufficient_payload(agent_input),
            needs_more_evidence=True,
        ),
    )
    score = BenchmarkScorer().score(run, case.benchmark_truth)

    assert run.status == OfflineDiagnosisRunStatus.INSUFFICIENT_EVIDENCE
    assert run.structured_failure_code is None
    assert score.diagnosis_expectation_match is False
    assert score.passed is False


def test_unknown_and_invalid_tool_calls_remain_auditable():
    """Core records both unknown calls and failed bounded argument repair."""
    case = reference_robot_cases()[0]
    agent_input = case.agent_view()
    final = diagnosed_payload(agent_input)
    unknown = ScriptedCompletion(
        agent_input,
        final,
        needs_tools=True,
        calls=[tool_call('unknown', 'not_registered', {})],
    )
    unknown_run = run_offline_diagnosis(agent_input, unknown)
    invalid = ScriptedCompletion(
        agent_input,
        final,
        needs_tools=True,
        calls=[tool_call('invalid', 'inspect_metric_window', {'component': 'imu'})],
        repair_output=json.dumps({
            'repairs': [{
                'tool_call_id': 'invalid',
                'name': 'inspect_metric_window',
                'arguments': {'component': 'imu'},
            }],
        }),
    )
    invalid_run = run_offline_diagnosis(agent_input, invalid)

    assert unknown_run.tool_records[-1].error_type == 'unknown_tool'
    assert len(invalid_run.tool_records) == 2
    assert all(
        item.error_type == 'validation_error'
        for item in invalid_run.tool_records
    )
    assert invalid_run.model_request_count == 4


def test_partial_tool_failure_keeps_success_and_failure_records():
    """One failed sibling does not erase a successful read-only Tool result."""
    case = reference_robot_cases()[0]
    agent_input = case.agent_view()
    fake = ScriptedCompletion(
        agent_input,
        diagnosed_payload(agent_input),
        needs_tools=True,
        calls=[
            tool_call('valid', 'get_incident_health_snapshot', {}),
            tool_call('invalid', 'not_registered', {}),
        ],
    )
    run = run_offline_diagnosis(agent_input, fake)
    score = BenchmarkScorer().score(run, case.benchmark_truth)

    assert [item.status for item in run.tool_records] == [
        ToolExecutionStatus.SUCCESS,
        ToolExecutionStatus.ERROR,
    ]
    assert run.trace.status.value == 'partial'
    assert score.tool_call_count == 2
    assert score.tool_failure_count == 1


def test_scorer_catches_nonexistent_evidence_and_unsupported_claim():
    """Schema-valid invented evidence still fails deterministic scoring."""
    case = reference_robot_cases()[0]
    agent_input = case.agent_view()
    run = run_offline_diagnosis(
        agent_input,
        ScriptedCompletion(
            agent_input,
            diagnosed_payload(agent_input, evidence_ids=['E-NOT-FOUND']),
        ),
    )
    score = BenchmarkScorer().score(run, case.benchmark_truth)

    assert run.status == OfflineDiagnosisRunStatus.COMPLETED
    assert score.evidence_references_valid is False
    assert score.invalid_evidence_reference_count == 1
    assert score.unsupported_claim_count == 1
    assert score.passed is False


def test_detector_hint_wording_does_not_drive_truth_scoring():
    """CASE-006 passes from its derived cause, not the camera_quality hint."""
    case = reference_robot_cases()[5]
    agent_input = case.agent_view()
    run = run_offline_diagnosis(
        agent_input,
        ReferencePipelineFakeCompletion(agent_input),
    )
    score = BenchmarkScorer().score(run, case.benchmark_truth)

    assert agent_input.incident.trigger_fault_hint == 'camera_quality'
    assert score.actual_primary_fault_type == 'underexposed'
    assert score.fault_type_correct is True


def test_output_that_mentions_prohibited_evaluation_data_is_rejected():
    """A model attempt to expose protected data becomes a leakage failure."""
    case = reference_robot_cases()[0]
    agent_input = case.agent_view()
    payload = diagnosed_payload(
        agent_input,
        summary='Ground Truth says this is the expected answer.',
    )
    run = run_offline_diagnosis(
        agent_input,
        ScriptedCompletion(agent_input, payload),
    )
    score = BenchmarkScorer().score(run, case.benchmark_truth)

    assert run.structured_failure_code == StructuredFailureCode.PROHIBITED_OUTPUT
    assert run.output_leakage_detected is True
    assert score.ground_truth_leakage is True
    assert score.passed is False


def test_healthy_false_diagnosis_is_rejected_and_scored():
    """A schema-valid diagnosis attempt on healthy input cannot pass silently."""
    case = reference_robot_cases()[-1]
    agent_input = case.agent_view()
    payload = {
        'incident_id': 'INVENTED-INCIDENT',
        'status': 'diagnosed',
        'primary_hypothesis_id': 'H-1',
        'hypotheses': [{
            'hypothesis_id': 'H-1',
            'component': 'imu',
            'cause': 'bias',
            'support_level': 'low',
            'supporting_evidence_ids': [agent_input.evidence[0].evidence_id],
            'contradicting_evidence_ids': [],
            'rationale': 'A false control-case diagnosis.',
        }],
        'missing_evidence': [],
        'recommended_checks': [],
        'summary': 'A false diagnosis was attempted.',
    }
    run = run_offline_diagnosis(
        agent_input,
        ScriptedCompletion(agent_input, payload),
    )
    score = BenchmarkScorer().score(run, case.benchmark_truth)

    assert run.structured_failure_code == StructuredFailureCode.CONTRACT_MISMATCH
    assert run.diagnosis_attempted is True
    assert score.false_diagnosis is True
    assert score.passed is False


def test_all_agent_side_messages_context_tools_and_traces_are_truth_free():
    """New Runtime and Tool projections retain zero prohibited token hits."""
    for case in reference_robot_cases():
        agent_input = case.agent_view()
        fake = ReferencePipelineFakeCompletion(agent_input)
        run = run_offline_diagnosis(agent_input, fake)
        payloads = [
            agent_input.model_dump(mode='json'),
            fake.calls,
            run.model_dump(mode='json'),
        ]
        if agent_input.incident is not None:
            context = OfflineDiagnosisContext.from_agent_input(agent_input)
            payloads.append(context.model_dump(mode='json'))
            snapshot = create_robot_tool_registry(context).execute(
                tool_call_id='audit',
                name='get_incident_health_snapshot',
                arguments_json='{}',
            )
            payloads.append(snapshot.model_dump(mode='json'))

        assert all(forbidden_hits(payload) == [] for payload in payloads)


def test_runner_rejects_builder_envelope_instead_of_reaching_truth():
    """Only OfflineAgentInput is accepted at the Runtime boundary."""
    case = reference_robot_cases()[0]

    with pytest.raises(TypeError):
        run_offline_diagnosis(
            case,
            ReferencePipelineFakeCompletion(case.agent_view()),
        )
