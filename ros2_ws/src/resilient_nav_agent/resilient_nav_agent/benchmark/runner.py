"""Repeatable batch execution for the offline Robot diagnosis benchmark."""

from __future__ import annotations

from collections import Counter
from collections.abc import Callable, Sequence
import json
from types import SimpleNamespace
from typing import Any, Mapping

from resilient_nav_agent.benchmark.schemas import (
    BenchmarkKind,
    BenchmarkReport,
)
from resilient_nav_agent.benchmark.scorer import BenchmarkScorer
from resilient_nav_agent.offline.fixtures import reference_robot_cases
from resilient_nav_agent.offline.runtime import (
    OfflineDiagnosisRunStatus,
    run_offline_diagnosis,
)
from resilient_nav_agent.offline.schemas import OfflineAgentInput, OfflineRobotCase


Completion = Callable[..., Any]
CompletionFactory = Callable[[OfflineAgentInput], Completion]


def _response(content: str | None, tool_calls=None):
    message = SimpleNamespace(content=content, tool_calls=tool_calls)
    return SimpleNamespace(choices=[SimpleNamespace(message=message)])


def _tool_call(call_id: str, name: str, arguments: dict[str, Any]):
    return SimpleNamespace(
        id=call_id,
        type='function',
        function=SimpleNamespace(
            name=name,
            arguments=json.dumps(arguments, separators=(',', ':')),
        ),
    )


class ReferencePipelineFakeCompletion:
    """Deterministic fixture completion that measures pipeline, not reasoning."""

    def __init__(self, agent_input: OfflineAgentInput):
        self._input = agent_input.model_copy(deep=True)
        self.calls: list[dict[str, Any]] = []

    def _needs_tools(self) -> bool:
        incident = self._input.incident
        return bool(
            incident is not None
            and incident.trigger_fault_hint != 'stale'
        )

    def _analysis(self) -> dict[str, Any]:
        incident = self._input.incident
        if incident is None:
            return {
                'incident_category': 'healthy',
                'primary_component': None,
                'evidence_sufficient': True,
                'needs_tools': False,
                'needs_more_evidence': False,
                'candidate_checks': [],
                'short_reason': 'Sanitized observations do not form an incident.',
            }
        return {
            'incident_category': 'sensor_health',
            'primary_component': incident.trigger_component,
            'evidence_sufficient': True,
            'needs_tools': self._needs_tools(),
            'needs_more_evidence': False,
            'candidate_checks': ['Inspect only the current sanitized context.'],
            'short_reason': 'The bounded health evidence supports diagnosis.',
        }

    def _selection(self):
        incident = self._input.incident
        assert incident is not None
        component = incident.trigger_component
        if len(self._input.health_observations) > 1:
            other = next(
                item.component
                for item in self._input.health_observations
                if item.component != component
            )
            return _response(
                None,
                [_tool_call(
                    'tool-1',
                    'compare_component_health',
                    {
                        'left_component': component,
                        'right_component': other,
                    },
                )],
            )
        if component == 'camera' and incident.trigger_fault_hint == 'freeze':
            return _response(
                None,
                [_tool_call('tool-1', 'get_incident_health_snapshot', {})],
            )
        metrics = incident.trigger_health.metrics
        metric_name = next(
            (
                name for name in sorted(metrics)
                if name != 'message_age_sec'
            ),
            sorted(metrics)[0],
        )
        return _response(
            None,
            [_tool_call(
                'tool-1',
                'inspect_metric_window',
                {'component': component, 'metric_name': metric_name},
            )],
        )

    def _cause(self) -> str:
        incident = self._input.incident
        assert incident is not None
        hint = incident.trigger_fault_hint or 'unspecified_anomaly'
        if hint != 'camera_quality':
            return hint
        metrics = incident.trigger_health.metrics
        if metrics.get('exposure_dark_candidate', 0.0) > 0.0:
            return 'underexposed'
        if metrics.get('blur_candidate', 0.0) > 0.0:
            return 'blurred'
        return 'camera_quality_anomaly'

    def _diagnosis(self) -> dict[str, Any]:
        incident = self._input.incident
        if incident is None:
            return {
                'incident_id': None,
                'status': 'no_diagnosis',
                'primary_hypothesis_id': None,
                'hypotheses': [],
                'missing_evidence': [],
                'recommended_checks': [],
                'summary': (
                    'Sanitized observations remain healthy; diagnosis is not '
                    'warranted.'
                ),
            }
        supporting = [
            item.evidence_id
            for item in self._input.evidence
            if item.component == incident.trigger_component
        ]
        return {
            'incident_id': incident.incident_id,
            'status': 'diagnosed',
            'primary_hypothesis_id': 'H-PRIMARY',
            'hypotheses': [{
                'hypothesis_id': 'H-PRIMARY',
                'component': incident.trigger_component,
                'cause': self._cause(),
                'support_level': 'high',
                'supporting_evidence_ids': supporting,
                'contradicting_evidence_ids': [],
                'rationale': (
                    'The cited sanitized observations support this bounded '
                    'fixture diagnosis.'
                ),
            }],
            'missing_evidence': [],
            'recommended_checks': [
                'Inspect a later sanitized read-only health window.'
            ],
            'summary': 'A bounded fixture diagnosis was produced from cited evidence.',
        }

    def __call__(self, **kwargs):
        """Return Analyzer, Tool-selection, or strict final fixture output."""
        self.calls.append(kwargs)
        if 'response_format' in kwargs:
            return _response(json.dumps(self._analysis()))
        if 'tools' in kwargs:
            return self._selection()
        return _response(json.dumps(self._diagnosis()))


def _ratio(values: Sequence[bool]) -> float | None:
    if not values:
        return None
    return sum(values) / len(values)


class BatchBenchmarkRunner:
    """Execute cases sequentially and score each isolated truth channel."""

    def __init__(self, scorer: BenchmarkScorer | None = None):
        self._scorer = scorer or BenchmarkScorer()

    def run(
        self,
        cases: Sequence[OfflineRobotCase],
        completion_factory: CompletionFactory,
        *,
        benchmark_kind: BenchmarkKind,
        request_options: Mapping[str, Any] | None = None,
        analysis_response_format: Mapping[str, Any] | None = {
            'type': 'json_object'
        },
    ) -> BenchmarkReport:
        """Return one aggregate report without feeding truth to completions."""
        runs = []
        results = []
        truths = []
        for case in cases:
            agent_input = case.agent_view()
            completion = completion_factory(agent_input)
            run = run_offline_diagnosis(
                agent_input,
                completion,
                request_options=dict(request_options or {}),
                analysis_response_format=analysis_response_format,
            )
            result = self._scorer.score(run, case.benchmark_truth)
            runs.append(run)
            results.append(result)
            truths.append(case.benchmark_truth)

        component_scores = [
            item.primary_component_correct
            for item in results
            if item.primary_component_correct is not None
        ]
        fault_scores = [
            item.fault_type_correct
            for item in results
            if item.fault_type_correct is not None
        ]
        top_k_scores = [
            item.top_k_truth_covered
            for item in results
            if item.top_k_truth_covered is not None
        ]
        observed_statuses = Counter(item.run_status for item in results)
        status_counts = {
            status.value: observed_statuses.get(status.value, 0)
            for status in OfflineDiagnosisRunStatus
        }
        return BenchmarkReport(
            benchmark_kind=benchmark_kind,
            case_count=len(results),
            diagnostic_case_count=sum(
                item.expected_should_diagnose for item in truths
            ),
            healthy_case_count=sum(
                not item.expected_should_diagnose for item in truths
            ),
            passed_case_count=sum(item.passed for item in results),
            component_accuracy=_ratio(component_scores),
            fault_type_accuracy=_ratio(fault_scores),
            top_k_coverage=_ratio(top_k_scores),
            evidence_reference_validity=(
                sum(item.evidence_references_valid for item in results)
                / len(results)
                if results else 1.0
            ),
            required_evidence_success_rate=(
                sum(item.minimum_required_evidence_satisfied for item in results)
                / len(results)
                if results else 1.0
            ),
            ground_truth_leakage_count=sum(
                item.ground_truth_leakage for item in results
            ),
            false_diagnosis_count=sum(item.false_diagnosis for item in results),
            total_tool_calls=sum(item.tool_call_count for item in results),
            total_tool_failures=sum(item.tool_failure_count for item in results),
            total_model_requests=sum(item.model_request_count for item in results),
            total_latency_ms=sum(item.latency_ms for item in runs),
            status_counts=status_counts,
            case_results=results,
        )


def run_reference_fake_benchmark() -> BenchmarkReport:
    """Run all eight fixtures with the explicitly non-intelligent Fake."""
    return BatchBenchmarkRunner().run(
        reference_robot_cases(),
        ReferencePipelineFakeCompletion,
        benchmark_kind=BenchmarkKind.PIPELINE_FAKE,
    )


def main() -> None:
    """Print the local Fake pipeline report as JSON."""
    report = run_reference_fake_benchmark()
    print(report.model_dump_json(indent=2))


__all__ = [
    'BatchBenchmarkRunner',
    'ReferencePipelineFakeCompletion',
    'run_reference_fake_benchmark',
]
