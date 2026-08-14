"""Deterministic V1 scoring of truth-free offline diagnosis runs."""

from __future__ import annotations

import re
from typing import Any

from agent_core.tools import ToolExecutionStatus

from resilient_nav_agent.benchmark.schemas import (
    BenchmarkCaseResult,
    BenchmarkTruth,
)
from resilient_nav_agent.offline.runtime import OfflineDiagnosisRun
from resilient_nav_agent.schemas import DiagnosisStatus


_LEAK_MARKERS = (
    'faultstatus',
    '/fault_injection/status',
    '/faulted/',
    'scenario_id',
    'scenario_seed',
    'parameters_yaml',
    'benchmark_answer',
    'truth_metadata',
    'ground_truth',
    'ground truth',
    'fault model truth',
)
_NON_ALNUM = re.compile(r'[^a-z0-9]+')


def _normalized_label(value: str) -> str:
    return _NON_ALNUM.sub('_', value.casefold()).strip('_')


def _contains_leak(value: Any) -> bool:
    if isinstance(value, dict):
        return any(
            _contains_leak(str(key)) or _contains_leak(item)
            for key, item in value.items()
        )
    if isinstance(value, (list, tuple)):
        return any(_contains_leak(item) for item in value)
    if not isinstance(value, str):
        return False
    lowered = value.casefold()
    return any(marker in lowered for marker in _LEAK_MARKERS)


def _latest_tool_records(run: OfflineDiagnosisRun):
    latest = {}
    for record in run.tool_records:
        latest[record.tool_call_id] = record
    return list(latest.values())


class BenchmarkScorer:
    """Apply general evidence, diagnosis, safety, and usage rules."""

    def __init__(self, *, top_k: int = 3):
        if top_k < 1:
            raise ValueError('top_k must be positive')
        self._top_k = top_k

    def score(
        self,
        run: OfflineDiagnosisRun,
        truth: BenchmarkTruth,
    ) -> BenchmarkCaseResult:
        """Score one completed run without invoking a model."""
        if run.case_id != truth.case_id:
            raise ValueError('run and BenchmarkTruth case IDs do not match')

        diagnosis = run.diagnosis_result
        diagnosed = bool(
            diagnosis is not None
            and diagnosis.status == DiagnosisStatus.DIAGNOSED
        )
        primary = None
        if diagnosis is not None and diagnosis.primary_hypothesis_id is not None:
            primary = next(
                (
                    item for item in diagnosis.hypotheses
                    if item.hypothesis_id == diagnosis.primary_hypothesis_id
                ),
                None,
            )

        accepted_components = {
            item.casefold()
            for item in (
                [truth.expected_component] if truth.expected_component else []
            ) + truth.acceptable_components
        }
        accepted_faults = {
            _normalized_label(item)
            for item in (
                [truth.expected_fault_type] if truth.expected_fault_type else []
            ) + truth.acceptable_fault_types
        }
        component_correct = None
        fault_correct = None
        top_k_covered = None
        if truth.expected_should_diagnose:
            component_correct = bool(
                primary is not None
                and primary.component.casefold() in accepted_components
            )
            fault_correct = bool(
                primary is not None
                and _normalized_label(primary.cause) in accepted_faults
            )
            top_k_covered = bool(
                diagnosis is not None
                and any(
                    item.component.casefold() in accepted_components
                    and _normalized_label(item.cause) in accepted_faults
                    for item in diagnosis.hypotheses[:self._top_k]
                )
            )

        catalog = {item.evidence_id: item for item in run.evidence_catalog}
        hypotheses = diagnosis.hypotheses if diagnosis is not None else []
        referenced_ids = {
            evidence_id
            for hypothesis in hypotheses
            for evidence_id in (
                hypothesis.supporting_evidence_ids
                + hypothesis.contradicting_evidence_ids
            )
        }
        valid_ids = referenced_ids.intersection(catalog)
        invalid_ids = referenced_ids.difference(catalog)
        evidence_references_valid = not invalid_ids and (
            not diagnosed or bool(referenced_ids)
        )

        supporting_ids = {
            evidence_id
            for hypothesis in hypotheses
            for evidence_id in hypothesis.supporting_evidence_ids
            if evidence_id in catalog
        }
        supporting_types = {
            catalog[evidence_id].evidence_type
            for evidence_id in supporting_ids
        }
        required_types = set(truth.minimum_required_evidence_types)
        missing_types = sorted(
            required_types.difference(supporting_types),
            key=lambda item: item.value,
        )

        unsupported_claim_count = 0
        for hypothesis in hypotheses:
            local_support = [
                catalog[evidence_id]
                for evidence_id in hypothesis.supporting_evidence_ids
                if evidence_id in catalog
            ]
            if not local_support or not any(
                item.component == hypothesis.component for item in local_support
            ):
                unsupported_claim_count += 1

        latest_records = _latest_tool_records(run)
        tool_failures = sum(
            item.status == ToolExecutionStatus.ERROR for item in latest_records
        )
        leakage = run.output_leakage_detected or _contains_leak(
            run.model_dump(mode='json')
        )
        false_diagnosis = bool(
            not truth.expected_should_diagnose
            and (diagnosed or run.diagnosis_attempted)
        )
        incident_match = run.incident_present == truth.expected_incident
        if truth.expected_should_diagnose:
            diagnosis_match = diagnosed
        elif truth.expected_incident:
            diagnosis_match = bool(
                diagnosis is not None
                and diagnosis.status != DiagnosisStatus.DIAGNOSED
            )
        else:
            diagnosis_match = bool(
                diagnosis is not None
                and diagnosis.status == DiagnosisStatus.NO_DIAGNOSIS
            )
        minimum_evidence_satisfied = not missing_types

        applicable_scores = [
            value
            for value in (
                component_correct,
                fault_correct,
                top_k_covered,
            )
            if value is not None
        ]
        passed = all([
            incident_match,
            diagnosis_match,
            evidence_references_valid,
            minimum_evidence_satisfied,
            unsupported_claim_count == 0,
            not leakage,
            not false_diagnosis,
            *applicable_scores,
        ])
        return BenchmarkCaseResult(
            case_id=run.case_id,
            run_status=run.status.value,
            route=run.route,
            passed=passed,
            expected_incident_match=incident_match,
            diagnosis_expectation_match=diagnosis_match,
            primary_component_correct=component_correct,
            fault_type_correct=fault_correct,
            top_k_truth_covered=top_k_covered,
            actual_primary_component=primary.component if primary else None,
            actual_primary_fault_type=primary.cause if primary else None,
            evidence_references_valid=evidence_references_valid,
            valid_evidence_reference_count=len(valid_ids),
            invalid_evidence_reference_count=len(invalid_ids),
            minimum_required_evidence_satisfied=minimum_evidence_satisfied,
            missing_required_evidence_types=missing_types,
            unsupported_claim_count=unsupported_claim_count,
            tool_call_count=len(latest_records),
            tool_failure_count=tool_failures,
            model_request_count=run.model_request_count,
            latency_ms=run.latency_ms,
            ground_truth_leakage=leakage,
            false_diagnosis=false_diagnosis,
        )


__all__ = ['BenchmarkScorer']
