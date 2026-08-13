"""Strict service boundary for one offline Robot diagnosis run."""

from __future__ import annotations

from collections.abc import Callable
from enum import Enum
import json
from time import monotonic
from typing import Any

from agent_core import AgentRuntime
from agent_core.observability import RunStatus, RunTrace
from agent_core.tools import ToolExecutionRecord
from pydantic import Field, model_validator, StrictBool, StrictInt, ValidationError

from resilient_nav_agent.extension import RobotDomainExtension
from resilient_nav_agent.offline.context import OfflineDiagnosisContext
from resilient_nav_agent.offline.schemas import OfflineAgentInput
from resilient_nav_agent.sanitizer import AgentInputSanitizer, SanitizationError
from resilient_nav_agent.schemas import (
    ComponentIdentifier,
    DiagnosisResult,
    DiagnosisStatus,
    DomainModel,
    EvidenceType,
    Identifier,
)


class OfflineDiagnosisRunStatus(str, Enum):
    """Robot-owned outcome after Core execution and strict output parsing."""

    COMPLETED = 'completed'
    NO_DIAGNOSIS = 'no_diagnosis'
    INSUFFICIENT_EVIDENCE = 'insufficient_evidence'
    BLOCKED = 'blocked'
    STRUCTURED_FAILURE = 'structured_failure'


class StructuredFailureCode(str, Enum):
    """Safe bounded reasons why a strict final result was unavailable."""

    INVALID_JSON = 'invalid_json'
    INVALID_SCHEMA = 'invalid_schema'
    CONTRACT_MISMATCH = 'contract_mismatch'
    PROHIBITED_OUTPUT = 'prohibited_output'
    RUNTIME_ERROR = 'runtime_error'


class RunEvidenceReference(DomainModel):
    """Truth-free evidence catalog retained for deterministic scoring."""

    evidence_id: Identifier
    evidence_type: EvidenceType
    component: ComponentIdentifier


class OfflineDiagnosisRun(DomainModel):
    """Auditable truth-free result of one offline Robot diagnosis execution."""

    case_id: Identifier
    incident_present: StrictBool
    route: str = Field(min_length=1, max_length=64)
    status: OfflineDiagnosisRunStatus
    diagnosis_result: DiagnosisResult | None = None
    evidence_catalog: tuple[RunEvidenceReference, ...] = Field(max_length=128)
    tool_records: tuple[ToolExecutionRecord, ...] = Field(
        default_factory=tuple,
        max_length=32,
    )
    trace: RunTrace | None = None
    model_request_count: StrictInt = Field(ge=0)
    latency_ms: float = Field(ge=0.0, allow_inf_nan=False)
    structured_failure_code: StructuredFailureCode | None = None
    output_leakage_detected: StrictBool = False
    diagnosis_attempted: StrictBool = False

    @model_validator(mode='after')
    def validate_outcome(self):
        """Keep successful and failed run payloads mutually exclusive."""
        if self.status == OfflineDiagnosisRunStatus.STRUCTURED_FAILURE:
            if self.structured_failure_code is None or self.diagnosis_result:
                raise ValueError('structured failure requires only a failure code')
        elif self.structured_failure_code is not None:
            raise ValueError('non-failure run cannot contain a failure code')
        elif self.status == OfflineDiagnosisRunStatus.BLOCKED:
            if (
                self.diagnosis_result is not None
                and self.diagnosis_result.status != DiagnosisStatus.BLOCKED
            ):
                raise ValueError('blocked run has an incompatible result')
        elif self.diagnosis_result is None:
            raise ValueError('completed run requires a strict diagnosis result')
        return self


def _catalog(agent_input: OfflineAgentInput) -> tuple[RunEvidenceReference, ...]:
    return tuple(
        RunEvidenceReference(
            evidence_id=item.evidence_id,
            evidence_type=item.evidence_type,
            component=item.component,
        )
        for item in agent_input.evidence
    )


def _status_for_result(result: DiagnosisResult) -> OfflineDiagnosisRunStatus:
    return {
        DiagnosisStatus.DIAGNOSED: OfflineDiagnosisRunStatus.COMPLETED,
        DiagnosisStatus.NO_DIAGNOSIS: OfflineDiagnosisRunStatus.NO_DIAGNOSIS,
        DiagnosisStatus.INSUFFICIENT_EVIDENCE: (
            OfflineDiagnosisRunStatus.INSUFFICIENT_EVIDENCE
        ),
        DiagnosisStatus.BLOCKED: OfflineDiagnosisRunStatus.BLOCKED,
    }[result.status]


def _run_failure(
    *,
    agent_input: OfflineAgentInput,
    route: str,
    code: StructuredFailureCode,
    elapsed_ms: float,
    tool_records: tuple[ToolExecutionRecord, ...] = (),
    trace: RunTrace | None = None,
    output_leakage_detected: bool = False,
    diagnosis_attempted: bool = False,
) -> OfflineDiagnosisRun:
    return OfflineDiagnosisRun(
        case_id=agent_input.case_id,
        incident_present=agent_input.incident is not None,
        route=route,
        status=OfflineDiagnosisRunStatus.STRUCTURED_FAILURE,
        diagnosis_result=None,
        evidence_catalog=_catalog(agent_input),
        tool_records=tool_records,
        trace=trace,
        model_request_count=trace.total_model_requests if trace else 0,
        latency_ms=elapsed_ms,
        structured_failure_code=code,
        output_leakage_detected=output_leakage_detected,
        diagnosis_attempted=diagnosis_attempted,
    )


def _diagnosis_attempted(payload: Any) -> bool:
    return isinstance(payload, dict) and payload.get('status') == 'diagnosed'


def _validate_result_contract(
    result: DiagnosisResult,
    agent_input: OfflineAgentInput,
) -> None:
    incident = agent_input.incident
    if incident is None:
        if (
            result.status != DiagnosisStatus.NO_DIAGNOSIS
            or result.incident_id is not None
        ):
            raise ValueError('healthy input cannot return a diagnosis')
        return
    if result.status == DiagnosisStatus.NO_DIAGNOSIS:
        raise ValueError('diagnosable input cannot return no_diagnosis')
    if result.incident_id != incident.incident_id:
        raise ValueError('result incident ID does not match Agent input')


def run_offline_diagnosis(
    agent_input: OfflineAgentInput,
    completion: Callable[..., Any],
    *,
    request_options: dict[str, Any] | None = None,
) -> OfflineDiagnosisRun:
    """Run Core against sanitized input and strictly parse DiagnosisResult."""
    if not isinstance(agent_input, OfflineAgentInput):
        raise TypeError('run_offline_diagnosis requires OfflineAgentInput')
    safe_input = agent_input.model_copy(deep=True)
    sanitizer = AgentInputSanitizer()
    sanitizer.assert_safe_payload(safe_input.model_dump(mode='json'))
    tool_context = (
        OfflineDiagnosisContext.from_agent_input(safe_input)
        if safe_input.incident is not None
        else None
    )
    extension = RobotDomainExtension(safe_input, tool_context)
    started = monotonic()
    try:
        core_result = AgentRuntime(
            extension,
            completion,
            request_options=request_options,
        ).run(
            f'offline:{safe_input.case_id}',
            'Assess the current sanitized offline Robot health input.',
        )
    except Exception:
        return _run_failure(
            agent_input=safe_input,
            route='runtime_error',
            code=StructuredFailureCode.RUNTIME_ERROR,
            elapsed_ms=(monotonic() - started) * 1000.0,
        )

    trace = core_result.trace
    elapsed_ms = trace.total_duration_ms
    records = tuple(core_result.tool_records)
    if core_result.trace.status == RunStatus.BLOCKED:
        return OfflineDiagnosisRun(
            case_id=safe_input.case_id,
            incident_present=safe_input.incident is not None,
            route=core_result.route.route,
            status=OfflineDiagnosisRunStatus.BLOCKED,
            diagnosis_result=None,
            evidence_catalog=_catalog(safe_input),
            tool_records=records,
            trace=trace,
            model_request_count=trace.total_model_requests,
            latency_ms=elapsed_ms,
        )

    try:
        sanitizer.assert_safe_payload(core_result.answer)
        sanitizer.assert_safe_payload(core_result.analysis)
        sanitizer.assert_safe_payload(
            core_result.context.model_dump(mode='json')
        )
        sanitizer.assert_safe_payload([
            item.model_dump(mode='json') for item in records
        ])
        sanitizer.assert_safe_payload(trace.model_dump(mode='json'))
    except SanitizationError:
        return _run_failure(
            agent_input=safe_input,
            route=core_result.route.route,
            code=StructuredFailureCode.PROHIBITED_OUTPUT,
            elapsed_ms=elapsed_ms,
            tool_records=records,
            trace=trace,
            output_leakage_detected=True,
        )

    try:
        payload = json.loads(core_result.answer)
    except (json.JSONDecodeError, TypeError):
        return _run_failure(
            agent_input=safe_input,
            route=core_result.route.route,
            code=StructuredFailureCode.INVALID_JSON,
            elapsed_ms=elapsed_ms,
            tool_records=records,
            trace=trace,
        )
    attempted = _diagnosis_attempted(payload)
    if not isinstance(payload, dict):
        return _run_failure(
            agent_input=safe_input,
            route=core_result.route.route,
            code=StructuredFailureCode.INVALID_SCHEMA,
            elapsed_ms=elapsed_ms,
            tool_records=records,
            trace=trace,
            diagnosis_attempted=attempted,
        )
    try:
        diagnosis = DiagnosisResult.model_validate(payload)
    except ValidationError:
        return _run_failure(
            agent_input=safe_input,
            route=core_result.route.route,
            code=StructuredFailureCode.INVALID_SCHEMA,
            elapsed_ms=elapsed_ms,
            tool_records=records,
            trace=trace,
            diagnosis_attempted=attempted,
        )
    try:
        _validate_result_contract(diagnosis, safe_input)
    except ValueError:
        return _run_failure(
            agent_input=safe_input,
            route=core_result.route.route,
            code=StructuredFailureCode.CONTRACT_MISMATCH,
            elapsed_ms=elapsed_ms,
            tool_records=records,
            trace=trace,
            diagnosis_attempted=attempted,
        )
    return OfflineDiagnosisRun(
        case_id=safe_input.case_id,
        incident_present=safe_input.incident is not None,
        route=core_result.route.route,
        status=_status_for_result(diagnosis),
        diagnosis_result=diagnosis,
        evidence_catalog=_catalog(safe_input),
        tool_records=records,
        trace=trace,
        model_request_count=trace.total_model_requests,
        latency_ms=elapsed_ms,
        diagnosis_attempted=attempted,
    )


__all__ = [
    'OfflineDiagnosisRun',
    'OfflineDiagnosisRunStatus',
    'StructuredFailureCode',
    'run_offline_diagnosis',
]
