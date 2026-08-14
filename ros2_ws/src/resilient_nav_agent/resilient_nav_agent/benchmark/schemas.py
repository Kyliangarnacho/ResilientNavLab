"""Evaluator-only truth and deterministic offline benchmark schemas."""

from __future__ import annotations

from enum import Enum

from pydantic import (
    Field,
    JsonValue,
    model_validator,
    StrictBool,
    StrictInt,
)

from resilient_nav_agent.schemas import (
    ComponentIdentifier,
    DomainModel,
    EvidenceType,
    Identifier,
    SummaryText,
    UnitFloat,
)


class BenchmarkTruth(DomainModel):
    """Evaluator-only expected outcome; never valid as Runtime context."""

    case_id: Identifier
    expected_incident: StrictBool
    expected_component: ComponentIdentifier | None = None
    expected_fault_type: str | None = Field(
        default=None,
        min_length=1,
        max_length=128,
    )
    acceptable_fault_types: list[str] = Field(default_factory=list, max_length=32)
    acceptable_components: list[ComponentIdentifier] = Field(
        default_factory=list,
        max_length=16,
    )
    expected_should_diagnose: StrictBool
    minimum_required_evidence_types: list[EvidenceType] = Field(
        default_factory=list,
        max_length=16,
    )
    notes: SummaryText | None = None
    source_scenario: str | None = Field(
        default=None,
        min_length=1,
        max_length=128,
    )
    truth_metadata: dict[str, JsonValue] = Field(default_factory=dict, max_length=64)

    @model_validator(mode='after')
    def validate_expected_outcome(self):
        """Keep healthy and diagnostic truth contracts unambiguous."""
        if self.expected_should_diagnose:
            if not self.expected_incident:
                raise ValueError('diagnosis expectation requires an incident')
            if self.expected_component is None or self.expected_fault_type is None:
                raise ValueError('diagnosis expectation requires component and fault')
        elif (
            self.expected_component is not None
            or self.expected_fault_type is not None
        ):
            raise ValueError('healthy truth cannot require component or fault type')
        if len(self.acceptable_fault_types) != len(
            set(self.acceptable_fault_types)
        ):
            raise ValueError('acceptable fault types must be unique')
        if len(self.acceptable_components) != len(set(self.acceptable_components)):
            raise ValueError('acceptable components must be unique')
        if len(self.minimum_required_evidence_types) != len(
            set(self.minimum_required_evidence_types)
        ):
            raise ValueError('minimum evidence types must be unique')
        return self


class BenchmarkKind(str, Enum):
    """Labels that prevent Fake pipeline runs from implying model quality."""

    PIPELINE_FAKE = 'PIPELINE / FAKE BENCHMARK'
    REAL_MODEL = 'REAL MODEL BENCHMARK'


class BenchmarkCaseResult(DomainModel):
    """Deterministic score for one truth-isolated diagnosis run."""

    case_id: Identifier
    run_status: str = Field(min_length=1, max_length=64)
    route: str = Field(min_length=1, max_length=64)
    passed: StrictBool
    expected_incident_match: StrictBool
    diagnosis_expectation_match: StrictBool
    primary_component_correct: StrictBool | None = None
    fault_type_correct: StrictBool | None = None
    top_k_truth_covered: StrictBool | None = None
    actual_primary_component: ComponentIdentifier | None = None
    actual_primary_fault_type: str | None = Field(
        default=None,
        min_length=1,
        max_length=256,
    )
    evidence_references_valid: StrictBool
    valid_evidence_reference_count: StrictInt = Field(ge=0)
    invalid_evidence_reference_count: StrictInt = Field(ge=0)
    minimum_required_evidence_satisfied: StrictBool
    missing_required_evidence_types: list[EvidenceType] = Field(
        default_factory=list,
        max_length=16,
    )
    unsupported_claim_count: StrictInt = Field(ge=0)
    tool_call_count: StrictInt = Field(ge=0)
    tool_failure_count: StrictInt = Field(ge=0)
    model_request_count: StrictInt = Field(ge=0)
    latency_ms: float = Field(ge=0.0, allow_inf_nan=False)
    ground_truth_leakage: StrictBool
    false_diagnosis: StrictBool


class BenchmarkReport(DomainModel):
    """Aggregate report for one repeatable offline benchmark batch."""

    benchmark_kind: BenchmarkKind
    case_count: StrictInt = Field(ge=0)
    diagnostic_case_count: StrictInt = Field(ge=0)
    healthy_case_count: StrictInt = Field(ge=0)
    passed_case_count: StrictInt = Field(ge=0)
    component_accuracy: UnitFloat | None = None
    fault_type_accuracy: UnitFloat | None = None
    top_k_coverage: UnitFloat | None = None
    evidence_reference_validity: UnitFloat
    required_evidence_success_rate: UnitFloat
    ground_truth_leakage_count: StrictInt = Field(ge=0)
    false_diagnosis_count: StrictInt = Field(ge=0)
    total_tool_calls: StrictInt = Field(ge=0)
    total_tool_failures: StrictInt = Field(ge=0)
    total_model_requests: StrictInt = Field(ge=0)
    total_latency_ms: float = Field(ge=0.0, allow_inf_nan=False)
    status_counts: dict[str, StrictInt] = Field(default_factory=dict)
    case_results: list[BenchmarkCaseResult] = Field(default_factory=list)

    @model_validator(mode='after')
    def validate_aggregate_counts(self):
        """Keep report totals aligned with the retained case results."""
        if self.case_count != len(self.case_results):
            raise ValueError('case count does not match case results')
        if self.diagnostic_case_count + self.healthy_case_count != self.case_count:
            raise ValueError('diagnostic and healthy counts do not add up')
        if sum(self.status_counts.values()) != self.case_count:
            raise ValueError('status counts do not add up')
        return self
