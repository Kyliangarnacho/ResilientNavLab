"""Strict canonical schemas at the Robot Agent trust boundary."""

from __future__ import annotations

from enum import Enum
from math import isfinite
from typing import Annotated, Any

from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    field_validator,
    JsonValue,
    model_validator,
    StrictFloat,
    StrictInt,
    StrictStr,
    StringConstraints,
)


Identifier = Annotated[
    StrictStr,
    StringConstraints(
        strip_whitespace=True,
        min_length=1,
        max_length=128,
        pattern=r'^[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}$',
    ),
]
ComponentIdentifier = Annotated[
    StrictStr,
    StringConstraints(
        strip_whitespace=True,
        min_length=1,
        max_length=64,
        pattern=r'^[a-z][a-z0-9_.:-]{0,63}$',
    ),
]
MetricName = Annotated[
    StrictStr,
    StringConstraints(
        strip_whitespace=True,
        min_length=1,
        max_length=64,
        pattern=r'^[A-Za-z][A-Za-z0-9_.:-]{0,63}$',
    ),
]
ShortText = Annotated[
    StrictStr,
    StringConstraints(strip_whitespace=True, min_length=1, max_length=256),
]
SummaryText = Annotated[
    StrictStr,
    StringConstraints(strip_whitespace=True, min_length=1, max_length=1000),
]
FiniteFloat = Annotated[StrictFloat, Field(allow_inf_nan=False)]
NonNegativeFloat = Annotated[
    StrictFloat,
    Field(ge=0.0, allow_inf_nan=False),
]
UnitFloat = Annotated[
    StrictFloat,
    Field(ge=0.0, le=1.0, allow_inf_nan=False),
]


class HealthState(str, Enum):
    """Canonical state produced by deterministic Health Monitors."""

    UNKNOWN = 'UNKNOWN'
    HEALTHY = 'HEALTHY'
    DEGRADED = 'DEGRADED'
    FAULT = 'FAULT'


class EvidenceType(str, Enum):
    """Evidence kinds allowed in the first offline diagnostic contract."""

    HEALTH = 'health'
    SENSOR_METRIC = 'sensor_metric'
    MOTION = 'motion'
    CAMERA_HEALTH = 'camera_health'
    COMPARISON = 'comparison'
    PROBE_RESULT = 'probe_result'


class IncidentMode(str, Enum):
    """Execution mode supported by the current incident contract."""

    OFFLINE = 'offline'


class OperationalSeverity(str, Enum):
    """Operational impact inferred only from Agent-visible health state."""

    INFO = 'info'
    WARNING = 'warning'
    FAULT = 'fault'
    CRITICAL = 'critical'


class SupportLevel(str, Enum):
    """Qualitative support for a diagnosis hypothesis."""

    LOW = 'low'
    MEDIUM = 'medium'
    HIGH = 'high'


class DiagnosisStatus(str, Enum):
    """Completion state for one offline diagnosis."""

    DIAGNOSED = 'diagnosed'
    INSUFFICIENT_EVIDENCE = 'insufficient_evidence'
    BLOCKED = 'blocked'


class DomainModel(BaseModel):
    """Shared immutable base that rejects undeclared fields."""

    model_config = ConfigDict(
        extra='forbid',
        frozen=True,
        allow_inf_nan=False,
    )


class HealthObservation(DomainModel):
    """One Sanitizer-approved Health Monitor observation visible to Agent."""

    component: ComponentIdentifier
    state: HealthState
    health_score: UnitFloat | None = None
    detector_confidence: UnitFloat | None = None
    detected_fault_hint: ShortText | None = None
    reasons: list[ShortText] = Field(default_factory=list, max_length=32)
    metrics: dict[MetricName, FiniteFloat] = Field(
        default_factory=dict,
        max_length=64,
    )
    window_start_sec: NonNegativeFloat
    window_end_sec: NonNegativeFloat
    sample_count: Annotated[StrictInt, Field(ge=0, le=10_000_000)]
    observed_at_sec: NonNegativeFloat

    @model_validator(mode='after')
    def validate_time_window(self):
        """Reject inverted observation windows."""
        if self.window_end_sec < self.window_start_sec:
            raise ValueError('window_end_sec must not precede window_start_sec')
        return self


def _assert_safe_json(value: Any, depth: int = 0) -> None:
    """Reject unsafe or non-finite values inside structured evidence data."""
    if depth > 8:
        raise ValueError('structured_data nesting is too deep')
    if isinstance(value, dict):
        if len(value) > 128:
            raise ValueError('structured_data mapping is too large')
        for key, item in value.items():
            if not isinstance(key, str) or not key or len(key) > 128:
                raise ValueError('structured_data has an invalid key')
            _assert_safe_json(item, depth + 1)
        return
    if isinstance(value, list):
        if len(value) > 256:
            raise ValueError('structured_data list is too large')
        for item in value:
            _assert_safe_json(item, depth + 1)
        return
    if isinstance(value, float) and not isfinite(value):
        raise ValueError('structured_data contains a non-finite number')
    if isinstance(value, str):
        lowered = value.lstrip().lower()
        if lowered.startswith('data:') or 'base64,' in lowered:
            raise ValueError('structured_data contains encoded payload data')
        if len(value) > 2000:
            raise ValueError('structured_data string is too large')


class EvidenceItem(DomainModel):
    """A finite, structured observation referenced by an incident."""

    evidence_id: Identifier
    evidence_type: EvidenceType
    component: ComponentIdentifier
    source: Identifier
    start_sec: NonNegativeFloat
    end_sec: NonNegativeFloat
    summary: SummaryText
    structured_data: dict[str, JsonValue] = Field(
        default_factory=dict,
        max_length=128,
    )

    @field_validator('structured_data')
    @classmethod
    def validate_structured_data(cls, value):
        """Enforce finite JSON without encoded binary payloads."""
        _assert_safe_json(value)
        return value

    @model_validator(mode='after')
    def validate_time_window(self):
        """Reject inverted evidence windows."""
        if self.end_sec < self.start_sec:
            raise ValueError('end_sec must not precede start_sec')
        return self


class RobotIncident(DomainModel):
    """One bounded offline diagnostic case assembled from sanitized data."""

    incident_id: Identifier
    mode: IncidentMode
    trigger_component: ComponentIdentifier
    trigger_state: HealthState
    trigger_fault_hint: ShortText | None = None
    trigger_reasons: list[ShortText] = Field(default_factory=list, max_length=32)
    trigger_health: HealthObservation
    related_health: list[HealthObservation] = Field(
        default_factory=list,
        max_length=64,
    )
    evidence_ids: list[Identifier] = Field(default_factory=list, max_length=128)
    incident_start_sec: NonNegativeFloat
    latest_observation_sec: NonNegativeFloat
    operational_severity: OperationalSeverity

    @model_validator(mode='after')
    def validate_consistency(self):
        """Keep duplicated trigger fields consistent and references unique."""
        if self.trigger_state not in (HealthState.DEGRADED, HealthState.FAULT):
            raise ValueError('trigger state is not diagnosable in offline mode')
        if self.trigger_component != self.trigger_health.component:
            raise ValueError('trigger component does not match trigger health')
        if self.trigger_state != self.trigger_health.state:
            raise ValueError('trigger state does not match trigger health')
        if self.trigger_fault_hint != self.trigger_health.detected_fault_hint:
            raise ValueError('trigger fault hint does not match trigger health')
        if self.trigger_reasons != self.trigger_health.reasons:
            raise ValueError('trigger reasons do not match trigger health')
        if self.latest_observation_sec < self.incident_start_sec:
            raise ValueError('latest observation precedes incident start')
        if len(self.evidence_ids) != len(set(self.evidence_ids)):
            raise ValueError('evidence_ids must be unique')
        return self


class DiagnosisHypothesis(DomainModel):
    """One qualitative, evidence-linked candidate diagnosis."""

    hypothesis_id: Identifier
    component: ComponentIdentifier
    cause: ShortText
    support_level: SupportLevel
    supporting_evidence_ids: list[Identifier] = Field(
        default_factory=list,
        max_length=128,
    )
    contradicting_evidence_ids: list[Identifier] = Field(
        default_factory=list,
        max_length=128,
    )
    rationale: SummaryText

    @model_validator(mode='after')
    def validate_evidence_references(self):
        """Reject duplicate or simultaneously supporting/contradicting IDs."""
        supporting = self.supporting_evidence_ids
        contradicting = self.contradicting_evidence_ids
        if len(supporting) != len(set(supporting)):
            raise ValueError('supporting evidence IDs must be unique')
        if len(contradicting) != len(set(contradicting)):
            raise ValueError('contradicting evidence IDs must be unique')
        if set(supporting).intersection(contradicting):
            raise ValueError('evidence cannot both support and contradict')
        return self


class DiagnosisResult(DomainModel):
    """Auditable diagnosis output without fabricated numeric probabilities."""

    incident_id: Identifier
    status: DiagnosisStatus
    primary_hypothesis_id: Identifier | None = None
    hypotheses: list[DiagnosisHypothesis] = Field(
        default_factory=list,
        max_length=32,
    )
    missing_evidence: list[ShortText] = Field(default_factory=list, max_length=64)
    recommended_checks: list[ShortText] = Field(
        default_factory=list,
        max_length=64,
    )
    summary: SummaryText

    @model_validator(mode='after')
    def validate_primary_hypothesis(self):
        """Require a diagnosed result to reference an existing hypothesis."""
        hypothesis_ids = [item.hypothesis_id for item in self.hypotheses]
        if len(hypothesis_ids) != len(set(hypothesis_ids)):
            raise ValueError('hypothesis IDs must be unique')
        if (
            self.primary_hypothesis_id is not None
            and self.primary_hypothesis_id not in hypothesis_ids
        ):
            raise ValueError('primary_hypothesis_id does not reference hypotheses')
        if (
            self.status == DiagnosisStatus.DIAGNOSED
            and self.primary_hypothesis_id is None
        ):
            raise ValueError('diagnosed result requires a primary hypothesis')
        return self
