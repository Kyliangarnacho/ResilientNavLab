"""Strict contracts for the offline Agent-input side of RA-1A."""

from __future__ import annotations

from pydantic import Field, model_validator

from resilient_nav_agent.benchmark.schemas import BenchmarkTruth
from resilient_nav_agent.schemas import (
    DomainModel,
    EvidenceItem,
    HealthObservation,
    HealthState,
    Identifier,
    RobotIncident,
)


class OfflineAgentInput(DomainModel):
    """The complete sanitized object an offline Agent Runtime may receive."""

    case_id: Identifier
    incident: RobotIncident | None = None
    evidence: list[EvidenceItem] = Field(default_factory=list, max_length=128)
    health_observations: list[HealthObservation] = Field(
        default_factory=list,
        max_length=64,
    )
    sanitizer_version: str = Field(default='ra1a-v1', pattern=r'^ra1a-v[0-9]+$')

    @model_validator(mode='after')
    def validate_references(self):
        """Require an internally complete input without inventing incidents."""
        evidence_ids = [item.evidence_id for item in self.evidence]
        if len(evidence_ids) != len(set(evidence_ids)):
            raise ValueError('offline evidence IDs must be unique')
        components = [item.component for item in self.health_observations]
        if len(components) != len(set(components)):
            raise ValueError('offline health components must be unique')
        diagnosable = [
            item for item in self.health_observations
            if item.state in (HealthState.DEGRADED, HealthState.FAULT)
        ]
        if self.incident is None:
            if diagnosable:
                raise ValueError('diagnosable health requires an incident')
            return self
        if self.incident.trigger_health not in self.health_observations:
            raise ValueError('incident trigger health is absent from Agent input')
        if not set(self.incident.evidence_ids).issubset(evidence_ids):
            raise ValueError('incident references absent offline evidence')
        return self


class OfflineRobotCase(DomainModel):
    """A builder-only envelope holding physically separate runtime and truth."""

    case_id: Identifier
    agent_input: OfflineAgentInput
    benchmark_truth: BenchmarkTruth

    @model_validator(mode='after')
    def validate_case_ids(self):
        """Keep both channels aligned only by their opaque case ID."""
        if self.agent_input.case_id != self.case_id:
            raise ValueError('Agent-input case ID does not match case envelope')
        if self.benchmark_truth.case_id != self.case_id:
            raise ValueError('Benchmark-truth case ID does not match case envelope')
        return self

    def agent_view(self) -> OfflineAgentInput:
        """Return only the deep-copied runtime-safe channel."""
        return self.agent_input.model_copy(deep=True)
