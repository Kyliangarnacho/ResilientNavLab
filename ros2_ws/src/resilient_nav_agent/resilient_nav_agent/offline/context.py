"""In-memory, read-only dependency object for RA-1A offline Tools."""

from __future__ import annotations

from pydantic import Field, model_validator

from resilient_nav_agent.offline.schemas import OfflineAgentInput
from resilient_nav_agent.schemas import (
    DomainModel,
    EvidenceItem,
    HealthObservation,
    RobotIncident,
)


class OfflineDiagnosisContext(DomainModel):
    """Only sanitized observations accessible to the Robot Tool handlers."""

    case_id: str
    incident: RobotIncident
    evidence: tuple[EvidenceItem, ...] = Field(max_length=128)
    health_observations: tuple[HealthObservation, ...] = Field(max_length=64)

    @model_validator(mode='after')
    def validate_context(self):
        """Require Tool-visible references to resolve locally."""
        evidence_ids = {item.evidence_id for item in self.evidence}
        if not set(self.incident.evidence_ids).issubset(evidence_ids):
            raise ValueError('Tool context is missing incident evidence')
        return self

    @classmethod
    def from_agent_input(cls, agent_input: OfflineAgentInput):
        """Build context without any BenchmarkTruth parameter or field."""
        if agent_input.incident is None:
            raise ValueError('healthy Agent input has no diagnosis context')
        return cls(
            case_id=agent_input.case_id,
            incident=agent_input.incident.model_copy(deep=True),
            evidence=tuple(item.model_copy(deep=True) for item in agent_input.evidence),
            health_observations=tuple(
                item.model_copy(deep=True)
                for item in agent_input.health_observations
            ),
        )

    def evidence_by_id(self, evidence_id: str) -> EvidenceItem:
        """Resolve one evidence ID or fail with a safe bounded error."""
        for item in self.evidence:
            if item.evidence_id == evidence_id:
                return item
        raise ValueError('requested evidence is unavailable')

    def health_by_component(self, component: str) -> HealthObservation:
        """Resolve one canonical component or fail safely."""
        for item in self.health_observations:
            if item.component == component:
                return item
        raise ValueError('requested component health is unavailable')
