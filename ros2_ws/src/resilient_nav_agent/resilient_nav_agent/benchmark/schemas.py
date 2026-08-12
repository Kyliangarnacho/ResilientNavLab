"""Ground-truth schema isolated from the Agent input channel."""

from __future__ import annotations

from pydantic import Field, JsonValue, model_validator, StrictBool

from resilient_nav_agent.schemas import (
    ComponentIdentifier,
    DomainModel,
    EvidenceType,
    Identifier,
    SummaryText,
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
