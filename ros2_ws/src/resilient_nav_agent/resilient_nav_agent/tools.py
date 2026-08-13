"""Deterministic read-only Tools over one sanitized offline context."""

from __future__ import annotations

from agent_core.tools import ToolRegistry
from pydantic import Field, model_validator

from resilient_nav_agent.offline.context import OfflineDiagnosisContext
from resilient_nav_agent.schemas import (
    ComponentIdentifier,
    DomainModel,
    EvidenceType,
    FiniteFloat,
    HealthState,
    Identifier,
    MetricName,
    NonNegativeFloat,
    ShortText,
    UnitFloat,
)


class IncidentSnapshotArguments(DomainModel):
    """No-argument contract for the incident snapshot Tool."""


class CompareComponentHealthArguments(DomainModel):
    """Select two different sanitized components for comparison."""

    left_component: ComponentIdentifier
    right_component: ComponentIdentifier

    @model_validator(mode='after')
    def validate_distinct_components(self):
        """Reject a comparison of a component with itself."""
        if self.left_component == self.right_component:
            raise ValueError('comparison components must be different')
        return self


class InspectMetricWindowArguments(DomainModel):
    """Select one metric from one sanitized component window."""

    component: ComponentIdentifier
    metric_name: MetricName


class EvidenceReference(DomainModel):
    """Bounded reference to Agent-visible evidence."""

    evidence_id: Identifier
    evidence_type: EvidenceType
    component: ComponentIdentifier


class HealthSnapshot(DomainModel):
    """Structured copy of one canonical Health observation."""

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
    sample_count: int = Field(ge=0, le=10_000_000)
    observed_at_sec: NonNegativeFloat
    evidence_ids: list[Identifier] = Field(default_factory=list, max_length=128)


class IncidentHealthSnapshot(DomainModel):
    """Complete bounded health snapshot for the current incident."""

    case_id: Identifier
    incident_id: Identifier
    trigger_component: ComponentIdentifier
    trigger_state: HealthState
    observations: list[HealthSnapshot] = Field(max_length=64)
    evidence: list[EvidenceReference] = Field(max_length=128)


class SharedMetricComparison(DomainModel):
    """Deterministic difference for one metric shared by two components."""

    metric_name: MetricName
    left_value: FiniteFloat
    right_value: FiniteFloat
    left_minus_right: FiniteFloat


class ComponentHealthComparison(DomainModel):
    """Side-by-side state and metric comparison without causal diagnosis."""

    left: HealthSnapshot
    right: HealthSnapshot
    health_score_delta: FiniteFloat | None = None
    shared_metrics: list[SharedMetricComparison] = Field(
        default_factory=list,
        max_length=64,
    )


class MetricWindowInspection(DomainModel):
    """One metric value and its bounded observation provenance."""

    component: ComponentIdentifier
    metric_name: MetricName
    value: FiniteFloat
    state: HealthState
    window_start_sec: NonNegativeFloat
    window_end_sec: NonNegativeFloat
    sample_count: int = Field(ge=0, le=10_000_000)
    observed_at_sec: NonNegativeFloat
    evidence_ids: list[Identifier] = Field(default_factory=list, max_length=128)


def _evidence_references(
    context: OfflineDiagnosisContext,
    component: str | None = None,
) -> list[EvidenceReference]:
    return [
        EvidenceReference(
            evidence_id=item.evidence_id,
            evidence_type=item.evidence_type,
            component=item.component,
        )
        for item in context.evidence
        if component is None or item.component == component
    ]


def _health_snapshot(
    context: OfflineDiagnosisContext,
    component: str,
) -> HealthSnapshot:
    health = context.health_by_component(component)
    return HealthSnapshot(
        **health.model_dump(mode='python'),
        evidence_ids=[
            item.evidence_id
            for item in context.evidence
            if item.component == component
        ],
    )


def create_robot_tool_registry(context: OfflineDiagnosisContext) -> ToolRegistry:
    """Register the three authorized RA-1A read-only diagnostic Tools."""
    registry = ToolRegistry()

    def get_incident_health_snapshot() -> dict:
        result = IncidentHealthSnapshot(
            case_id=context.case_id,
            incident_id=context.incident.incident_id,
            trigger_component=context.incident.trigger_component,
            trigger_state=context.incident.trigger_state,
            observations=[
                _health_snapshot(context, item.component)
                for item in context.health_observations
            ],
            evidence=_evidence_references(context),
        )
        return result.model_dump(mode='json')

    registry.register(
        name='get_incident_health_snapshot',
        description=(
            'Return the current sanitized incident health observations and '
            'their evidence references without adding external data.'
        ),
        parameters_model=IncidentSnapshotArguments,
        handler=get_incident_health_snapshot,
    )

    def compare_component_health(
        left_component: str,
        right_component: str,
    ) -> dict:
        left = _health_snapshot(context, left_component)
        right = _health_snapshot(context, right_component)
        score_delta = None
        if left.health_score is not None and right.health_score is not None:
            score_delta = left.health_score - right.health_score
        shared = sorted(set(left.metrics).intersection(right.metrics))
        result = ComponentHealthComparison(
            left=left,
            right=right,
            health_score_delta=score_delta,
            shared_metrics=[
                SharedMetricComparison(
                    metric_name=name,
                    left_value=left.metrics[name],
                    right_value=right.metrics[name],
                    left_minus_right=left.metrics[name] - right.metrics[name],
                )
                for name in shared
            ],
        )
        return result.model_dump(mode='json')

    registry.register(
        name='compare_component_health',
        description=(
            'Compare two available sanitized component health states, scores, '
            'and shared metrics without inferring a fault cause.'
        ),
        parameters_model=CompareComponentHealthArguments,
        handler=compare_component_health,
    )

    def inspect_metric_window(component: str, metric_name: str) -> dict:
        health = context.health_by_component(component)
        if metric_name not in health.metrics:
            raise ValueError('requested metric is unavailable')
        result = MetricWindowInspection(
            component=component,
            metric_name=metric_name,
            value=health.metrics[metric_name],
            state=health.state,
            window_start_sec=health.window_start_sec,
            window_end_sec=health.window_end_sec,
            sample_count=health.sample_count,
            observed_at_sec=health.observed_at_sec,
            evidence_ids=[
                item.evidence_id
                for item in context.evidence
                if item.component == component
            ],
        )
        return result.model_dump(mode='json')

    registry.register(
        name='inspect_metric_window',
        description=(
            'Return one available sanitized metric with its observation '
            'window, sample count, state, and evidence references.'
        ),
        parameters_model=InspectMetricWindowArguments,
        handler=inspect_metric_window,
    )
    return registry


__all__ = [
    'CompareComponentHealthArguments',
    'IncidentSnapshotArguments',
    'InspectMetricWindowArguments',
    'create_robot_tool_registry',
]
