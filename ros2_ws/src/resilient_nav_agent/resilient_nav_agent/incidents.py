"""Minimal deterministic construction of offline diagnosis incidents."""

from collections.abc import Iterable

from resilient_nav_agent.schemas import (
    HealthObservation,
    HealthState,
    IncidentMode,
    OperationalSeverity,
    RobotIncident,
)


class IncidentBuildError(ValueError):
    """Safe failure for inputs that must not trigger an incident."""


def build_offline_incident(
    trigger_health: HealthObservation,
    related_health: Iterable[HealthObservation] = (),
    evidence_ids: Iterable[str] = (),
    *,
    incident_id: str | None = None,
) -> RobotIncident:
    """Build an incident only for DEGRADED or FAULT trigger health."""
    if trigger_health.state == HealthState.DEGRADED:
        severity = OperationalSeverity.WARNING
    elif trigger_health.state == HealthState.FAULT:
        severity = OperationalSeverity.FAULT
    else:
        raise IncidentBuildError(
            'Only DEGRADED or FAULT health can trigger offline diagnosis.'
        )

    related = list(related_health)
    latest_observation_sec = max(
        [trigger_health.observed_at_sec]
        + [item.observed_at_sec for item in related]
    )
    generated_id = (
        f'incident-{trigger_health.component}-'
        f'{int(trigger_health.observed_at_sec * 1000)}'
    )
    return RobotIncident(
        incident_id=incident_id or generated_id,
        mode=IncidentMode.OFFLINE,
        trigger_component=trigger_health.component,
        trigger_state=trigger_health.state,
        trigger_fault_hint=trigger_health.detected_fault_hint,
        trigger_reasons=list(trigger_health.reasons),
        trigger_health=trigger_health,
        related_health=related,
        evidence_ids=list(evidence_ids),
        incident_start_sec=trigger_health.window_start_sec,
        latest_observation_sec=latest_observation_sec,
        operational_severity=severity,
    )
