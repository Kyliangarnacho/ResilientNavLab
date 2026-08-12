"""Evidence builders for the pure offline Robot Domain."""

from resilient_nav_agent.schemas import EvidenceItem, EvidenceType, HealthObservation


def build_health_evidence(
    observation: HealthObservation,
    evidence_id: str,
) -> EvidenceItem:
    """Create evidence without preserving transport or experiment topic names."""
    hint = observation.detected_fault_hint or 'none'
    summary = (
        f'{observation.component} health state {observation.state.value}; '
        f'detector hint {hint}.'
    )
    evidence_type = (
        EvidenceType.CAMERA_HEALTH
        if observation.component == 'camera'
        else EvidenceType.HEALTH
    )
    return EvidenceItem(
        evidence_id=evidence_id,
        evidence_type=evidence_type,
        component=observation.component,
        source=f'health-monitor:{observation.component}',
        start_sec=observation.window_start_sec,
        end_sec=observation.window_end_sec,
        summary=summary,
        structured_data=observation.model_dump(mode='json'),
    )
