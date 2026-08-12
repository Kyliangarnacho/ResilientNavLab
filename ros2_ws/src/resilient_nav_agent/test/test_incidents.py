"""Tests for deterministic offline Incident and Evidence builders."""

import pytest

from resilient_nav_agent.evidence import build_health_evidence
from resilient_nav_agent.incidents import build_offline_incident, IncidentBuildError
from resilient_nav_agent.schemas import (
    HealthObservation,
    HealthState,
    OperationalSeverity,
)


def make_health(component='imu', state=HealthState.FAULT, observed_at=12.0):
    """Create a valid HealthObservation for builder tests."""
    return HealthObservation(
        component=component,
        state=state,
        health_score=0.0 if state == HealthState.FAULT else 0.5,
        detector_confidence=0.9,
        detected_fault_hint='bias' if component == 'imu' else 'freeze',
        reasons=['deterministic_rule_triggered'],
        metrics={'message_age_sec': 0.02},
        window_start_sec=10.0,
        window_end_sec=11.5,
        sample_count=20,
        observed_at_sec=observed_at,
    )


@pytest.mark.parametrize('state', [HealthState.HEALTHY, HealthState.UNKNOWN])
def test_non_fault_health_does_not_create_incident(state):
    """HEALTHY and UNKNOWN are not fault-diagnosis triggers in RA-1A."""
    with pytest.raises(IncidentBuildError):
        build_offline_incident(make_health(state=state))


def test_degraded_health_creates_warning_incident():
    """DEGRADED maps deterministically to warning severity."""
    trigger = make_health(state=HealthState.DEGRADED)
    incident = build_offline_incident(
        trigger,
        evidence_ids=['ev-imu-1'],
        incident_id='incident-imu-1',
    )

    assert incident.operational_severity == OperationalSeverity.WARNING
    assert incident.trigger_health == trigger


def test_fault_health_creates_fault_incident_with_latest_related_time():
    """FAULT maps to fault severity and includes related health timing."""
    trigger = make_health()
    related = make_health(component='wheel', observed_at=13.0)
    incident = build_offline_incident(
        trigger,
        [related],
        ['ev-imu-1', 'ev-wheel-1'],
        incident_id='incident-imu-2',
    )

    assert incident.operational_severity == OperationalSeverity.FAULT
    assert incident.latest_observation_sec == 13.0
    assert incident.evidence_ids == ['ev-imu-1', 'ev-wheel-1']


def test_health_evidence_has_canonical_source_only():
    """Evidence derives from canonical Health without transport identifiers."""
    evidence = build_health_evidence(make_health(), 'ev-imu-1')
    serialized = evidence.model_dump_json()

    assert evidence.source == 'health-monitor:imu'
    assert '/faulted/' not in serialized
    assert 'source_topic' not in serialized
