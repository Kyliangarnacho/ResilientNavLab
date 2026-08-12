"""Tests for strict Robot Domain schemas."""

import math

from pydantic import ValidationError
import pytest

from resilient_nav_agent.schemas import (
    DiagnosisHypothesis,
    DiagnosisResult,
    DiagnosisStatus,
    EvidenceItem,
    HealthObservation,
    HealthState,
    SupportLevel,
)


def make_health(**overrides):
    """Return one valid canonical health observation."""
    values = {
        'component': 'imu',
        'state': HealthState.FAULT,
        'health_score': 0.0,
        'detector_confidence': 0.9,
        'detected_fault_hint': 'bias',
        'reasons': ['yaw_rate_residual_exceeded'],
        'metrics': {'message_age_sec': 0.02},
        'window_start_sec': 10.0,
        'window_end_sec': 12.0,
        'sample_count': 20,
        'observed_at_sec': 12.1,
    }
    values.update(overrides)
    return HealthObservation(**values)


def test_health_observation_forbids_extra_fields():
    """Undeclared cross-boundary fields must be rejected."""
    with pytest.raises(ValidationError):
        make_health(scenario_id='leak')


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_health_observation_rejects_non_finite_metrics(value):
    """Non-finite numbers must not enter the JSON contract."""
    with pytest.raises(ValidationError):
        make_health(metrics={'message_age_sec': value})


def test_health_observation_rejects_string_score_coercion():
    """Numeric contract fields must not accept numeric strings."""
    with pytest.raises(ValidationError):
        make_health(health_score='0.5')


def test_evidence_type_does_not_include_ground_truth():
    """Benchmark-only evidence types are outside the Agent schema."""
    with pytest.raises(ValidationError):
        EvidenceItem(
            evidence_id='ev-1',
            evidence_type='ground_truth',
            component='imu',
            source='health-monitor:imu',
            start_sec=1.0,
            end_sec=2.0,
            summary='Prohibited evidence type.',
            structured_data={},
        )


def test_diagnosis_primary_hypothesis_must_exist():
    """A primary hypothesis ID must resolve inside the result."""
    hypothesis = DiagnosisHypothesis(
        hypothesis_id='hyp-1',
        component='imu',
        cause='possible yaw-rate bias',
        support_level=SupportLevel.MEDIUM,
        supporting_evidence_ids=['ev-1'],
        contradicting_evidence_ids=[],
        rationale='The deterministic monitor reported a persistent residual.',
    )
    with pytest.raises(ValidationError):
        DiagnosisResult(
            incident_id='incident-1',
            status=DiagnosisStatus.DIAGNOSED,
            primary_hypothesis_id='hyp-missing',
            hypotheses=[hypothesis],
            missing_evidence=[],
            recommended_checks=['Compare a later sanitized health window.'],
            summary='A candidate diagnosis exists.',
        )


def test_valid_diagnosis_uses_qualitative_support():
    """A valid result carries qualitative rather than fake probabilities."""
    hypothesis = DiagnosisHypothesis(
        hypothesis_id='hyp-1',
        component='imu',
        cause='possible yaw-rate bias',
        support_level=SupportLevel.HIGH,
        supporting_evidence_ids=['ev-1'],
        contradicting_evidence_ids=[],
        rationale='Two sanitized health windows show the same detector hint.',
    )
    result = DiagnosisResult(
        incident_id='incident-1',
        status=DiagnosisStatus.DIAGNOSED,
        primary_hypothesis_id='hyp-1',
        hypotheses=[hypothesis],
        missing_evidence=[],
        recommended_checks=['Inspect another read-only residual window.'],
        summary='The available evidence supports one candidate cause.',
    )

    assert result.primary_hypothesis_id == 'hyp-1'
    assert result.hypotheses[0].support_level == SupportLevel.HIGH
