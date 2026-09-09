"""Focused tests for the pure localization-quality state machine."""

import math

from localization_quality import (
    LocalizationEvidence,
    LocalizationQualityConfig,
    LocalizationQualityEvaluator,
    LocalizationState,
    ScanHealthState,
    valid_runtime_stamp,
)


def healthy_evidence(**overrides):
    """Build one complete healthy evidence snapshot."""
    values = {
        'pose_age_sec': 0.1,
        'map_to_odom_age_sec': 0.1,
        'scan_health_age_sec': 0.1,
        'position_variance': 0.05,
        'yaw_variance': 0.02,
        'scan_health_state': ScanHealthState.HEALTHY,
        'update_sequence': 1,
    }
    values.update(overrides)
    return LocalizationEvidence(**values)


def test_runtime_stamp_validation_is_independent_of_callback_order():
    """A finite positive stamp remains valid if its /clock callback lags."""
    assert valid_runtime_stamp(12.5) is True
    assert valid_runtime_stamp(1_000_000.0) is True
    assert valid_runtime_stamp(0.0) is False
    assert valid_runtime_stamp(-0.1) is False
    assert valid_runtime_stamp(math.nan) is False
    assert valid_runtime_stamp(math.inf) is False


def test_startup_without_map_frame_evidence_is_nonusable_provisional():
    """Startup without AMCL/TF evidence must not claim usability."""
    decision = LocalizationQualityEvaluator().evaluate(
        LocalizationEvidence(), elapsed_since_start_sec=0.5
    )

    assert decision.state == LocalizationState.PROVISIONAL
    assert decision.localization_usable is False
    assert 'amcl_pose_missing' in decision.reasons


def test_startup_clean_core_accepts_unknown_scan_as_usable_provisional():
    """Valid map evidence bypasses scan-health warmup rejection."""
    decision = LocalizationQualityEvaluator().evaluate(
        healthy_evidence(
            scan_health_state=ScanHealthState.UNKNOWN,
            scan_health_age_sec=0.1,
        ),
        elapsed_since_start_sec=0.5,
    )

    assert decision.state == LocalizationState.PROVISIONAL
    assert decision.localization_usable is True
    assert decision.reasons == ('scan_health_unknown',)


def test_first_complete_clean_snapshot_enters_ok_without_confirmation_window():
    """The first complete clean snapshot is immediately OK."""
    decision = LocalizationQualityEvaluator().evaluate(
        healthy_evidence(), elapsed_since_start_sec=0.2
    )

    assert decision.state == LocalizationState.OK
    assert decision.localization_usable is True


def test_scan_fault_degrades_but_does_not_claim_map_localization_is_lost():
    """Scan failure alone is degraded until map localization itself expires."""
    decision = LocalizationQualityEvaluator().evaluate(
        healthy_evidence(scan_health_state=ScanHealthState.FAULT),
        elapsed_since_start_sec=5.0,
    )

    assert decision.state == LocalizationState.DEGRADED
    assert decision.localization_usable is True
    assert decision.reasons == ('scan_health_fault',)


def test_explicit_scan_fault_is_not_hidden_by_startup_grace():
    """Startup grace only exempts absent/unknown scan evidence."""
    decision = LocalizationQualityEvaluator().evaluate(
        healthy_evidence(scan_health_state=ScanHealthState.FAULT),
        elapsed_since_start_sec=0.5,
    )

    assert decision.state == LocalizationState.DEGRADED
    assert decision.localization_usable is True


def test_stale_pose_with_fresh_tf_is_usable_but_degraded():
    """AMCL pose may be low-rate while its authoritative TF stays valid."""
    evaluator = LocalizationQualityEvaluator()

    stale = evaluator.evaluate(
        healthy_evidence(pose_age_sec=2.1), elapsed_since_start_sec=5.0
    )

    assert stale.state == LocalizationState.DEGRADED
    assert stale.localization_usable is True
    assert stale.reasons == ('amcl_pose_stale',)


def test_stale_tf_or_obvious_pose_jump_is_lost_immediately():
    """Hard direct localization evidence causes immediate LOST."""
    evaluator = LocalizationQualityEvaluator()

    stale = evaluator.evaluate(
        healthy_evidence(map_to_odom_age_sec=2.1),
        elapsed_since_start_sec=5.0,
    )
    jumped = evaluator.evaluate(
        healthy_evidence(position_jump_m=2.1), elapsed_since_start_sec=5.1
    )

    assert stale.state == LocalizationState.LOST
    assert stale.localization_usable is False
    assert jumped.state == LocalizationState.LOST
    assert jumped.localization_usable is False


def test_missing_pose_after_startup_degrades_when_tf_is_still_available():
    """Missing uncertainty evidence alone must not invalidate a fresh TF."""
    decision = LocalizationQualityEvaluator().evaluate(
        LocalizationEvidence(
            map_to_odom_age_sec=0.1,
            scan_health_age_sec=0.1,
            scan_health_state=ScanHealthState.HEALTHY,
        ),
        elapsed_since_start_sec=5.0,
    )

    assert decision.state == LocalizationState.DEGRADED
    assert decision.localization_usable is True
    assert 'amcl_pose_missing' in decision.reasons


def test_lost_recovery_uses_two_new_clean_updates_not_timer_ticks():
    """Recovery bridges through degraded and ignores duplicate snapshots."""
    evaluator = LocalizationQualityEvaluator()
    evaluator.evaluate(
        healthy_evidence(map_to_odom_age_sec=2.1, update_sequence=1),
        elapsed_since_start_sec=5.0,
    )

    first_clean = evaluator.evaluate(
        healthy_evidence(update_sequence=2), elapsed_since_start_sec=5.1
    )
    duplicate = evaluator.evaluate(
        healthy_evidence(update_sequence=2), elapsed_since_start_sec=5.2
    )
    recovered = evaluator.evaluate(
        healthy_evidence(update_sequence=3), elapsed_since_start_sec=5.3
    )

    assert first_clean.state == LocalizationState.DEGRADED
    assert first_clean.localization_usable is True
    assert first_clean.reasons == ('localization_recovery_pending',)
    assert duplicate.state == LocalizationState.DEGRADED
    assert recovered.state == LocalizationState.OK
    assert recovered.localization_usable is True


def test_recovery_confirmation_resets_on_intervening_warning():
    """Only uninterrupted clean map-to-odom updates can finish recovery."""
    evaluator = LocalizationQualityEvaluator()
    evaluator.evaluate(
        healthy_evidence(map_to_odom_age_sec=2.1, update_sequence=1),
        elapsed_since_start_sec=5.0,
    )
    evaluator.evaluate(
        healthy_evidence(update_sequence=2), elapsed_since_start_sec=5.1
    )
    warning = evaluator.evaluate(
        healthy_evidence(
            scan_health_state=ScanHealthState.DEGRADED,
            update_sequence=3,
        ),
        elapsed_since_start_sec=5.2,
    )
    first_after_warning = evaluator.evaluate(
        healthy_evidence(update_sequence=4), elapsed_since_start_sec=5.3
    )
    recovered = evaluator.evaluate(
        healthy_evidence(update_sequence=5), elapsed_since_start_sec=5.4
    )

    assert warning.state == LocalizationState.DEGRADED
    assert first_after_warning.reasons == ('localization_recovery_pending',)
    assert recovered.state == LocalizationState.OK


def test_warning_thresholds_produce_usable_degraded_state():
    """Soft evidence remains usable but explicitly degraded."""
    decision = LocalizationQualityEvaluator().evaluate(
        healthy_evidence(yaw_variance=0.4), elapsed_since_start_sec=5.0
    )

    assert decision.state == LocalizationState.DEGRADED
    assert decision.localization_usable is True
    assert decision.reasons == ('amcl_yaw_variance_high',)


def test_threshold_configuration_must_be_ordered():
    """Warning thresholds must remain below hard lost thresholds."""
    try:
        LocalizationQualityConfig(
            pose_age_warning_sec=2.0,
            pose_age_lost_sec=1.0,
        )
    except ValueError as error:
        assert 'pose age' in str(error)
    else:
        raise AssertionError('unordered thresholds were accepted')


def test_recovery_requires_at_least_two_clean_observations():
    """The short recovery bridge cannot be configured away accidentally."""
    try:
        LocalizationQualityConfig(recovery_clean_observations=1)
    except ValueError as error:
        assert 'recovery_clean_observations' in str(error)
    else:
        raise AssertionError('zero-observation recovery was accepted')


def test_recovery_clean_observation_count_must_be_an_integer():
    """Fractional update counts are invalid configuration."""
    try:
        LocalizationQualityConfig(recovery_clean_observations=1.5)
    except ValueError as error:
        assert 'recovery_clean_observations' in str(error)
    else:
        raise AssertionError('fractional recovery count was accepted')
