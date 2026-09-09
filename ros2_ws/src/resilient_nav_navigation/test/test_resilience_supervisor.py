"""Focused tests for tolerant navigation-level resilience supervision."""

from resilience_supervisor import (
    FusionState,
    LocalizationState,
    ResilienceSupervisorPolicy,
    SensorState,
    SupervisorEvidence,
    SupervisorState,
)


def healthy_evidence(**overrides):
    """Return one complete healthy runtime snapshot."""
    values = {
        'wheel_state': SensorState.HEALTHY,
        'imu_state': SensorState.HEALTHY,
        'scan_state': SensorState.HEALTHY,
        'fusion_state': FusionState.NOMINAL,
        'localization_state': LocalizationState.OK,
        'localization_usable': True,
        'accepted_measurements': ('wheel_velocity', 'imu_yaw_rate'),
        'wheel_age_sec': 0.1,
        'imu_age_sec': 0.1,
        'scan_age_sec': 0.1,
        'fusion_age_sec': 0.1,
        'localization_age_sec': 0.1,
    }
    values.update(overrides)
    return SupervisorEvidence(**values)


def test_missing_startup_evidence_waits_without_claiming_hold():
    """A cold graph is STARTING rather than a false runtime HOLD."""
    decision = ResilienceSupervisorPolicy().evaluate(
        SupervisorEvidence(), now_sec=0.5, elapsed_since_start_sec=0.5
    )

    assert decision.state == SupervisorState.STARTING
    assert decision.navigation_allowed is False


def test_healthy_chain_navigates():
    """All nominal inputs explicitly allow navigation."""
    decision = ResilienceSupervisorPolicy().evaluate(
        healthy_evidence(), now_sec=5.0, elapsed_since_start_sec=5.0
    )

    assert decision.state == SupervisorState.NAVIGATE
    assert decision.navigation_allowed is True


def test_single_sensor_fault_and_usable_degradation_do_not_stop_navigation():
    """Fallback-capable partial faults remain operationally degraded."""
    decision = ResilienceSupervisorPolicy().evaluate(
        healthy_evidence(
            imu_state=SensorState.FAULT,
            fusion_state=FusionState.DEGRADED,
            accepted_measurements=('wheel_velocity', 'wheel_yaw_rate'),
            localization_state=LocalizationState.DEGRADED,
        ),
        now_sec=5.0,
        elapsed_since_start_sec=5.0,
    )

    assert decision.state == SupervisorState.DEGRADED
    assert decision.navigation_allowed is True


def test_one_missing_sensor_monitor_is_soft_degradation_only():
    """A lost auxiliary health topic cannot stop an otherwise usable chain."""
    policy = ResilienceSupervisorPolicy()
    evidence = healthy_evidence(scan_state=None, scan_age_sec=None)

    first = policy.evaluate(
        evidence, now_sec=5.0, elapsed_since_start_sec=5.0
    )
    later = policy.evaluate(
        evidence, now_sec=6.0, elapsed_since_start_sec=6.0
    )

    assert first.state == SupervisorState.DEGRADED
    assert later.state == SupervisorState.DEGRADED
    assert later.navigation_allowed is True


def test_persistent_navigation_scan_fault_stops_after_confirmation():
    """A sustained loss of Nav2 obstacle sensing reaches HOLD, not abort."""
    policy = ResilienceSupervisorPolicy()
    evidence = healthy_evidence(scan_state=SensorState.FAULT)

    pending = policy.evaluate(
        evidence, now_sec=5.0, elapsed_since_start_sec=5.0
    )
    held = policy.evaluate(
        evidence, now_sec=5.31, elapsed_since_start_sec=5.31
    )

    assert pending.state == SupervisorState.DEGRADED
    assert pending.navigation_allowed is True
    assert held.state == SupervisorState.HOLD
    assert held.navigation_allowed is False
    assert 'navigation_scan_fault' in held.reasons


def test_scan_fault_ignores_unknown_but_allows_degraded_recovery():
    """No-data cannot reopen Nav2, while real degraded evidence can."""
    policy = ResilienceSupervisorPolicy()
    fault = healthy_evidence(scan_state=SensorState.FAULT)
    policy.evaluate(fault, now_sec=5.0, elapsed_since_start_sec=5.0)
    policy.evaluate(fault, now_sec=5.31, elapsed_since_start_sec=5.31)

    unknown = policy.evaluate(
        healthy_evidence(scan_state=SensorState.UNKNOWN),
        now_sec=5.5,
        elapsed_since_start_sec=5.5,
    )
    missing = policy.evaluate(
        healthy_evidence(scan_state=None, scan_age_sec=None),
        now_sec=5.6,
        elapsed_since_start_sec=5.6,
    )
    recovering = policy.evaluate(
        healthy_evidence(scan_state=SensorState.DEGRADED),
        now_sec=5.7,
        elapsed_since_start_sec=5.7,
    )

    assert unknown.state == SupervisorState.HOLD
    assert unknown.navigation_allowed is False
    assert 'navigation_scan_fault_latched' in unknown.reasons
    assert missing.state == SupervisorState.HOLD
    assert missing.navigation_allowed is False
    assert recovering.state == SupervisorState.DEGRADED
    assert recovering.navigation_allowed is True
    assert 'scan_health_not_healthy' in recovering.reasons


def test_unready_localization_keeps_startup_gate_closed():
    """PROVISIONAL is not operational until localization declares it usable."""
    decision = ResilienceSupervisorPolicy().evaluate(
        healthy_evidence(
            localization_state=LocalizationState.PROVISIONAL,
            localization_usable=False,
        ),
        now_sec=0.5,
        elapsed_since_start_sec=0.5,
    )

    assert decision.state == SupervisorState.STARTING
    assert decision.navigation_allowed is False


def test_one_fusion_hold_spike_is_confirmation_pending_not_a_stop():
    """A brief upstream hard spike does not cancel a navigation goal."""
    policy = ResilienceSupervisorPolicy()
    decision = policy.evaluate(
        healthy_evidence(
            fusion_state=FusionState.HOLD,
            accepted_measurements=(),
        ),
        now_sec=5.0,
        elapsed_since_start_sec=5.0,
    )

    assert decision.state == SupervisorState.DEGRADED
    assert decision.navigation_allowed is True
    assert 'hard_condition_confirmation_pending' in decision.reasons


def test_persistent_fusion_hold_stops_navigation():
    """A persistent lack of fusion measurements reaches HOLD."""
    policy = ResilienceSupervisorPolicy()
    evidence = healthy_evidence(
        fusion_state=FusionState.HOLD,
        accepted_measurements=(),
    )
    policy.evaluate(evidence, now_sec=5.0, elapsed_since_start_sec=5.0)
    decision = policy.evaluate(
        evidence, now_sec=5.31, elapsed_since_start_sec=5.31
    )

    assert decision.state == SupervisorState.HOLD
    assert decision.navigation_allowed is False


def test_localization_lost_is_an_immediate_hold():
    """An untrustworthy map pose is not persistence-filtered again."""
    decision = ResilienceSupervisorPolicy().evaluate(
        healthy_evidence(
            localization_state=LocalizationState.LOST,
            localization_usable=False,
        ),
        now_sec=5.0,
        elapsed_since_start_sec=5.0,
    )

    assert decision.state == SupervisorState.HOLD
    assert decision.navigation_allowed is False


def test_wheel_and_scan_fault_pair_is_confirmed_before_hold():
    """Losing both translation sources is a necessary-stop condition."""
    policy = ResilienceSupervisorPolicy()
    evidence = healthy_evidence(
        wheel_state=SensorState.FAULT,
        scan_state=SensorState.FAULT,
        fusion_state=FusionState.DEGRADED,
        accepted_measurements=('imu_yaw_rate',),
    )
    pending = policy.evaluate(
        evidence, now_sec=5.0, elapsed_since_start_sec=5.0
    )
    held = policy.evaluate(
        evidence, now_sec=5.31, elapsed_since_start_sec=5.31
    )

    assert pending.navigation_allowed is True
    assert held.state == SupervisorState.HOLD


def test_hard_confirmation_resets_when_evidence_recovers():
    """Separated hard spikes cannot accumulate into a later HOLD."""
    policy = ResilienceSupervisorPolicy()
    hard = healthy_evidence(
        fusion_state=FusionState.HOLD, accepted_measurements=()
    )
    policy.evaluate(hard, now_sec=5.0, elapsed_since_start_sec=5.0)
    recovered = policy.evaluate(
        healthy_evidence(), now_sec=5.2, elapsed_since_start_sec=5.2
    )
    next_spike = policy.evaluate(
        hard, now_sec=5.4, elapsed_since_start_sec=5.4
    )

    assert recovered.state == SupervisorState.NAVIGATE
    assert next_spike.state == SupervisorState.DEGRADED
    assert next_spike.navigation_allowed is True
