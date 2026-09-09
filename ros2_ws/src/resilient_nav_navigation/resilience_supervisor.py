"""Pure, tolerant navigation-level resilience supervision."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math


class SensorState(IntEnum):
    """Wire values from SensorHealth."""

    UNKNOWN = 0
    HEALTHY = 1
    DEGRADED = 2
    FAULT = 3


class FusionState(IntEnum):
    """Wire values from FusionStatus."""

    UNKNOWN = 0
    NOMINAL = 1
    DEGRADED = 2
    HOLD = 3


class LocalizationState(IntEnum):
    """Wire values from LocalizationQuality."""

    PROVISIONAL = 0
    OK = 1
    DEGRADED = 2
    LOST = 3


class SupervisorState(IntEnum):
    """Wire values from ResilienceStatus."""

    STARTING = 0
    NAVIGATE = 1
    DEGRADED = 2
    HOLD = 3


@dataclass(frozen=True)
class SupervisorConfig:
    """Small timing contract for missing inputs and hard-fault persistence."""

    startup_grace_sec: float = 3.0
    status_stale_sec: float = 1.0
    hard_fault_confirmation_sec: float = 0.3

    def __post_init__(self) -> None:
        """Reject invalid timing bounds."""
        for name, value in (
            ('startup_grace_sec', self.startup_grace_sec),
            ('status_stale_sec', self.status_stale_sec),
            ('hard_fault_confirmation_sec', self.hard_fault_confirmation_sec),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f'{name} must be finite and non-negative')
        if self.status_stale_sec <= 0.0:
            raise ValueError('status_stale_sec must be positive')


@dataclass(frozen=True)
class SupervisorEvidence:
    """Latest sanitized monitor outputs and their local receipt ages."""

    wheel_state: int | None = None
    imu_state: int | None = None
    scan_state: int | None = None
    fusion_state: int | None = None
    localization_state: int | None = None
    localization_usable: bool | None = None
    accepted_measurements: tuple[str, ...] = ()
    wheel_age_sec: float | None = None
    imu_age_sec: float | None = None
    scan_age_sec: float | None = None
    fusion_age_sec: float | None = None
    localization_age_sec: float | None = None


@dataclass(frozen=True)
class SupervisorDecision:
    """Navigation-level state and explicit action permission."""

    state: SupervisorState
    navigation_allowed: bool
    confidence: float
    hard_condition_elapsed_sec: float
    reasons: tuple[str, ...]


class ResilienceSupervisorPolicy:
    """Permit degraded operation while confirming non-local hard faults."""

    def __init__(self, config: SupervisorConfig | None = None) -> None:
        """Create a supervisor before any runtime evidence has arrived."""
        self.config = config or SupervisorConfig()
        self._hard_since_sec = None
        self._scan_fault_latched = False

    def evaluate(
        self,
        evidence: SupervisorEvidence,
        now_sec: float,
        elapsed_since_start_sec: float,
    ) -> SupervisorDecision:
        """Return one deterministic supervisor decision."""
        if not all(
            math.isfinite(value)
            for value in (now_sec, elapsed_since_start_sec)
        ):
            raise ValueError('supervisor times must be finite')
        elapsed_since_start_sec = max(elapsed_since_start_sec, 0.0)
        if evidence.scan_state == SensorState.FAULT:
            self._scan_fault_latched = True
        elif evidence.scan_state in (
            SensorState.HEALTHY,
            SensorState.DEGRADED,
        ):
            self._scan_fault_latched = False
        critical_missing = self._critical_missing_reasons(evidence)
        if (
            critical_missing
            and elapsed_since_start_sec < self.config.startup_grace_sec
        ):
            self._hard_since_sec = None
            return SupervisorDecision(
                SupervisorState.STARTING,
                False,
                0.0,
                0.0,
                tuple(critical_missing),
            )

        immediate = self._immediate_hold_reasons(evidence)
        if immediate:
            self._hard_since_sec = now_sec
            return SupervisorDecision(
                SupervisorState.HOLD, False, 0.0, 0.0, tuple(immediate)
            )

        hard = critical_missing + self._confirmed_hold_reasons(evidence)
        if hard:
            if self._hard_since_sec is None:
                self._hard_since_sec = now_sec
            elapsed = max(now_sec - self._hard_since_sec, 0.0)
            if elapsed >= self.config.hard_fault_confirmation_sec:
                return SupervisorDecision(
                    SupervisorState.HOLD,
                    False,
                    0.0,
                    elapsed,
                    tuple(hard),
                )
            return SupervisorDecision(
                SupervisorState.DEGRADED,
                True,
                0.25,
                elapsed,
                tuple(hard + ['hard_condition_confirmation_pending']),
            )

        self._hard_since_sec = None
        soft = self._soft_reasons(evidence)
        if soft:
            return SupervisorDecision(
                SupervisorState.DEGRADED, True, 0.6, 0.0, tuple(soft)
            )
        return SupervisorDecision(
            SupervisorState.NAVIGATE, True, 1.0, 0.0, ('all_systems_ready',)
        )

    def _critical_missing_reasons(
        self, evidence: SupervisorEvidence
    ) -> list[str]:
        """Return missing evidence that prevents safe goal execution."""
        reasons = []
        for name, state, age in (
            ('fusion_status', evidence.fusion_state, evidence.fusion_age_sec),
            (
                'localization_quality',
                evidence.localization_state,
                evidence.localization_age_sec,
            ),
        ):
            if state is None or age is None:
                reasons.append(f'{name}_missing')
            elif not math.isfinite(age) or age >= self.config.status_stale_sec:
                reasons.append(f'{name}_stale')
        if evidence.fusion_state == FusionState.UNKNOWN:
            reasons.append('fusion_not_ready')
        if (
            evidence.localization_state == LocalizationState.PROVISIONAL
            and evidence.localization_usable is not True
        ):
            reasons.append('localization_not_ready')
        return reasons

    @staticmethod
    def _immediate_hold_reasons(evidence: SupervisorEvidence) -> list[str]:
        if evidence.localization_state == LocalizationState.LOST:
            return ['localization_lost']
        if (
            evidence.localization_state is not None
            and evidence.localization_state != LocalizationState.PROVISIONAL
            and evidence.localization_usable is False
        ):
            return ['localization_unusable']
        return []

    def _confirmed_hold_reasons(
        self, evidence: SupervisorEvidence
    ) -> list[str]:
        reasons = []
        if evidence.fusion_state in (FusionState.UNKNOWN, FusionState.HOLD):
            reasons.append('fusion_has_no_safe_solution')
        if (
            evidence.fusion_state is not None
            and not evidence.accepted_measurements
        ):
            reasons.append('fusion_has_no_accepted_measurement')
        if (
            evidence.wheel_state == SensorState.FAULT
            and evidence.imu_state == SensorState.FAULT
        ):
            reasons.append('wheel_and_imu_fault')
        if (
            evidence.wheel_state == SensorState.FAULT
            and evidence.scan_state == SensorState.FAULT
        ):
            reasons.append('wheel_and_scan_fault')
        if self._scan_fault_latched:
            reasons.append(
                'navigation_scan_fault'
                if evidence.scan_state == SensorState.FAULT
                else 'navigation_scan_fault_latched'
            )
        return reasons

    def _soft_reasons(self, evidence: SupervisorEvidence) -> list[str]:
        reasons = []
        for name, state, age in (
            ('wheel', evidence.wheel_state, evidence.wheel_age_sec),
            ('imu', evidence.imu_state, evidence.imu_age_sec),
            ('scan', evidence.scan_state, evidence.scan_age_sec),
        ):
            if state is None or age is None:
                reasons.append(f'{name}_health_missing')
            elif (
                not math.isfinite(age)
                or age >= self.config.status_stale_sec
            ):
                reasons.append(f'{name}_health_stale')
            elif state != SensorState.HEALTHY:
                reasons.append(f'{name}_health_not_healthy')
        if evidence.fusion_state != FusionState.NOMINAL:
            reasons.append('fusion_not_nominal')
        if evidence.localization_state != LocalizationState.OK:
            reasons.append('localization_not_ok')
        return reasons
