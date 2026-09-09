"""Pure localization-quality decisions without ROS or benchmark truth."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum


class LocalizationState(IntEnum):
    """Wire-compatible localization-quality states."""

    PROVISIONAL = 0
    OK = 1
    DEGRADED = 2
    LOST = 3


class ScanHealthState(IntEnum):
    """Mirror SensorHealth values without importing generated ROS code."""

    UNKNOWN = 0
    HEALTHY = 1
    DEGRADED = 2
    FAULT = 3


def valid_runtime_stamp(stamp_sec: float) -> bool:
    """Accept finite positive stamps regardless of callback arrival order.

    With simulated time, a sensor or TF callback can be dispatched before the
    corresponding /clock callback.  Freshness still uses both receipt age and
    header-stamp age, so rejecting such a stamp here would create a permanent
    false LOST without improving stale-data detection.
    """
    return math.isfinite(stamp_sec) and stamp_sec > 0.0


@dataclass(frozen=True)
class LocalizationQualityConfig:
    """Configurable thresholds for map-frame localization evidence."""

    startup_grace_sec: float = 3.0
    pose_age_warning_sec: float = 0.8
    pose_age_lost_sec: float = 2.0
    tf_age_warning_sec: float = 0.8
    tf_age_lost_sec: float = 2.0
    scan_health_age_warning_sec: float = 0.8
    scan_health_age_lost_sec: float = 2.0
    position_variance_warning: float = 0.5
    position_variance_lost: float = 2.0
    yaw_variance_warning: float = 0.3
    yaw_variance_lost: float = 1.0
    position_jump_warning_m: float = 0.75
    position_jump_lost_m: float = 2.0
    yaw_jump_warning_rad: float = 0.8
    yaw_jump_lost_rad: float = 1.6
    recovery_clean_observations: int = 2

    def __post_init__(self) -> None:
        """Reject invalid or reversed warning/lost thresholds."""
        ordered_pairs = (
            ('pose age', self.pose_age_warning_sec, self.pose_age_lost_sec),
            ('TF age', self.tf_age_warning_sec, self.tf_age_lost_sec),
            (
                'scan-health age',
                self.scan_health_age_warning_sec,
                self.scan_health_age_lost_sec,
            ),
            (
                'position variance',
                self.position_variance_warning,
                self.position_variance_lost,
            ),
            (
                'yaw variance',
                self.yaw_variance_warning,
                self.yaw_variance_lost,
            ),
            (
                'position jump',
                self.position_jump_warning_m,
                self.position_jump_lost_m,
            ),
            ('yaw jump', self.yaw_jump_warning_rad, self.yaw_jump_lost_rad),
        )
        if (
            not math.isfinite(self.startup_grace_sec)
            or self.startup_grace_sec < 0.0
        ):
            raise ValueError('startup grace must be finite and non-negative')
        if (
            type(self.recovery_clean_observations) is not int
            or self.recovery_clean_observations < 2
        ):
            raise ValueError('recovery_clean_observations must be at least 2')
        for name, warning, lost in ordered_pairs:
            if not all(math.isfinite(value) for value in (warning, lost)):
                raise ValueError(f'{name} thresholds must be finite')
            if warning < 0.0 or lost <= warning:
                raise ValueError(
                    f'{name} thresholds must be non-negative and ordered'
                )


@dataclass(frozen=True)
class LocalizationEvidence:
    """Latest sanitized runtime observations used by the decision layer."""

    pose_age_sec: float | None = None
    map_to_odom_age_sec: float | None = None
    scan_health_age_sec: float | None = None
    position_variance: float | None = None
    yaw_variance: float | None = None
    position_jump_m: float = 0.0
    yaw_jump_rad: float = 0.0
    scan_health_state: int | None = None
    pose_valid: bool = True
    map_to_odom_valid: bool = True
    update_sequence: int = 0


@dataclass(frozen=True)
class LocalizationDecision:
    """A compact state-machine output for the ROS adapter."""

    state: LocalizationState
    localization_usable: bool
    confidence: float
    reasons: tuple[str, ...]


class LocalizationQualityEvaluator:
    """Fuse pose, TF, and scan-health evidence with fast startup/recovery."""

    def __init__(
        self, config: LocalizationQualityConfig | None = None
    ) -> None:
        """Create an evaluator in the explicit startup state."""
        self.config = config or LocalizationQualityConfig()
        self.state = LocalizationState.PROVISIONAL
        self._recovery_active = False
        self._recovery_clean_count = 0
        self._last_recovery_sequence = None

    def evaluate(
        self,
        evidence: LocalizationEvidence,
        elapsed_since_start_sec: float,
    ) -> LocalizationDecision:
        """Evaluate a snapshot with a short evidence-counted recovery."""
        if not math.isfinite(elapsed_since_start_sec):
            raise ValueError('startup elapsed time must be finite')
        elapsed_since_start_sec = max(elapsed_since_start_sec, 0.0)
        hard_reasons = self._hard_reasons(evidence)
        if hard_reasons:
            self._begin_recovery()
            return self._decision(
                LocalizationState.LOST, False, 0.0, hard_reasons
            )

        core_complete = self._core_complete(evidence)
        if elapsed_since_start_sec < self.config.startup_grace_sec:
            if not core_complete:
                reasons = self._missing_core_reasons(evidence)
                return self._decision(
                    LocalizationState.PROVISIONAL,
                    False,
                    0.2,
                    reasons or ['localization_starting'],
                )
            warning_reasons = self._warning_reasons(evidence)
            scan_is_warming_up = evidence.scan_health_state in (
                None,
                ScanHealthState.UNKNOWN,
            )
            warmup_reasons = {'scan_health_missing', 'scan_health_unknown'}
            if (
                scan_is_warming_up
                and set(warning_reasons).issubset(warmup_reasons)
            ):
                return self._decision(
                    LocalizationState.PROVISIONAL,
                    True,
                    0.6,
                    self._scan_reasons(evidence) or ['scan_health_warming_up'],
                )
            if warning_reasons:
                self._reset_clean_recovery_count()
                return self._decision(
                    LocalizationState.DEGRADED,
                    True,
                    0.5,
                    warning_reasons,
                )

        if not core_complete:
            reasons = self._missing_core_reasons(evidence)
            if evidence.map_to_odom_age_sec is None:
                self._begin_recovery()
                return self._decision(
                    LocalizationState.LOST, False, 0.0, reasons
                )
            self._reset_clean_recovery_count()
            return self._decision(
                LocalizationState.DEGRADED, True, 0.5, reasons
            )

        warning_reasons = self._warning_reasons(evidence)
        if warning_reasons:
            self._reset_clean_recovery_count()
            return self._decision(
                LocalizationState.DEGRADED,
                True,
                0.5,
                warning_reasons,
            )
        if self._recovery_active:
            self._record_clean_recovery_observation(evidence.update_sequence)
            if (
                self._recovery_clean_count
                < self.config.recovery_clean_observations
            ):
                return self._decision(
                    LocalizationState.DEGRADED,
                    True,
                    0.6,
                    ['localization_recovery_pending'],
                )
            self._recovery_active = False
            self._reset_clean_recovery_count()
        return self._decision(
            LocalizationState.OK, True, 1.0, ['all_checks_ok']
        )

    def _begin_recovery(self) -> None:
        self._recovery_active = True
        self._reset_clean_recovery_count()

    def _reset_clean_recovery_count(self) -> None:
        self._recovery_clean_count = 0
        self._last_recovery_sequence = None

    def _record_clean_recovery_observation(self, sequence: int) -> None:
        if sequence == self._last_recovery_sequence:
            return
        self._last_recovery_sequence = sequence
        self._recovery_clean_count += 1

    def _decision(
        self, state, usable, confidence, reasons
    ) -> LocalizationDecision:
        self.state = state
        return LocalizationDecision(state, usable, confidence, tuple(reasons))

    @staticmethod
    def _core_complete(evidence: LocalizationEvidence) -> bool:
        return all(
            value is not None
            for value in (
                evidence.pose_age_sec,
                evidence.map_to_odom_age_sec,
                evidence.position_variance,
                evidence.yaw_variance,
            )
        )

    @staticmethod
    def _missing_core_reasons(evidence: LocalizationEvidence) -> list[str]:
        reasons = []
        if evidence.pose_age_sec is None:
            reasons.append('amcl_pose_missing')
        if evidence.map_to_odom_age_sec is None:
            reasons.append('map_to_odom_tf_missing')
        if evidence.position_variance is None or evidence.yaw_variance is None:
            reasons.append('amcl_covariance_missing')
        return reasons

    def _hard_reasons(self, evidence: LocalizationEvidence) -> list[str]:
        config = self.config
        reasons = []
        if not evidence.pose_valid:
            reasons.append('amcl_pose_invalid')
        if not evidence.map_to_odom_valid:
            reasons.append('map_to_odom_tf_invalid')
        self._append_threshold_reason(
            reasons, evidence.map_to_odom_age_sec, config.tf_age_lost_sec,
            'map_to_odom_tf_stale',
        )
        self._append_threshold_reason(
            reasons, evidence.position_variance, config.position_variance_lost,
            'amcl_position_variance_lost',
        )
        self._append_threshold_reason(
            reasons, evidence.yaw_variance, config.yaw_variance_lost,
            'amcl_yaw_variance_lost',
        )
        self._append_threshold_reason(
            reasons, evidence.position_jump_m, config.position_jump_lost_m,
            'amcl_position_jump_lost',
        )
        self._append_threshold_reason(
            reasons, evidence.yaw_jump_rad, config.yaw_jump_lost_rad,
            'amcl_yaw_jump_lost',
        )
        return reasons

    def _warning_reasons(self, evidence: LocalizationEvidence) -> list[str]:
        config = self.config
        reasons = []
        if evidence.pose_age_sec is not None:
            if (
                not math.isfinite(evidence.pose_age_sec)
                or evidence.pose_age_sec >= config.pose_age_lost_sec
            ):
                reasons.append('amcl_pose_stale')
            elif evidence.pose_age_sec >= config.pose_age_warning_sec:
                reasons.append('amcl_pose_delayed')
        self._append_threshold_reason(
            reasons, evidence.map_to_odom_age_sec, config.tf_age_warning_sec,
            'map_to_odom_tf_delayed',
        )
        self._append_threshold_reason(
            reasons, evidence.position_variance,
            config.position_variance_warning,
            'amcl_position_variance_high',
        )
        self._append_threshold_reason(
            reasons, evidence.yaw_variance, config.yaw_variance_warning,
            'amcl_yaw_variance_high',
        )
        self._append_threshold_reason(
            reasons, evidence.position_jump_m, config.position_jump_warning_m,
            'amcl_position_jump',
        )
        self._append_threshold_reason(
            reasons, evidence.yaw_jump_rad, config.yaw_jump_warning_rad,
            'amcl_yaw_jump',
        )
        reasons.extend(self._scan_reasons(evidence))
        return reasons

    def _scan_reasons(self, evidence: LocalizationEvidence) -> list[str]:
        reasons = []
        age = evidence.scan_health_age_sec
        if evidence.scan_health_state is None:
            reasons.append('scan_health_missing')
        elif evidence.scan_health_state == ScanHealthState.UNKNOWN:
            reasons.append('scan_health_unknown')
        elif evidence.scan_health_state == ScanHealthState.DEGRADED:
            reasons.append('scan_health_degraded')
        elif evidence.scan_health_state == ScanHealthState.FAULT:
            reasons.append('scan_health_fault')
        elif evidence.scan_health_state != ScanHealthState.HEALTHY:
            reasons.append('scan_health_invalid')
        if age is not None:
            if age >= self.config.scan_health_age_lost_sec:
                reasons.append('scan_health_stale')
            elif age >= self.config.scan_health_age_warning_sec:
                reasons.append('scan_health_delayed')
        return reasons

    @staticmethod
    def _append_threshold_reason(reasons, value, threshold, reason) -> None:
        if value is not None and (
            not math.isfinite(value) or value >= threshold
        ):
            reasons.append(reason)
