"""Deterministic, ROS-free health-aware measurement selection policy."""

from dataclasses import dataclass
from enum import Enum
from typing import Mapping


class HealthState(str, Enum):
    """Allowed sanitized single-sensor health states."""

    UNKNOWN = 'unknown'
    PROVISIONAL = 'provisional'
    HEALTHY = 'healthy'
    DEGRADED = 'degraded'
    FAULT = 'fault'


class FusionState(str, Enum):
    """Runtime state aligned with the FusionStatus state contract."""

    UNKNOWN = 'unknown'
    NOMINAL = 'nominal'
    DEGRADED = 'degraded'
    HOLD = 'hold'


@dataclass(frozen=True)
class MeasurementHealth:
    """Sanitized health evidence used by the pure policy."""

    state: HealthState
    confidence: float = 1.0

    def __post_init__(self):
        object.__setattr__(self, 'state', HealthState(self.state))
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError('measurement health confidence must be in [0, 1]')


@dataclass(frozen=True)
class ReliabilityScores:
    """GT-free measurement probabilities supplied by the RF/ICP layer."""

    wheel_translation: float = 1.0
    wheel_rotation: float = 1.0
    imu_yaw_rate: float = 1.0
    lidar_translation: float = 1.0
    rf_ready: bool = False

    def __post_init__(self) -> None:
        for name in (
            'wheel_translation',
            'wheel_rotation',
            'imu_yaw_rate',
            'lidar_translation',
        ):
            value = float(getattr(self, name))
            if not 0.0 <= value <= 1.0:
                raise ValueError(f'{name} reliability must be in [0, 1]')


@dataclass(frozen=True)
class FusionPolicyConfig:
    """Explicit covariance and recovery values enforced by the adapter."""

    wheel_degraded_covariance_scale: float = 4.0
    imu_degraded_covariance_scale: float = 4.0
    lidar_degraded_covariance_scale: float = 4.0
    wheel_yaw_fallback_covariance_scale: float = 8.0
    recovery_confirmation_cycles: int = 1
    nominal_reliability: float = 0.95
    reliability_floor: float = 0.05
    maximum_covariance_scale: float = 100.0
    fallback_reliability_threshold: float = 0.10

    def __post_init__(self):
        for name, value in (
            ('wheel_degraded_covariance_scale', self.wheel_degraded_covariance_scale),
            ('imu_degraded_covariance_scale', self.imu_degraded_covariance_scale),
            (
                'lidar_degraded_covariance_scale',
                self.lidar_degraded_covariance_scale,
            ),
            (
                'wheel_yaw_fallback_covariance_scale',
                self.wheel_yaw_fallback_covariance_scale,
            ),
        ):
            if value < 1.0:
                raise ValueError(f'{name} must be greater than or equal to 1.0')
        if self.recovery_confirmation_cycles < 1:
            raise ValueError('recovery_confirmation_cycles must be at least 1')
        if not 0.0 < self.reliability_floor < self.nominal_reliability <= 1.0:
            raise ValueError('reliability floor/nominal values are invalid')
        if self.maximum_covariance_scale < 1.0:
            raise ValueError('maximum_covariance_scale must be at least 1.0')
        if not 0.0 <= self.fallback_reliability_threshold <= 1.0:
            raise ValueError('fallback_reliability_threshold must be in [0, 1]')


@dataclass(frozen=True)
class FusionDecision:
    """One side-effect-free measurement selection result."""

    state: FusionState
    accepted_measurements: tuple[str, ...]
    rejected_measurements: tuple[str, ...]
    covariance_scales: Mapping[str, float | None]
    wheel_yaw_fallback_enabled: bool
    lidar_translation_fallback_enabled: bool
    reasons: tuple[str, ...]
    confidence: float


@dataclass(frozen=True)
class _MeasurementAction:
    """The internally latched acceptance state of one primary measurement."""

    accepted: bool
    covariance_scale: float | None
    health_state: HealthState


class FusionPolicy:
    """Select wheel, IMU, and fallback LiDAR measurements with hysteresis.

    Rejection for ``FAULT`` or ``UNKNOWN`` is immediate.  The first legal
    startup measurement may enter as ``PROVISIONAL`` without being mistaken
    for fault recovery.  After a measurement has been active, recovery from a
    rejection or covariance reduction still requires consecutive observations.
    """

    # Stable public status name for the primary wheel Odometry channel. The
    # adapter now forwards both longitudinal velocity and yaw pose from it.
    WHEEL_VELOCITY = 'wheel_velocity'
    WHEEL_ROTATION = 'wheel_rotation'
    LIDAR_VELOCITY = 'lidar_velocity'
    IMU_YAW_RATE = 'imu_yaw_rate'
    WHEEL_YAW_RATE = 'wheel_yaw_rate'

    def __init__(self, config: FusionPolicyConfig | None = None):
        self._config = config or FusionPolicyConfig()
        self._latched: dict[str, _MeasurementAction] = {}
        self._recovery_counts = {
            self.WHEEL_VELOCITY: 0,
            self.LIDAR_VELOCITY: 0,
            self.IMU_YAW_RATE: 0,
        }
        self._ever_accepted = {
            self.WHEEL_VELOCITY: False,
            self.LIDAR_VELOCITY: False,
            self.IMU_YAW_RATE: False,
        }
        self._fault_observed = {
            self.WHEEL_VELOCITY: False,
            self.LIDAR_VELOCITY: False,
            self.IMU_YAW_RATE: False,
        }

    @property
    def config(self) -> FusionPolicyConfig:
        """Return the immutable configuration used by this policy."""
        return self._config

    def decide(
        self,
        wheel_health: MeasurementHealth | HealthState | str,
        imu_health: MeasurementHealth | HealthState | str,
        lidar_health: (
            MeasurementHealth | HealthState | str
        ) = HealthState.UNKNOWN,
        reliability: ReliabilityScores | None = None,
    ) -> FusionDecision:
        """Return a decision from sanitized wheel, IMU, and LiDAR health."""
        wheel = self._coerce_health(wheel_health)
        imu = self._coerce_health(imu_health)
        lidar = self._coerce_health(lidar_health)
        scores = reliability or ReliabilityScores()

        wheel_action, wheel_pending = self._latch_action(
            self.WHEEL_VELOCITY,
            self._desired_action(
                wheel.state, self._config.wheel_degraded_covariance_scale
            ),
        )
        imu_action, imu_pending = self._latch_action(
            self.IMU_YAW_RATE,
            self._desired_action(
                imu.state, self._config.imu_degraded_covariance_scale
            ),
        )
        lidar_action, lidar_pending = self._latch_action(
            self.LIDAR_VELOCITY,
            self._desired_action(
                lidar.state, self._config.lidar_degraded_covariance_scale
            ),
        )

        wheel_scale = self._combined_scale(
            wheel_action.covariance_scale, scores.wheel_translation
        )
        wheel_rotation_scale = self._combined_scale(
            wheel_action.covariance_scale, scores.wheel_rotation
        )
        imu_scale = self._combined_scale(
            imu_action.covariance_scale, scores.imu_yaw_rate
        )
        lidar_scale = self._combined_scale(
            lidar_action.covariance_scale, scores.lidar_translation
        )
        imu_rf_rejected = (
            scores.rf_ready
            and scores.imu_yaw_rate <= self._config.reliability_floor
        )
        if imu_rf_rejected:
            imu_scale = None
        wheel_yaw_fallback_enabled = (
            wheel_action.accepted
            and wheel_rotation_scale is not None
            and (not imu_action.accepted or imu_rf_rejected)
        )
        lidar_translation_fallback_enabled = (
            lidar_action.accepted
            and lidar_scale is not None
            and scores.lidar_translation > self._config.reliability_floor
            and (
                not wheel_action.accepted
                or (
                    scores.rf_ready
                    and scores.wheel_translation
                    <= self._config.fallback_reliability_threshold
                )
            )
        )
        scales = {
            self.WHEEL_VELOCITY: wheel_scale,
            self.WHEEL_ROTATION: wheel_rotation_scale,
            self.LIDAR_VELOCITY: (
                lidar_scale
                if lidar_translation_fallback_enabled
                else None
            ),
            self.IMU_YAW_RATE: imu_scale,
            self.WHEEL_YAW_RATE: (
                min(
                    self._config.maximum_covariance_scale,
                    wheel_rotation_scale
                    * self._config.wheel_yaw_fallback_covariance_scale,
                )
                if wheel_yaw_fallback_enabled
                else None
            ),
        }
        accepted = tuple(
            name
            for name in (
                self.WHEEL_VELOCITY,
                self.LIDAR_VELOCITY,
                self.IMU_YAW_RATE,
                self.WHEEL_YAW_RATE,
            )
            if scales[name] is not None
        )
        rejected = tuple(
            name
            for name in (
                self.WHEEL_VELOCITY,
                self.LIDAR_VELOCITY,
                self.IMU_YAW_RATE,
                self.WHEEL_YAW_RATE,
            )
            if scales[name] is None
        )
        state = self._fusion_state(
            wheel,
            imu,
            lidar,
            wheel_action,
            imu_action,
            lidar_translation_fallback_enabled,
            scales,
        )
        reasons = self._reasons(
            wheel,
            imu,
            wheel_action,
            imu_action,
            lidar,
            lidar_action,
            wheel_pending,
            imu_pending,
            lidar_pending,
            wheel_yaw_fallback_enabled,
            lidar_translation_fallback_enabled,
            scores,
            imu_rf_rejected,
        )
        return FusionDecision(
            state=state,
            accepted_measurements=accepted,
            rejected_measurements=rejected,
            covariance_scales=scales,
            wheel_yaw_fallback_enabled=wheel_yaw_fallback_enabled,
            lidar_translation_fallback_enabled=(
                lidar_translation_fallback_enabled
            ),
            reasons=reasons,
            confidence=self._confidence(
                state,
                wheel,
                imu,
                lidar,
                wheel_action,
                imu_action,
                lidar_translation_fallback_enabled,
                scores,
            ),
        )

    def _combined_scale(
        self,
        health_scale: float | None,
        reliability: float,
    ) -> float | None:
        if health_scale is None:
            return None
        if reliability >= self._config.nominal_reliability:
            reliability_scale = 1.0
        else:
            bounded = max(float(reliability), self._config.reliability_floor)
            reliability_scale = (self._config.nominal_reliability / bounded) ** 2
        return min(
            self._config.maximum_covariance_scale,
            float(health_scale) * reliability_scale,
        )

    def _desired_action(
        self,
        health_state: HealthState,
        degraded_scale: float,
    ) -> _MeasurementAction:
        if health_state in (HealthState.PROVISIONAL, HealthState.HEALTHY):
            return _MeasurementAction(True, 1.0, health_state)
        if health_state is HealthState.DEGRADED:
            return _MeasurementAction(True, degraded_scale, health_state)
        return _MeasurementAction(False, None, health_state)

    def _latch_action(
        self,
        measurement: str,
        desired: _MeasurementAction,
    ) -> tuple[_MeasurementAction, bool]:
        previous = self._latched.get(measurement)
        if desired.health_state is HealthState.FAULT:
            self._fault_observed[measurement] = True
        if (
            desired.accepted
            and not self._ever_accepted[measurement]
            and not self._fault_observed[measurement]
        ):
            self._latched[measurement] = desired
            self._recovery_counts[measurement] = 0
            self._ever_accepted[measurement] = True
            return desired, False
        if previous is None:
            self._latched[measurement] = desired
            return desired, False
        if not desired.accepted:
            self._latched[measurement] = desired
            self._recovery_counts[measurement] = 0
            return desired, False
        if not previous.accepted:
            return self._confirm_recovery(measurement, previous, desired)
        if desired.covariance_scale >= previous.covariance_scale:
            self._latched[measurement] = desired
            self._recovery_counts[measurement] = 0
            return desired, False
        return self._confirm_recovery(measurement, previous, desired)

    def _confirm_recovery(
        self,
        measurement: str,
        previous: _MeasurementAction,
        desired: _MeasurementAction,
    ) -> tuple[_MeasurementAction, bool]:
        self._recovery_counts[measurement] += 1
        if (
            self._recovery_counts[measurement]
            >= self._config.recovery_confirmation_cycles
        ):
            self._latched[measurement] = desired
            self._recovery_counts[measurement] = 0
            return desired, False
        return previous, True

    @staticmethod
    def _coerce_health(
        health: MeasurementHealth | HealthState | str,
    ) -> MeasurementHealth:
        if isinstance(health, MeasurementHealth):
            return health
        return MeasurementHealth(state=HealthState(health))

    @staticmethod
    def _fusion_state(
        wheel: MeasurementHealth,
        imu: MeasurementHealth,
        lidar: MeasurementHealth,
        wheel_action: _MeasurementAction,
        imu_action: _MeasurementAction,
        lidar_translation_fallback_enabled: bool,
        scales: Mapping[str, float | None],
    ) -> FusionState:
        if (
            not wheel_action.accepted
            and not imu_action.accepted
            and not lidar_translation_fallback_enabled
        ):
            if (
                wheel.state is HealthState.UNKNOWN
                and imu.state is HealthState.UNKNOWN
                and lidar.state is HealthState.UNKNOWN
            ):
                return FusionState.UNKNOWN
            return FusionState.HOLD
        if (
            wheel_action.accepted
            and imu_action.accepted
            and scales[FusionPolicy.WHEEL_VELOCITY] == 1.0
            and scales[FusionPolicy.WHEEL_ROTATION] == 1.0
            and scales[FusionPolicy.IMU_YAW_RATE] == 1.0
            and wheel.state is HealthState.HEALTHY
            and imu.state is HealthState.HEALTHY
        ):
            return FusionState.NOMINAL
        return FusionState.DEGRADED

    def _reasons(
        self,
        wheel: MeasurementHealth,
        imu: MeasurementHealth,
        wheel_action: _MeasurementAction,
        imu_action: _MeasurementAction,
        lidar: MeasurementHealth,
        lidar_action: _MeasurementAction,
        wheel_pending: bool,
        imu_pending: bool,
        lidar_pending: bool,
        wheel_yaw_fallback_enabled: bool,
        lidar_translation_fallback_enabled: bool,
        scores: ReliabilityScores,
        imu_rf_rejected: bool,
    ) -> tuple[str, ...]:
        reasons = [
            self._measurement_reason(
                self.WHEEL_VELOCITY, wheel, wheel_action, wheel_pending
            ),
            self._measurement_reason(
                self.IMU_YAW_RATE, imu, imu_action, imu_pending
            ),
        ]
        if lidar_translation_fallback_enabled:
            reasons.append(self._measurement_reason(
                self.LIDAR_VELOCITY,
                lidar,
                lidar_action,
                lidar_pending,
            ))
        elif wheel_action.accepted:
            reasons.append('lidar_velocity_inactive_wheel_available')
        else:
            reasons.append(
                f'lidar_velocity_rejected_{lidar.state.value}'
            )
        if wheel_yaw_fallback_enabled:
            reasons.append('wheel_yaw_fallback_enabled')
        else:
            reasons.append('wheel_yaw_fallback_disabled')
        if scores.rf_ready:
            reasons.extend((
                f'wheel_translation_reliability_{scores.wheel_translation:.3f}',
                f'wheel_rotation_reliability_{scores.wheel_rotation:.3f}',
                f'imu_yaw_rate_reliability_{scores.imu_yaw_rate:.3f}',
            ))
            if (
                scores.wheel_translation
                <= self._config.fallback_reliability_threshold
            ):
                reasons.append('wheel_translation_rf_extremely_low')
            if scores.wheel_rotation <= self._config.reliability_floor:
                reasons.append('wheel_rotation_rf_extremely_low')
        else:
            reasons.append('rf_window_warming_neutral_weight')
        if imu_rf_rejected:
            reasons.append('imu_yaw_rate_rf_extremely_low')
        return tuple(reasons)

    @staticmethod
    def _measurement_reason(
        measurement: str,
        health: MeasurementHealth,
        action: _MeasurementAction,
        recovery_pending: bool,
    ) -> str:
        if recovery_pending:
            return f'{measurement}_recovery_pending'
        if action.accepted and health.state is HealthState.DEGRADED:
            return f'{measurement}_accepted_degraded'
        if action.accepted and health.state is HealthState.PROVISIONAL:
            return f'{measurement}_accepted_provisional'
        if action.accepted:
            return f'{measurement}_accepted_healthy'
        return f'{measurement}_rejected_{health.state.value}'

    @staticmethod
    def _confidence(
        state: FusionState,
        wheel: MeasurementHealth,
        imu: MeasurementHealth,
        lidar: MeasurementHealth,
        wheel_action: _MeasurementAction,
        imu_action: _MeasurementAction,
        lidar_translation_fallback_enabled: bool,
        scores: ReliabilityScores,
    ) -> float:
        accepted_confidences = []
        if wheel_action.accepted:
            accepted_confidences.append(wheel.confidence)
        if imu_action.accepted:
            accepted_confidences.append(imu.confidence)
        if lidar_translation_fallback_enabled:
            accepted_confidences.append(lidar.confidence)
        if not accepted_confidences:
            return 0.0
        confidence = min(accepted_confidences)
        if scores.rf_ready:
            confidence = min(
                confidence,
                scores.wheel_translation if wheel_action.accepted else 1.0,
                scores.wheel_rotation if wheel_action.accepted else 1.0,
                scores.imu_yaw_rate if imu_action.accepted else 1.0,
                (
                    scores.lidar_translation
                    if lidar_translation_fallback_enabled else 1.0
                ),
            )
        if state is FusionState.DEGRADED:
            confidence *= 0.75 if len(accepted_confidences) == 2 else 0.5
        return confidence
