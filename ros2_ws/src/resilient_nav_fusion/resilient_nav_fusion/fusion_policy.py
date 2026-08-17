"""Deterministic, ROS-free health-aware measurement selection policy."""

from dataclasses import dataclass
from enum import Enum
from typing import Mapping


class HealthState(str, Enum):
    """Allowed sanitized single-sensor health states."""

    UNKNOWN = 'unknown'
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
class FusionPolicyConfig:
    """Explicit policy values; none are applied to an EKF in this milestone."""

    wheel_degraded_covariance_scale: float = 4.0
    imu_degraded_covariance_scale: float = 4.0
    wheel_yaw_fallback_covariance_scale: float = 8.0
    recovery_confirmation_cycles: int = 2

    def __post_init__(self):
        for name, value in (
            ('wheel_degraded_covariance_scale', self.wheel_degraded_covariance_scale),
            ('imu_degraded_covariance_scale', self.imu_degraded_covariance_scale),
            (
                'wheel_yaw_fallback_covariance_scale',
                self.wheel_yaw_fallback_covariance_scale,
            ),
        ):
            if value < 1.0:
                raise ValueError(f'{name} must be greater than or equal to 1.0')
        if self.recovery_confirmation_cycles < 1:
            raise ValueError('recovery_confirmation_cycles must be at least 1')


@dataclass(frozen=True)
class FusionDecision:
    """One side-effect-free measurement selection result."""

    state: FusionState
    accepted_measurements: tuple[str, ...]
    rejected_measurements: tuple[str, ...]
    covariance_scales: Mapping[str, float | None]
    wheel_yaw_fallback_enabled: bool
    reasons: tuple[str, ...]
    confidence: float


@dataclass(frozen=True)
class _MeasurementAction:
    """The internally latched acceptance state of one primary measurement."""

    accepted: bool
    covariance_scale: float | None
    health_state: HealthState


class FusionPolicy:
    """Select wheel and IMU measurements with conservative recovery hysteresis.

    Rejection for ``FAULT`` or ``UNKNOWN`` is immediate.  Any transition from
    rejected to accepted, or from a larger covariance scale to a smaller one,
    requires consecutive observations specified by ``recovery_confirmation_cycles``.
    More conservative changes, including degradation, apply immediately.
    """

    WHEEL_VELOCITY = 'wheel_velocity'
    IMU_YAW_RATE = 'imu_yaw_rate'
    WHEEL_YAW_RATE = 'wheel_yaw_rate'

    def __init__(self, config: FusionPolicyConfig | None = None):
        self._config = config or FusionPolicyConfig()
        self._latched: dict[str, _MeasurementAction] = {}
        self._recovery_counts = {
            self.WHEEL_VELOCITY: 0,
            self.IMU_YAW_RATE: 0,
        }

    @property
    def config(self) -> FusionPolicyConfig:
        """Return the immutable configuration used by this policy."""
        return self._config

    def decide(
        self,
        wheel_health: MeasurementHealth | HealthState | str,
        imu_health: MeasurementHealth | HealthState | str,
    ) -> FusionDecision:
        """Return a decision from sanitized wheel and IMU health only."""
        wheel = self._coerce_health(wheel_health)
        imu = self._coerce_health(imu_health)

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

        fallback_enabled = wheel_action.accepted and not imu_action.accepted
        scales = {
            self.WHEEL_VELOCITY: wheel_action.covariance_scale,
            self.IMU_YAW_RATE: imu_action.covariance_scale,
            self.WHEEL_YAW_RATE: (
                wheel_action.covariance_scale
                * self._config.wheel_yaw_fallback_covariance_scale
                if fallback_enabled
                else None
            ),
        }
        accepted = tuple(
            name
            for name in (
                self.WHEEL_VELOCITY,
                self.IMU_YAW_RATE,
                self.WHEEL_YAW_RATE,
            )
            if scales[name] is not None
        )
        rejected = tuple(
            name
            for name in (
                self.WHEEL_VELOCITY,
                self.IMU_YAW_RATE,
                self.WHEEL_YAW_RATE,
            )
            if scales[name] is None
        )
        state = self._fusion_state(
            wheel,
            imu,
            wheel_action,
            imu_action,
        )
        reasons = self._reasons(
            wheel,
            imu,
            wheel_action,
            imu_action,
            wheel_pending,
            imu_pending,
            fallback_enabled,
        )
        return FusionDecision(
            state=state,
            accepted_measurements=accepted,
            rejected_measurements=rejected,
            covariance_scales=scales,
            wheel_yaw_fallback_enabled=fallback_enabled,
            reasons=reasons,
            confidence=self._confidence(state, wheel, imu, wheel_action, imu_action),
        )

    def _desired_action(
        self,
        health_state: HealthState,
        degraded_scale: float,
    ) -> _MeasurementAction:
        if health_state is HealthState.HEALTHY:
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
        wheel_action: _MeasurementAction,
        imu_action: _MeasurementAction,
    ) -> FusionState:
        if not wheel_action.accepted and not imu_action.accepted:
            if (
                wheel.state is HealthState.UNKNOWN
                and imu.state is HealthState.UNKNOWN
            ):
                return FusionState.UNKNOWN
            return FusionState.HOLD
        if (
            wheel_action.accepted
            and imu_action.accepted
            and wheel_action.covariance_scale == 1.0
            and imu_action.covariance_scale == 1.0
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
        wheel_pending: bool,
        imu_pending: bool,
        fallback_enabled: bool,
    ) -> tuple[str, ...]:
        reasons = [
            self._measurement_reason(
                self.WHEEL_VELOCITY, wheel, wheel_action, wheel_pending
            ),
            self._measurement_reason(
                self.IMU_YAW_RATE, imu, imu_action, imu_pending
            ),
        ]
        if fallback_enabled:
            reasons.append('wheel_yaw_fallback_enabled')
        else:
            reasons.append('wheel_yaw_fallback_disabled')
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
        if action.accepted:
            return f'{measurement}_accepted_healthy'
        return f'{measurement}_rejected_{health.state.value}'

    @staticmethod
    def _confidence(
        state: FusionState,
        wheel: MeasurementHealth,
        imu: MeasurementHealth,
        wheel_action: _MeasurementAction,
        imu_action: _MeasurementAction,
    ) -> float:
        accepted_confidences = [
            health.confidence
            for health, action in ((wheel, wheel_action), (imu, imu_action))
            if action.accepted
        ]
        if not accepted_confidences:
            return 0.0
        confidence = min(accepted_confidences)
        if state is FusionState.DEGRADED:
            confidence *= 0.75 if len(accepted_confidences) == 2 else 0.5
        return confidence
