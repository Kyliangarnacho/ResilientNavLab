"""Pure event ordering and result shaping for Phase 8 sensor benchmarks."""

from dataclasses import dataclass, field
from math import isfinite
from numbers import Real
from typing import Mapping


FAULT_ACTIVE = 1
FAULT_ENDED = 2
SENSOR_HEALTHY = 1
FUSION_NOMINAL = 1


@dataclass
class SensorBenchmarkObserver:
    """Record only benchmark-side events; estimator inputs are never touched."""

    target_sensor: str = 'imu'
    events: dict[str, float] = field(default_factory=dict)
    metrics: Mapping[str, object] | None = None
    metrics_after_recovery: int = 0
    fusion_behavior: dict[str, object] = field(init=False)

    def __post_init__(self) -> None:
        """Restrict benchmark targets to the two implemented policy sensors."""
        if self.target_sensor not in {'healthy', 'imu', 'wheel'}:
            raise ValueError('target_sensor must be healthy, imu, or wheel')
        self.fusion_behavior = {}
        if self.target_sensor == 'wheel':
            self.fusion_behavior = {
                'wheel_velocity_rejected': None,
                'imu_yaw_rate_accepted': None,
                'wheel_yaw_fallback_enabled': None,
            }

    def observe_fault_status(self, state: int, stamp_sec: float) -> None:
        """Record the existing scenario's ACTIVE and ENDED boundaries once."""
        if state == FAULT_ACTIVE and 'fault_start' not in self.events:
            self.events['fault_start'] = stamp_sec
        if (
            state == FAULT_ENDED
            and 'fault_start' in self.events
            and 'fault_end' not in self.events
        ):
            self.events['fault_end'] = stamp_sec

    def observe_target_health(self, state: int, stamp_sec: float) -> None:
        """Record first post-fault anomaly and first post-end healthy state."""
        if 'fault_start' not in self.events:
            return
        abnormal_event = f'{self.target_sensor}_health_first_abnormal'
        recovered_event = f'{self.target_sensor}_health_recovered_healthy'
        if state != SENSOR_HEALTHY and abnormal_event not in self.events:
            self.events[abnormal_event] = stamp_sec
        if (
            'fault_end' in self.events
            and state == SENSOR_HEALTHY
            and recovered_event not in self.events
        ):
            self.events[recovered_event] = stamp_sec

    def observe_imu_health(self, state: int, stamp_sec: float) -> None:
        """Retain the existing IMU-specific test helper."""
        if self.target_sensor != 'imu':
            raise ValueError('IMU helper cannot observe a non-IMU benchmark')
        self.observe_target_health(state, stamp_sec)

    def observe_fusion_status(
        self,
        state: int,
        stamp_sec: float,
        *,
        accepted_measurements: tuple[str, ...] = (),
        rejected_measurements: tuple[str, ...] = (),
    ) -> None:
        """Record policy response and its confirmed return to NOMINAL."""
        if 'fault_start' not in self.events:
            return
        if state != FUSION_NOMINAL and 'fusion_first_non_nominal' not in self.events:
            self.events['fusion_first_non_nominal'] = stamp_sec
        if self.target_sensor == 'wheel' and state != FUSION_NOMINAL:
            self._observe_wheel_fault_response(
                stamp_sec, accepted_measurements, rejected_measurements
            )
        if (
            'fault_end' in self.events
            and state == FUSION_NOMINAL
            and 'fusion_recovered_nominal' not in self.events
        ):
            self.events['fusion_recovered_nominal'] = stamp_sec

    def _observe_wheel_fault_response(
        self,
        stamp_sec: float,
        accepted: tuple[str, ...],
        rejected: tuple[str, ...],
    ) -> None:
        """Verify frozen wheel velocity is rejected while IMU yaw-rate remains."""
        if 'wheel_fault_response' in self.events:
            return
        wheel_rejected = 'wheel_velocity' in rejected
        imu_accepted = 'imu_yaw_rate' in accepted
        fallback_enabled = 'wheel_yaw_rate' in accepted
        self.fusion_behavior = {
            'wheel_velocity_rejected': wheel_rejected,
            'imu_yaw_rate_accepted': imu_accepted,
            'wheel_yaw_fallback_enabled': fallback_enabled,
        }
        if wheel_rejected and imu_accepted and not fallback_enabled:
            self.events.setdefault('wheel_fault_response', stamp_sec)

    def observe_metrics(self, metrics: Mapping[str, object]) -> None:
        """Keep the latest complete evaluator output, including after recovery."""
        if not _has_required_metrics(metrics):
            return
        self.metrics = metrics
        if self.recovered:
            self.metrics_after_recovery += 1

    @property
    def recovered(self) -> bool:
        """Return whether both health and policy reported recovery."""
        if self.target_sensor == 'healthy':
            return True
        return {
            f'{self.target_sensor}_health_recovered_healthy',
            'fusion_recovered_nominal',
        }.issubset(self.events)

    @property
    def complete(self) -> bool:
        """Require all timeline boundaries and fresh post-recovery metrics."""
        return (
            set(self._required_events()).issubset(self.events)
            and self.metrics is not None
            and self.metrics_after_recovery >= 2
        )

    def result(
        self,
        benchmark: str,
        outcome: str,
        *,
        error: str | None = None,
    ) -> dict[str, object]:
        """Return a stable, report-ready benchmark record without inventing data."""
        result: dict[str, object] = {
            'benchmark': benchmark,
            'benchmark_outcome': outcome,
            'adaptive_improved': _adaptive_improved(self.metrics),
            'events': dict(self.events),
            'fusion_behavior': dict(self.fusion_behavior),
            'metrics': self.metrics,
        }
        if error is not None:
            result['error'] = error
            result['missing_requirements'] = self.missing_requirements()
        return result

    def missing_requirements(self) -> list[str]:
        """Name unmet conditions explicitly for fail-closed diagnostics."""
        required = list(self._required_events())
        missing = [name for name in required if name not in self.events]
        if self.metrics is None:
            missing.append('evaluator_metrics')
        elif self.metrics_after_recovery < 2:
            missing.append('post_recovery_evaluator_metrics')
        return missing

    def _required_events(self) -> tuple[str, ...]:
        """Return only the evidence events required for the active sensor."""
        if self.target_sensor == 'healthy':
            return ()
        required = [
            'fault_start',
            f'{self.target_sensor}_health_first_abnormal',
            'fusion_first_non_nominal',
            'fault_end',
            f'{self.target_sensor}_health_recovered_healthy',
            'fusion_recovered_nominal',
        ]
        if self.target_sensor == 'wheel':
            required.append('wheel_fault_response')
        return tuple(required)


def _has_required_metrics(metrics: Mapping[str, object]) -> bool:
    """Accept only the complete fixed/adaptive evaluator shape."""
    try:
        fixed = metrics['fixed']
        adaptive = metrics['adaptive']
        benefit = metrics['position_rmse_benefit']
    except (KeyError, TypeError):
        return False
    if not isinstance(fixed, Mapping) or not isinstance(adaptive, Mapping):
        return False
    required = {
        'position_rmse',
        'yaw_rmse',
        'max_position_error',
        'max_yaw_error',
        'final_position_drift',
        'final_yaw_drift',
        'sample_count',
    }
    if not required.issubset(fixed) or not required.issubset(adaptive):
        return False
    if not _valid_metric_group(fixed) or not _valid_metric_group(adaptive):
        return False
    ratio = metrics.get('position_rmse_improvement_ratio')
    return _finite_real(benefit) and (ratio is None or _finite_real(ratio))


def _valid_metric_group(group: Mapping[str, object]) -> bool:
    """Validate every evaluator numeric field before accepting a result."""
    fields = (
        'position_rmse',
        'yaw_rmse',
        'max_position_error',
        'max_yaw_error',
        'final_position_drift',
        'final_yaw_drift',
    )
    if not all(_finite_real(group[name]) and group[name] >= 0.0 for name in fields):
        return False
    sample_count = group['sample_count']
    return isinstance(sample_count, int) and not isinstance(sample_count, bool) and sample_count > 0


def _finite_real(value: object) -> bool:
    """Reject strings, booleans, NaN, and infinities in benchmark JSON."""
    return isinstance(value, Real) and not isinstance(value, bool) and isfinite(value)


def _adaptive_improved(metrics: Mapping[str, object] | None) -> bool | None:
    """Report improvement separately, requiring both position and yaw gains."""
    if metrics is None or not _has_required_metrics(metrics):
        return None
    fixed = metrics['fixed']
    adaptive = metrics['adaptive']
    return (
        adaptive['position_rmse'] < fixed['position_rmse']
        and adaptive['yaw_rmse'] < fixed['yaw_rmse']
    )


# Backward-compatible names for the completed IMU benchmark imports/tests.
ImuBenchmarkObserver = SensorBenchmarkObserver
ImuBiasBenchmarkObserver = SensorBenchmarkObserver
