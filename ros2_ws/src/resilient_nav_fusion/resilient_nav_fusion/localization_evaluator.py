"""Pure, fail-closed trajectory metrics for Phase 8 evaluation only."""

from dataclasses import asdict, dataclass
from math import atan2, isfinite, pi, sqrt
from typing import Iterable


class EvaluationError(ValueError):
    """Raised when metrics cannot be derived without guessing."""


@dataclass(frozen=True)
class TimedPose:
    """A planar pose at one valid, monotonically ordered timestamp."""

    stamp_sec: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class TrajectoryMetrics:
    """Error summary for one estimate against the same truth samples."""

    sample_count: int
    position_rmse: float
    yaw_rmse: float
    max_position_error: float
    max_yaw_error: float
    final_position_drift: float
    final_yaw_drift: float


@dataclass(frozen=True)
class LocalizationEvaluation:
    """Comparable fixed and adaptive metrics plus an explicit benefit."""

    fixed: TrajectoryMetrics
    adaptive: TrajectoryMetrics
    position_rmse_benefit: float
    position_rmse_improvement_ratio: float | None

    def to_dict(self) -> dict[str, object]:
        """Return JSON-safe, stable structured output for benchmarks."""
        return asdict(self)


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi) without altering its physical meaning."""
    return (angle + pi) % (2.0 * pi) - pi


def evaluate_trajectories(
    ground_truth: Iterable[TimedPose],
    fixed: Iterable[TimedPose],
    adaptive: Iterable[TimedPose],
    *,
    max_alignment_delta_sec: float = 0.05,
    min_samples: int = 3,
) -> LocalizationEvaluation:
    """Time-align all three trajectories and calculate fixed/adaptive metrics.

    Each truth sample must have one nearest fixed and adaptive sample inside the
    declared tolerance.  Inputs that cannot support this unambiguous evaluation
    raise ``EvaluationError`` rather than yielding partial or invented metrics.
    """
    if not isfinite(max_alignment_delta_sec) or max_alignment_delta_sec < 0.0:
        raise EvaluationError('max_alignment_delta_sec must be finite and non-negative')
    if min_samples < 1:
        raise EvaluationError('min_samples must be positive')

    truth_samples = _validated_samples('ground_truth', ground_truth, min_samples)
    fixed_samples = _validated_samples('fixed', fixed, min_samples)
    adaptive_samples = _validated_samples('adaptive', adaptive, min_samples)
    aligned_fixed: list[tuple[TimedPose, TimedPose]] = []
    aligned_adaptive: list[tuple[TimedPose, TimedPose]] = []

    for truth in truth_samples:
        fixed_match = _nearest_within_tolerance(
            truth, fixed_samples, max_alignment_delta_sec
        )
        adaptive_match = _nearest_within_tolerance(
            truth, adaptive_samples, max_alignment_delta_sec
        )
        if fixed_match is None or adaptive_match is None:
            continue
        aligned_fixed.append((truth, fixed_match))
        aligned_adaptive.append((truth, adaptive_match))

    if len(aligned_fixed) < min_samples:
        raise EvaluationError('insufficient mutually time-aligned samples')
    fixed_metrics = _metrics(aligned_fixed)
    adaptive_metrics = _metrics(aligned_adaptive)
    benefit = fixed_metrics.position_rmse - adaptive_metrics.position_rmse
    ratio = (
        benefit / fixed_metrics.position_rmse
        if fixed_metrics.position_rmse > 0.0
        else None
    )
    return LocalizationEvaluation(
        fixed=fixed_metrics,
        adaptive=adaptive_metrics,
        position_rmse_benefit=benefit,
        position_rmse_improvement_ratio=ratio,
    )


def evaluate_alignment_sensitivity(
    ground_truth: Iterable[TimedPose],
    fixed: Iterable[TimedPose],
    adaptive: Iterable[TimedPose],
    *,
    windows_sec: Iterable[float],
    min_samples: int = 3,
) -> dict[str, object]:
    """Evaluate declared alignment tolerances without changing any trajectory.

    Each window is independently fail-closed.  A narrow tolerance that cannot
    form a common trajectory reports ``INSUFFICIENT_ALIGNED_SAMPLES`` instead
    of inheriting or extrapolating results from a wider window.
    """
    truth_samples = tuple(ground_truth)
    fixed_samples = tuple(fixed)
    adaptive_samples = tuple(adaptive)
    results: dict[str, object] = {}
    for window in windows_sec:
        if not isfinite(window) or window < 0.0:
            raise EvaluationError('alignment windows must be finite and non-negative')
        key = f'{window:.3f}'
        try:
            metrics = evaluate_trajectories(
                truth_samples,
                fixed_samples,
                adaptive_samples,
                max_alignment_delta_sec=window,
                min_samples=min_samples,
            ).to_dict()
        except EvaluationError as error:
            results[key] = {
                'outcome': 'INSUFFICIENT_ALIGNED_SAMPLES',
                'error': str(error),
            }
            continue
        fixed_metrics = metrics['fixed']
        adaptive_metrics = metrics['adaptive']
        results[key] = {
            'outcome': 'PASS',
            'metrics': metrics,
            'adaptive_improved': (
                adaptive_metrics['position_rmse'] < fixed_metrics['position_rmse']
                and adaptive_metrics['yaw_rmse'] < fixed_metrics['yaw_rmse']
            ),
        }
    return results


def _validated_samples(
    name: str, samples: Iterable[TimedPose], min_samples: int
) -> list[TimedPose]:
    values = list(samples)
    if len(values) < min_samples:
        raise EvaluationError(f'{name} has fewer than {min_samples} samples')
    previous_stamp: float | None = None
    for sample in values:
        if not all(isfinite(value) for value in (sample.stamp_sec, sample.x, sample.y, sample.yaw)):
            raise EvaluationError(f'{name} contains a missing or non-finite value')
        if sample.stamp_sec < 0.0:
            raise EvaluationError(f'{name} contains a negative timestamp')
        if previous_stamp is not None and sample.stamp_sec <= previous_stamp:
            raise EvaluationError(f'{name} timestamps must be strictly increasing')
        previous_stamp = sample.stamp_sec
    return values


def _nearest_within_tolerance(
    truth: TimedPose, estimates: list[TimedPose], tolerance: float
) -> TimedPose | None:
    nearest = min(estimates, key=lambda item: abs(item.stamp_sec - truth.stamp_sec))
    if abs(nearest.stamp_sec - truth.stamp_sec) > tolerance:
        return None
    return nearest


def _metrics(aligned: list[tuple[TimedPose, TimedPose]]) -> TrajectoryMetrics:
    position_errors = [
        sqrt((estimate.x - truth.x) ** 2 + (estimate.y - truth.y) ** 2)
        for truth, estimate in aligned
    ]
    yaw_errors = [abs(normalize_angle(estimate.yaw - truth.yaw)) for truth, estimate in aligned]
    count = len(aligned)
    return TrajectoryMetrics(
        sample_count=count,
        position_rmse=sqrt(sum(error**2 for error in position_errors) / count),
        yaw_rmse=sqrt(sum(error**2 for error in yaw_errors) / count),
        max_position_error=max(position_errors),
        max_yaw_error=max(yaw_errors),
        final_position_drift=position_errors[-1],
        final_yaw_drift=yaw_errors[-1],
    )
