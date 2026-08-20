"""Pure, evaluation-only metrics for the Phase 9 SLAM trajectories."""

from dataclasses import asdict, dataclass
from math import cos, sin
from typing import Iterable

from resilient_nav_fusion.localization_evaluator import (
    EvaluationError,
    TimedPose,
    _nearest_within_tolerance,
    _validated_samples,
    evaluate_trajectories,
    normalize_angle,
)


@dataclass(frozen=True)
class SE2Alignment:
    """A map-frame estimate to Ground Truth-frame planar transform."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class SlamTrajectoryMetrics:
    """SLAM metrics after one declared initial SE(2) alignment."""

    sample_count: int
    position_rmse: float
    yaw_rmse: float
    max_position_error: float
    max_yaw_error: float
    final_position_drift: float
    final_yaw_drift: float
    return_to_start_position_error: float
    return_to_start_yaw_error: float
    initial_alignment: SE2Alignment

    def to_dict(self) -> dict[str, object]:
        """Return a stable JSON-safe evaluation record."""
        return asdict(self)


def initial_se2_alignment(
    truth: TimedPose, estimate: TimedPose
) -> SE2Alignment:
    """Align one map-frame pose to one Ground Truth-frame pose exactly."""
    yaw = normalize_angle(truth.yaw - estimate.yaw)
    aligned_x = cos(yaw) * estimate.x - sin(yaw) * estimate.y
    aligned_y = sin(yaw) * estimate.x + cos(yaw) * estimate.y
    return SE2Alignment(
        x=truth.x - aligned_x,
        y=truth.y - aligned_y,
        yaw=yaw,
    )


def apply_se2_alignment(pose: TimedPose, alignment: SE2Alignment) -> TimedPose:
    """Express one map-frame pose in the Ground Truth frame."""
    return TimedPose(
        stamp_sec=pose.stamp_sec,
        x=alignment.x + cos(alignment.yaw) * pose.x - sin(alignment.yaw) * pose.y,
        y=alignment.y + sin(alignment.yaw) * pose.x + cos(alignment.yaw) * pose.y,
        yaw=normalize_angle(alignment.yaw + pose.yaw),
    )


def evaluate_slam_trajectory(
    ground_truth: Iterable[TimedPose],
    slam_pose: Iterable[TimedPose],
    *,
    max_alignment_delta_sec: float = 0.05,
    min_samples: int = 3,
) -> SlamTrajectoryMetrics:
    """Pair, initially align, and score a map-frame SLAM trajectory.

    Timestamp validation and nearest-neighbour matching deliberately reuse the
    existing Phase 8 evaluation implementation.  The only Phase 9 addition is
    the explicit one-time SE(2) frame alignment required for a map frame whose
    origin differs from the Gazebo Ground Truth frame.
    """
    truth_samples = _validated_samples('ground_truth', ground_truth, min_samples)
    estimate_samples = _validated_samples('slam_pose', slam_pose, min_samples)
    paired = _pair_samples(
        truth_samples, estimate_samples, max_alignment_delta_sec
    )
    if len(paired) < min_samples:
        raise EvaluationError('insufficient mutually time-aligned samples')

    alignment = initial_se2_alignment(*paired[0])
    aligned_estimates = [
        apply_se2_alignment(estimate, alignment) for estimate in estimate_samples
    ]
    phase8_result = evaluate_trajectories(
        truth_samples,
        aligned_estimates,
        aligned_estimates,
        max_alignment_delta_sec=max_alignment_delta_sec,
        min_samples=min_samples,
    )
    metrics = phase8_result.fixed
    aligned_pairs = _pair_samples(
        truth_samples, aligned_estimates, max_alignment_delta_sec
    )
    first_truth, first_estimate = aligned_pairs[0]
    final_truth, final_estimate = aligned_pairs[-1]
    truth_dx = final_truth.x - first_truth.x
    truth_dy = final_truth.y - first_truth.y
    estimate_dx = final_estimate.x - first_estimate.x
    estimate_dy = final_estimate.y - first_estimate.y
    return SlamTrajectoryMetrics(
        sample_count=metrics.sample_count,
        position_rmse=metrics.position_rmse,
        yaw_rmse=metrics.yaw_rmse,
        max_position_error=metrics.max_position_error,
        max_yaw_error=metrics.max_yaw_error,
        final_position_drift=metrics.final_position_drift,
        final_yaw_drift=metrics.final_yaw_drift,
        return_to_start_position_error=(
            (estimate_dx - truth_dx) ** 2 + (estimate_dy - truth_dy) ** 2
        ) ** 0.5,
        return_to_start_yaw_error=abs(normalize_angle(
            (final_estimate.yaw - first_estimate.yaw)
            - (final_truth.yaw - first_truth.yaw)
        )),
        initial_alignment=alignment,
    )


def _pair_samples(
    truth: list[TimedPose],
    estimates: list[TimedPose],
    tolerance: float,
) -> list[tuple[TimedPose, TimedPose]]:
    """Use the Phase 8 nearest timestamp pairing semantics unchanged."""
    if tolerance < 0.0:
        raise EvaluationError('max_alignment_delta_sec must be finite and non-negative')
    pairs = []
    for truth_pose in truth:
        estimate = _nearest_within_tolerance(truth_pose, estimates, tolerance)
        if estimate is not None:
            pairs.append((truth_pose, estimate))
    return pairs
