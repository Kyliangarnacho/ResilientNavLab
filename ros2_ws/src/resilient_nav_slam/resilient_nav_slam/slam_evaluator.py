"""Pure, evaluation-only metrics for the Phase 9 SLAM trajectories."""

from dataclasses import asdict, dataclass
from math import atan2, cos, sin
from typing import Iterable

import numpy as np

from resilient_nav_fusion.localization_evaluator import (
    EvaluationError,
    TimedPose,
    TrajectoryMetrics,
    _nearest_within_tolerance,
    _validated_samples,
    evaluate_trajectories,
    normalize_angle,
)


@dataclass(frozen=True)
class SE2Alignment:
    """A declared planar transform from an input frame to an output frame."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class MappingTrajectoryMetrics:
    """SLAM metrics after a fixed-scale best-fit SE(2) alignment."""

    sample_count: int
    position_rmse: float
    yaw_rmse: float
    max_position_error: float
    max_yaw_error: float
    final_position_drift: float
    final_yaw_drift: float
    return_to_start_position_error: float
    return_to_start_yaw_error: float
    best_fit_alignment: SE2Alignment

    def to_dict(self) -> dict[str, object]:
        """Return a stable JSON-safe evaluation record."""
        return asdict(self)


@dataclass(frozen=True)
class PersistedMapLocalizationMetrics:
    """Direct persisted-map metrics using one declared ``odom -> map`` SE(2)."""

    slam: TrajectoryMetrics
    healthy_ekf: TrajectoryMetrics
    map_to_odom: SE2Alignment

    def to_dict(self) -> dict[str, object]:
        """Return a stable JSON-safe evaluation record."""
        return asdict(self)


def best_fit_se2_alignment(
    paired: Iterable[tuple[TimedPose, TimedPose]], *, min_samples: int = 3
) -> SE2Alignment:
    """Return the fixed-scale planar Kabsch alignment from estimate to truth.

    The inputs are already timestamp-associated pairs.  This is the 2D rigid
    (not similarity) subset of the SVD alignment used by TUM/evo-style ATE:
    ``truth ~= R @ estimate + t``.  A trajectory with no spatial spread cannot
    determine a rotation, so it is rejected instead of silently anchoring one
    arbitrary sample.
    """
    values = list(paired)
    if len(values) < min_samples:
        raise EvaluationError('insufficient pairs for best-fit SE(2) alignment')

    estimate_points = np.asarray(
        [[estimate.x, estimate.y] for _, estimate in values], dtype=float
    )
    truth_points = np.asarray(
        [[truth.x, truth.y] for truth, _ in values], dtype=float
    )
    if (
        estimate_points.shape != truth_points.shape
        or estimate_points.ndim != 2
        or estimate_points.shape[1] != 2
        or not np.all(np.isfinite(estimate_points))
        or not np.all(np.isfinite(truth_points))
    ):
        raise EvaluationError('non-finite or invalid best-fit SE(2) input')

    estimate_mean = estimate_points.mean(axis=0)
    truth_mean = truth_points.mean(axis=0)
    centered_estimate = estimate_points - estimate_mean
    centered_truth = truth_points - truth_mean
    if np.linalg.norm(centered_estimate, ord='fro') <= np.finfo(float).eps:
        raise EvaluationError('degenerate best-fit SE(2) input has no spatial spread')

    covariance = centered_estimate.T @ centered_truth
    if not np.all(np.isfinite(covariance)):
        raise EvaluationError('non-finite best-fit SE(2) covariance')
    try:
        left, _, right_transpose = np.linalg.svd(covariance)
    except np.linalg.LinAlgError as error:
        raise EvaluationError('best-fit SE(2) SVD failed') from error
    rotation = right_transpose.T @ left.T
    if np.linalg.det(rotation) < 0.0:
        right_transpose[-1, :] *= -1.0
        rotation = right_transpose.T @ left.T
    if not np.all(np.isfinite(rotation)) or np.linalg.det(rotation) <= 0.0:
        raise EvaluationError('invalid best-fit SE(2) rotation')

    translation = truth_mean - rotation @ estimate_mean
    yaw = atan2(rotation[1, 0], rotation[0, 0])
    if not np.all(np.isfinite(translation)) or not np.isfinite(yaw):
        raise EvaluationError('non-finite best-fit SE(2) result')
    return SE2Alignment(
        x=float(translation[0]), y=float(translation[1]), yaw=float(yaw)
    )


def apply_se2_alignment(pose: TimedPose, alignment: SE2Alignment) -> TimedPose:
    """Express one map-frame pose in the Ground Truth frame."""
    return TimedPose(
        stamp_sec=pose.stamp_sec,
        x=alignment.x + cos(alignment.yaw) * pose.x - sin(alignment.yaw) * pose.y,
        y=alignment.y + sin(alignment.yaw) * pose.x + cos(alignment.yaw) * pose.y,
        yaw=normalize_angle(alignment.yaw + pose.yaw),
    )


def transform_trajectory(
    samples: Iterable[TimedPose], alignment: SE2Alignment
) -> list[TimedPose]:
    """Apply one declared, fixed SE(2) transform to evaluation samples."""
    _validate_alignment(alignment)
    return [apply_se2_alignment(sample, alignment) for sample in samples]


def evaluate_mapping_trajectory(
    ground_truth: Iterable[TimedPose],
    slam_pose: Iterable[TimedPose],
    *,
    max_alignment_delta_sec: float = 0.05,
    min_samples: int = 3,
) -> MappingTrajectoryMetrics:
    """Pair, best-fit align, and score a map-frame SLAM trajectory.

    Timestamp validation and nearest-neighbour matching deliberately reuse the
    existing Phase 8 evaluation implementation.  The Phase 9 map-frame result
    uses one fixed-scale best-fit SE(2) transform across the timestamp-
    associated trajectory.  No first-pose transform participates in mapping
    metrics.
    """
    truth_samples = _validated_samples('ground_truth', ground_truth, min_samples)
    estimate_samples = _validated_samples('slam_pose', slam_pose, min_samples)
    paired = _pair_samples(
        truth_samples, estimate_samples, max_alignment_delta_sec
    )
    if len(paired) < min_samples:
        raise EvaluationError('insufficient mutually time-aligned samples')

    alignment = best_fit_se2_alignment(paired, min_samples=min_samples)
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
    return MappingTrajectoryMetrics(
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
        best_fit_alignment=alignment,
    )


def evaluate_persisted_map_localization_trajectory(
    ground_truth_odom: Iterable[TimedPose],
    healthy_ekf_odom: Iterable[TimedPose],
    slam_map_pose: Iterable[TimedPose],
    *,
    map_to_odom: SE2Alignment,
    max_alignment_delta_sec: float = 0.05,
    min_samples: int = 3,
) -> PersistedMapLocalizationMetrics:
    """Evaluate a saved map in its fixed frame without trajectory fitting.

    ``map_to_odom`` is declared by the experiment before samples are captured:
    it expresses the fresh run's odom poses in the persisted map frame.  The
    evaluator transforms Ground Truth and Healthy EKF once, then directly
    compares them with the SLAM map-frame poses.
    """
    ground_truth_map = transform_trajectory(ground_truth_odom, map_to_odom)
    healthy_ekf_map = transform_trajectory(healthy_ekf_odom, map_to_odom)
    slam = _direct_metrics(
        ground_truth_map,
        slam_map_pose,
        max_alignment_delta_sec=max_alignment_delta_sec,
        min_samples=min_samples,
    )
    healthy_ekf = _direct_metrics(
        ground_truth_map,
        healthy_ekf_map,
        max_alignment_delta_sec=max_alignment_delta_sec,
        min_samples=min_samples,
    )
    return PersistedMapLocalizationMetrics(
        slam=slam,
        healthy_ekf=healthy_ekf,
        map_to_odom=map_to_odom,
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


def _validate_alignment(alignment: SE2Alignment) -> None:
    if not all(np.isfinite(value) for value in (
        alignment.x, alignment.y, alignment.yaw
    )):
        raise EvaluationError('fixed SE(2) alignment contains a non-finite value')


def _direct_metrics(
    truth: Iterable[TimedPose],
    estimate: Iterable[TimedPose],
    *,
    max_alignment_delta_sec: float,
    min_samples: int,
) -> TrajectoryMetrics:
    return evaluate_trajectories(
        truth,
        estimate,
        estimate,
        max_alignment_delta_sec=max_alignment_delta_sec,
        min_samples=min_samples,
    ).fixed
