"""Pure evaluation contracts for Phase 9 map-frame SLAM scoring."""

from math import cos, pi, sin

import pytest

from resilient_nav_fusion.localization_evaluator import EvaluationError, TimedPose
from resilient_nav_slam.slam_evaluator import (
    SE2Alignment,
    apply_se2_alignment,
    best_fit_se2_alignment,
    evaluate_mapping_trajectory,
    evaluate_persisted_map_localization_trajectory,
)
from resilient_nav_slam.slam_evaluator_node import _comparison


def _poses(*, x_offset=0.0, y_offset=0.0, yaw_offset=0.0, time_offset=0.0):
    return [
        TimedPose(
            stamp_sec=float(index) + time_offset,
            x=float(index) + x_offset,
            y=0.25 * index + y_offset,
            yaw=0.1 * index + yaw_offset,
        )
        for index in range(4)
    ]


def test_mapping_metrics_use_one_best_fit_rigid_transform():
    truth = _poses()
    transform = SE2Alignment(10.0, -3.0, 0.6)
    estimate = [
        TimedPose(
            pose.stamp_sec,
            cos(-transform.yaw) * (pose.x - transform.x)
            - sin(-transform.yaw) * (pose.y - transform.y),
            sin(-transform.yaw) * (pose.x - transform.x)
            + cos(-transform.yaw) * (pose.y - transform.y),
            pose.yaw - transform.yaw,
        )
        for pose in truth
    ]

    result = evaluate_mapping_trajectory(truth, estimate)

    assert result.sample_count == 4
    assert result.position_rmse == pytest.approx(0.0)
    assert result.yaw_rmse == pytest.approx(0.0)
    assert result.return_to_start_position_error == pytest.approx(0.0)
    assert result.best_fit_alignment.yaw == pytest.approx(transform.yaw)


def test_best_fit_se2_alignment_recovers_known_rigid_transform():
    estimate = _poses()
    expected = (2.5, -1.25, 0.7)
    truth = [
        TimedPose(
            pose.stamp_sec,
            expected[0] + cos(expected[2]) * pose.x - sin(expected[2]) * pose.y,
            expected[1] + sin(expected[2]) * pose.x + cos(expected[2]) * pose.y,
            pose.yaw + expected[2],
        )
        for pose in estimate
    ]

    result = evaluate_mapping_trajectory(truth, estimate)

    assert result.best_fit_alignment.x == pytest.approx(expected[0])
    assert result.best_fit_alignment.y == pytest.approx(expected[1])
    assert result.best_fit_alignment.yaw == pytest.approx(expected[2])
    assert result.position_rmse == pytest.approx(0.0)
    assert result.yaw_rmse == pytest.approx(0.0)


def test_best_fit_se2_alignment_handles_pure_translation_and_rotation():
    estimate = _poses()
    rotation = -0.8
    truth = [
        TimedPose(
            pose.stamp_sec,
            4.0 + cos(rotation) * pose.x - sin(rotation) * pose.y,
            -2.0 + sin(rotation) * pose.x + cos(rotation) * pose.y,
            pose.yaw + rotation,
        )
        for pose in estimate
    ]

    alignment = best_fit_se2_alignment(list(zip(truth, estimate)))

    assert alignment.x == pytest.approx(4.0)
    assert alignment.y == pytest.approx(-2.0)
    assert alignment.yaw == pytest.approx(rotation)


def test_best_fit_alignment_retains_small_noisy_residuals():
    estimate = _poses()
    rotation = 0.4
    truth = []
    for index, pose in enumerate(estimate):
        noise_x = (-0.01, 0.01, -0.005, 0.005)[index]
        noise_y = (0.005, -0.005, 0.01, -0.01)[index]
        truth.append(TimedPose(
            pose.stamp_sec,
            1.0 + cos(rotation) * pose.x - sin(rotation) * pose.y + noise_x,
            -2.0 + sin(rotation) * pose.x + cos(rotation) * pose.y + noise_y,
            pose.yaw + rotation,
        ))

    result = evaluate_mapping_trajectory(truth, estimate)

    assert 0.0 < result.position_rmse < 0.03
    assert 0.0 < result.yaw_rmse < 0.01


def test_best_fit_alignment_rejects_degenerate_or_insufficient_pairs():
    stationary = [TimedPose(float(index), 1.0, 2.0, 0.0) for index in range(3)]

    with pytest.raises(EvaluationError, match='degenerate'):
        evaluate_mapping_trajectory(stationary, stationary)
    with pytest.raises(EvaluationError, match='insufficient pairs'):
        best_fit_se2_alignment([], min_samples=3)


def test_best_fit_alignment_rejects_non_finite_pair_input():
    valid = _poses()
    invalid = list(valid)
    invalid[2] = TimedPose(2.0, float('nan'), 0.0, 0.0)

    with pytest.raises(EvaluationError, match='non-finite'):
        evaluate_mapping_trajectory(valid, invalid)


def test_mapping_metrics_keep_post_alignment_drift_and_return_error():
    truth = _poses()
    estimate = _poses(x_offset=2.0)
    estimate[-1] = TimedPose(3.0, 6.0, 0.75, 0.3)

    result = evaluate_mapping_trajectory(truth, estimate)

    assert 0.0 < result.final_position_drift < 1.0
    assert 0.0 < result.return_to_start_position_error < 1.0


def test_yaw_wraparound_after_mapping_alignment_uses_shortest_distance():
    truth = [TimedPose(float(index), float(index), 0.0, pi - 0.05) for index in range(4)]
    estimate = [TimedPose(float(index), float(index), 0.0, -pi + 0.05) for index in range(4)]

    result = evaluate_mapping_trajectory(truth, estimate)

    assert result.yaw_rmse == pytest.approx(0.1)
    assert result.max_yaw_error == pytest.approx(0.1)


def test_timestamp_pairing_reuses_fail_closed_phase8_semantics():
    with pytest.raises(EvaluationError, match='time-aligned'):
        evaluate_mapping_trajectory(
            _poses(), _poses(time_offset=0.2), max_alignment_delta_sec=0.05
        )


def test_persisted_map_localization_uses_declared_map_to_odom_transform():
    truth_odom = _poses()
    transform = SE2Alignment(5.5, 4.0, pi)
    healthy_odom = _poses(x_offset=0.2)
    slam_map = [apply_se2_alignment(pose, transform) for pose in truth_odom]

    result = evaluate_persisted_map_localization_trajectory(
        truth_odom, healthy_odom, slam_map, map_to_odom=transform
    )

    assert result.map_to_odom == transform
    assert result.slam.position_rmse == pytest.approx(0.0)
    assert result.slam.yaw_rmse == pytest.approx(0.0)
    assert result.healthy_ekf.position_rmse == pytest.approx(0.2)


def test_persisted_map_localization_does_not_fit_away_a_global_offset():
    truth_odom = _poses()
    transform = SE2Alignment(5.5, 4.0, pi)
    slam_map = [
        TimedPose(pose.stamp_sec, pose.x + 0.75, pose.y, pose.yaw + 0.2)
        for pose in (apply_se2_alignment(sample, transform) for sample in truth_odom)
    ]

    result = evaluate_persisted_map_localization_trajectory(
        truth_odom, truth_odom, slam_map, map_to_odom=transform
    )

    assert result.slam.position_rmse == pytest.approx(0.75)
    assert result.slam.yaw_rmse == pytest.approx(0.2)


def test_persisted_map_comparison_uses_absolute_metrics():
    truth_odom = _poses()
    transform = SE2Alignment(5.5, 4.0, pi)
    persisted = evaluate_persisted_map_localization_trajectory(
        truth_odom,
        truth_odom,
        [
            TimedPose(pose.stamp_sec, pose.x + 0.75, pose.y, pose.yaw + 0.2)
            for pose in (apply_se2_alignment(sample, transform) for sample in truth_odom)
        ],
        map_to_odom=transform,
    )

    comparison = _comparison(persisted.slam, persisted.healthy_ekf)

    assert comparison['outcome'] == 'NEGATIVE_OPTIMIZATION'
