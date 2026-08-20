"""Pure evaluation contracts for Phase 9 map-frame SLAM scoring."""

from math import cos, pi, sin

import pytest

from resilient_nav_fusion.localization_evaluator import EvaluationError, TimedPose
from resilient_nav_slam.slam_evaluator import (
    apply_se2_alignment,
    evaluate_slam_trajectory,
    initial_se2_alignment,
)


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


def test_initial_se2_alignment_maps_the_anchor_exactly():
    truth = TimedPose(1.0, 3.0, -2.0, 0.75)
    estimate = TimedPose(1.0, 1.0, 2.0, -0.5)

    aligned = apply_se2_alignment(estimate, initial_se2_alignment(truth, estimate))

    assert aligned.x == pytest.approx(truth.x)
    assert aligned.y == pytest.approx(truth.y)
    assert aligned.yaw == pytest.approx(truth.yaw)


def test_slam_metrics_remove_only_the_declared_initial_frame_offset():
    truth = _poses()
    translation_x = 10.0
    translation_y = -3.0
    yaw_offset = 0.6
    estimate = [
        TimedPose(
            pose.stamp_sec,
            cos(-yaw_offset) * (pose.x - translation_x)
            - sin(-yaw_offset) * (pose.y - translation_y),
            sin(-yaw_offset) * (pose.x - translation_x)
            + cos(-yaw_offset) * (pose.y - translation_y),
            pose.yaw - yaw_offset,
        )
        for pose in truth
    ]

    result = evaluate_slam_trajectory(truth, estimate)

    assert result.sample_count == 4
    assert result.position_rmse == pytest.approx(0.0)
    assert result.yaw_rmse == pytest.approx(0.0)
    assert result.return_to_start_position_error == pytest.approx(0.0)
    assert result.initial_alignment.yaw == pytest.approx(0.6)


def test_slam_metrics_keep_post_alignment_drift_and_return_error():
    truth = _poses()
    estimate = _poses(x_offset=2.0)
    estimate[-1] = TimedPose(3.0, 6.0, 0.75, 0.3)

    result = evaluate_slam_trajectory(truth, estimate)

    assert result.final_position_drift == pytest.approx(1.0)
    assert result.return_to_start_position_error == pytest.approx(1.0)


def test_yaw_wraparound_after_alignment_uses_shortest_distance():
    truth = [TimedPose(float(index), 0.0, 0.0, pi - 0.05) for index in range(4)]
    estimate = [TimedPose(float(index), 0.0, 0.0, -pi + 0.05) for index in range(4)]

    result = evaluate_slam_trajectory(truth, estimate)

    assert result.yaw_rmse == pytest.approx(0.0)
    assert result.max_yaw_error == pytest.approx(0.0)


def test_timestamp_pairing_reuses_fail_closed_phase8_semantics():
    with pytest.raises(EvaluationError, match='time-aligned'):
        evaluate_slam_trajectory(
            _poses(), _poses(time_offset=0.2), max_alignment_delta_sec=0.05
        )
