"""Targeted contracts for pure, fail-closed localization evaluation."""

from math import pi

import pytest

from resilient_nav_fusion.localization_evaluator import (
    EvaluationError,
    TimedPose,
    evaluate_alignment_sensitivity,
    evaluate_trajectories,
)


def poses(*, x_offset=0.0, yaw_offset=0.0, time_offset=0.0):
    """Return three orderly planar samples with controlled error."""
    return [
        TimedPose(
            stamp_sec=float(index) + time_offset,
            x=float(index) + x_offset,
            y=0.0,
            yaw=0.2 * index + yaw_offset,
        )
        for index in range(3)
    ]


def test_exactly_overlapping_trajectories_have_zero_fixed_and_adaptive_error():
    """No drift is invented when both estimators equal ground truth."""
    truth = poses()
    result = evaluate_trajectories(truth, truth, truth)

    assert result.fixed.position_rmse == 0.0
    assert result.fixed.yaw_rmse == 0.0
    assert result.adaptive.position_rmse == 0.0
    assert result.adaptive.final_position_drift == 0.0
    assert result.position_rmse_benefit == 0.0
    assert result.position_rmse_improvement_ratio is None


def test_fixed_offset_reports_fixed_and_adaptive_metrics_and_benefit():
    """The result retains both metric sets and makes improvement explicit."""
    result = evaluate_trajectories(poses(), poses(x_offset=2.0), poses(x_offset=0.5))

    assert result.fixed.position_rmse == 2.0
    assert result.fixed.max_position_error == 2.0
    assert result.fixed.final_position_drift == 2.0
    assert result.adaptive.position_rmse == 0.5
    assert result.position_rmse_benefit == 1.5
    assert result.position_rmse_improvement_ratio == 0.75
    assert result.to_dict()['fixed']['sample_count'] == 3


def test_yaw_wraparound_uses_shortest_angular_distance():
    """Near-pi orientations must not appear nearly a full turn apart."""
    truth = [TimedPose(float(index), 0.0, 0.0, pi - 0.1) for index in range(3)]
    estimate = [TimedPose(float(index), 0.0, 0.0, -pi + 0.1) for index in range(3)]
    result = evaluate_trajectories(truth, estimate, estimate)

    assert result.fixed.yaw_rmse == pytest.approx(0.2)
    assert result.fixed.max_yaw_error == pytest.approx(0.2)
    assert result.fixed.final_yaw_drift == pytest.approx(0.2)


def test_insufficient_samples_fail_closed():
    """A single pose cannot be represented as a trajectory metric."""
    one = [TimedPose(1.0, 0.0, 0.0, 0.0)]

    with pytest.raises(EvaluationError, match='fewer than 3 samples'):
        evaluate_trajectories(one, one, one)


def test_unalignable_timestamps_fail_closed():
    """Metrics are withheld when a common time base cannot be established."""
    with pytest.raises(EvaluationError, match='time-aligned'):
        evaluate_trajectories(
            poses(),
            poses(time_offset=0.2),
            poses(time_offset=0.2),
            max_alignment_delta_sec=0.05,
        )


def test_alignment_sensitivity_withholds_only_the_too_narrow_window():
    """Narrow windows cannot borrow wider-window metrics or sample counts."""
    result = evaluate_alignment_sensitivity(
        poses(),
        poses(x_offset=2.0, time_offset=0.025),
        poses(x_offset=0.5, time_offset=0.025),
        windows_sec=(0.02, 0.03, 0.05),
    )

    assert result['0.020']['outcome'] == 'INSUFFICIENT_ALIGNED_SAMPLES'
    assert result['0.030']['outcome'] == 'PASS'
    assert result['0.030']['metrics']['fixed']['sample_count'] == 3
    assert result['0.030']['adaptive_improved'] is False
    assert result['0.050']['metrics']['adaptive']['position_rmse'] == 0.5
