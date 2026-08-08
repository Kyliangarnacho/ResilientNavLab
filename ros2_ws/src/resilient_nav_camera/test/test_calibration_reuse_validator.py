"""Unit tests for deterministic legacy-calibration validation helpers."""

import cv2
import numpy as np

from resilient_nav_camera.calibration_reuse_validator import (
    AcceptedSample,
    CHARUCO_SQUARES_X,
    CHARUCO_SQUARES_Y,
    coverage_metrics,
    CoverageMetrics,
    holdout_reprojection_rmse,
    is_coverage_duplicate,
    make_charuco_board,
    median_common_corner_motion,
    split_pose_fit_and_holdout,
)


def test_fixed_charuco_board_has_the_requested_geometry():
    """The validator fixes the requested 5x7 board definition in metres."""
    board, _ = make_charuco_board()

    assert CHARUCO_SQUARES_X == 5
    assert CHARUCO_SQUARES_Y == 7
    assert board.chessboardCorners.shape == (24, 3)


def test_split_is_deterministic_and_keeps_fit_and_holdout_disjoint():
    """Sorted alternating IDs provide reproducible pose-fit and holdout groups."""
    ids = np.array([8, 3, 4, 7, 1, 6, 2, 5], dtype=np.int32)
    corners = np.array([[corner_id, corner_id + 0.5] for corner_id in ids])

    fit_ids, fit_corners, holdout_ids, holdout_corners = split_pose_fit_and_holdout(
        ids, corners)

    assert fit_ids.tolist() == [1, 3, 5, 7]
    assert holdout_ids.tolist() == [2, 4, 6, 8]
    assert fit_corners[:, 0].tolist() == [1.0, 3.0, 5.0, 7.0]
    assert holdout_corners[:, 0].tolist() == [2.0, 4.0, 6.0, 8.0]


def test_motion_and_coverage_filters_are_measured_in_separate_domains():
    """Motion uses matching IDs while duplicate filtering uses all four metrics."""
    previous_ids = np.array([1, 2, 3], dtype=np.int32)
    previous = np.array([[10.0, 10.0], [20.0, 20.0], [30.0, 30.0]])
    current_ids = np.array([2, 3, 4], dtype=np.int32)
    current = np.array([[23.0, 24.0], [33.0, 34.0], [50.0, 50.0]])

    motion, common_count = median_common_corner_motion(
        previous_ids, previous, current_ids, current)
    metrics = coverage_metrics(
        np.array([[100.0, 100.0], [300.0, 100.0], [300.0, 300.0], [100.0, 300.0]]),
        640,
        480,
    )
    accepted = [AcceptedSample(metrics, 0.3)]

    assert common_count == 2
    assert motion == np.sqrt(25.0)
    assert is_coverage_duplicate(
        metrics, accepted, CoverageMetrics(0.01, 0.01, 0.01, 0.01))
    assert not is_coverage_duplicate(
        CoverageMetrics(metrics.x + 0.02, metrics.y, metrics.size, metrics.skew),
        accepted,
        CoverageMetrics(0.01, 0.01, 0.01, 0.01),
    )


def test_holdout_rmse_is_zero_for_synthetic_points_from_the_same_pose():
    """Pose fitting and holdout scoring use disjoint deterministic corner groups."""
    board, _ = make_charuco_board()
    camera_matrix = np.array(
        [[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]])
    distortion = np.zeros((5, 1))
    ids = np.arange(12, dtype=np.int32)
    projected, _ = cv2.projectPoints(
        board.chessboardCorners[ids],
        np.array([[0.1], [-0.2], [0.05]]),
        np.array([[0.01], [0.02], [1.2]]),
        camera_matrix,
        distortion,
    )
    fit_ids, fit_corners, holdout_ids, holdout_corners = split_pose_fit_and_holdout(
        ids, projected.reshape(-1, 2))

    rmse = holdout_reprojection_rmse(
        board,
        fit_ids,
        fit_corners,
        holdout_ids,
        holdout_corners,
        camera_matrix,
        distortion,
    )

    assert rmse < 1.0e-4
