"""Validate a legacy C920 calibration against diverse ChArUco observations."""

from dataclasses import dataclass
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


CHARUCO_SQUARES_X = 5
CHARUCO_SQUARES_Y = 7
CHARUCO_DICTIONARY_ID = cv2.aruco.DICT_5X5_100
CHARUCO_SQUARE_LENGTH_M = 0.0288
CHARUCO_MARKER_LENGTH_M = 0.0144
DEFAULT_CALIBRATION_FILE = '/tmp/camera_params_old.yaml'
IMAGE_TOPIC = '/camera/c920/image_raw'


@dataclass(frozen=True)
class CoverageMetrics:
    """Describe the image-plane coverage of one ChArUco observation."""

    x: float
    y: float
    size: float
    skew: float


@dataclass(frozen=True)
class AcceptedSample:
    """Store one accepted pose's coverage and holdout reprojection error."""

    coverage: CoverageMetrics
    holdout_rmse_px: float


def make_charuco_board():
    """Create the fixed 5x7 DICT_5X5_100 ChArUco board in metres."""
    dictionary = cv2.aruco.getPredefinedDictionary(CHARUCO_DICTIONARY_ID)
    return cv2.aruco.CharucoBoard_create(
        CHARUCO_SQUARES_X,
        CHARUCO_SQUARES_Y,
        CHARUCO_SQUARE_LENGTH_M,
        CHARUCO_MARKER_LENGTH_M,
        dictionary,
    ), dictionary


def read_legacy_calibration(path):
    """Read and validate camera_matrix and dist_coeffs without writing them."""
    storage = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise RuntimeError(f'Cannot open legacy calibration file: {path}')
    try:
        camera_matrix = storage.getNode('camera_matrix').mat()
        dist_coeffs = storage.getNode('dist_coeffs').mat()
    finally:
        storage.release()

    if camera_matrix is None or camera_matrix.shape != (3, 3):
        raise RuntimeError('Legacy camera_matrix must be a 3x3 OpenCV matrix.')
    if dist_coeffs is None or dist_coeffs.size == 0:
        raise RuntimeError('Legacy dist_coeffs must be a non-empty OpenCV matrix.')
    return (
        np.asarray(camera_matrix, dtype=np.float64),
        np.asarray(dist_coeffs, dtype=np.float64).reshape(-1, 1),
    )


def split_pose_fit_and_holdout(charuco_ids, charuco_corners):
    """Split sorted corner IDs deterministically into alternating fit/holdout sets."""
    ids = np.asarray(charuco_ids, dtype=np.int32).reshape(-1)
    corners = np.asarray(charuco_corners, dtype=np.float32).reshape(-1, 2)
    if ids.size != corners.shape[0]:
        raise ValueError('ChArUco IDs and corners must have the same length.')
    order = np.argsort(ids, kind='stable')
    ids = ids[order]
    corners = corners[order]
    return (
        ids[::2], corners[::2],
        ids[1::2], corners[1::2],
    )


def coverage_metrics(corners, image_width, image_height):
    """Calculate normalized centre, scale and perspective-skew coverage metrics."""
    points = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    if points.shape[0] < 4:
        raise ValueError('At least four corners are required for coverage metrics.')
    centre = np.mean(points, axis=0)
    hull = cv2.convexHull(points.astype(np.float32)).reshape(-1, 2)
    hull_area = float(cv2.contourArea(hull))
    image_area = float(image_width * image_height)
    if image_area <= 0.0:
        raise ValueError('Image dimensions must be positive.')
    distances = np.linalg.norm(points - centre, axis=1)
    mean_radius = float(np.mean(distances))
    radius_spread = float(np.std(distances))
    return CoverageMetrics(
        x=float(centre[0] / image_width),
        y=float(centre[1] / image_height),
        size=float(math.sqrt(max(hull_area, 0.0) / image_area)),
        skew=float(radius_spread / mean_radius) if mean_radius > 0.0 else 0.0,
    )


def median_common_corner_motion(previous_ids, previous_corners, ids, corners):
    """Return median motion in pixels over corresponding ChArUco IDs, if possible."""
    previous = {
        int(corner_id): point
        for corner_id, point in zip(np.asarray(previous_ids).reshape(-1), previous_corners)
    }
    current = {
        int(corner_id): point
        for corner_id, point in zip(np.asarray(ids).reshape(-1), corners)
    }
    common_ids = sorted(set(previous).intersection(current))
    if not common_ids:
        return None, 0
    motions = [
        np.linalg.norm(current[corner_id] - previous[corner_id])
        for corner_id in common_ids
    ]
    return float(np.median(motions)), len(common_ids)


def is_coverage_duplicate(candidate, accepted, minimum_differences):
    """Return true only when every requested coverage dimension is too similar."""
    return any(
        abs(candidate.x - sample.coverage.x) < minimum_differences.x
        and abs(candidate.y - sample.coverage.y) < minimum_differences.y
        and abs(candidate.size - sample.coverage.size) < minimum_differences.size
        and abs(candidate.skew - sample.coverage.skew) < minimum_differences.skew
        for sample in accepted
    )


def holdout_reprojection_rmse(
        board, fit_ids, fit_corners, holdout_ids, holdout_corners,
        camera_matrix, dist_coeffs):
    """Fit pose with one deterministic group and score only the other group."""
    object_points = np.asarray(board.chessboardCorners, dtype=np.float64)
    fit_object_points = object_points[np.asarray(fit_ids, dtype=np.int32)]
    holdout_object_points = object_points[np.asarray(holdout_ids, dtype=np.int32)]
    solved, rotation_vector, translation_vector = cv2.solvePnP(
        fit_object_points,
        np.asarray(fit_corners, dtype=np.float64),
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not solved:
        return None
    projected, _ = cv2.projectPoints(
        holdout_object_points,
        rotation_vector,
        translation_vector,
        camera_matrix,
        dist_coeffs,
    )
    errors = projected.reshape(-1, 2) - np.asarray(holdout_corners).reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum(np.square(errors), axis=1))))


def format_coverage(coverage):
    """Format coverage values consistently for runtime progress reports."""
    return (
        f'X={coverage.x:.3f} Y={coverage.y:.3f} '
        f'Size={coverage.size:.3f} Skew={coverage.skew:.3f}'
    )


class CalibrationReuseValidator(Node):
    """Collect diverse ChArUco poses and score only holdout reprojection points."""

    def __init__(self):
        """Load immutable legacy parameters and subscribe to the C920 raw image."""
        super().__init__('calibration_reuse_validator')
        self.declare_parameter('calibration_file', DEFAULT_CALIBRATION_FILE)
        self.declare_parameter('min_charuco_corners', 12)
        self.declare_parameter('min_motion_common_corners', 6)
        self.declare_parameter('max_interframe_corner_motion_px', 80.0)
        self.declare_parameter('min_x_difference', 0.08)
        self.declare_parameter('min_y_difference', 0.08)
        self.declare_parameter('min_size_difference', 0.06)
        self.declare_parameter('min_skew_difference', 0.05)
        self.declare_parameter('target_accepted_samples', 40)

        self._minimum_corners = self.get_parameter(
            'min_charuco_corners').value
        self._minimum_motion_common_corners = self.get_parameter(
            'min_motion_common_corners').value
        self._maximum_motion_px = self.get_parameter(
            'max_interframe_corner_motion_px').value
        self._minimum_differences = CoverageMetrics(
            x=self.get_parameter('min_x_difference').value,
            y=self.get_parameter('min_y_difference').value,
            size=self.get_parameter('min_size_difference').value,
            skew=self.get_parameter('min_skew_difference').value,
        )
        self._target_samples = self.get_parameter('target_accepted_samples').value
        if self._minimum_corners < 8:
            raise ValueError('min_charuco_corners must be at least 8 for both sets.')
        if self._target_samples < 1:
            raise ValueError('target_accepted_samples must be at least 1.')

        calibration_file = self.get_parameter('calibration_file').value
        self._camera_matrix, self._dist_coeffs = read_legacy_calibration(
            calibration_file)
        self._board, self._dictionary = make_charuco_board()
        self._detector_parameters = cv2.aruco.DetectorParameters_create()
        self._accepted_samples = []
        self._previous_ids = None
        self._previous_corners = None
        self._last_rejection_log_ns = 0
        self._summary_reported = False
        self.create_subscription(
            Image, IMAGE_TOPIC, self._image_callback, qos_profile_sensor_data)
        self.get_logger().info(
            f'Validating legacy K/D from {calibration_file!r}; '
            f'collecting {self._target_samples} diverse ChArUco poses.')

    def _log_rejection(self, reason):
        """Report rejection reasons at most twice per second."""
        now_ns = time.monotonic_ns()
        if now_ns - self._last_rejection_log_ns >= 500_000_000:
            self.get_logger().debug(f'ChArUco sample rejected: {reason}')
            self._last_rejection_log_ns = now_ns

    def _image_to_gray(self, image):
        """Convert the C920 rgb8/bgr8/mono8 image buffer to grayscale."""
        if image.height == 0 or image.width == 0:
            return None
        encoding = image.encoding.lower()
        channels = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}.get(encoding)
        if channels is None:
            self._log_rejection(f'unsupported encoding {image.encoding!r}')
            return None
        required_row_bytes = image.width * channels
        if image.step < required_row_bytes or len(image.data) < image.height * image.step:
            self._log_rejection('malformed image buffer')
            return None
        rows = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.step)
        pixels = rows[:, :required_row_bytes]
        if channels == 1:
            return pixels.reshape(image.height, image.width)
        colour = pixels.reshape(image.height, image.width, channels)
        conversion = cv2.COLOR_RGB2GRAY if encoding == 'rgb8' else cv2.COLOR_BGR2GRAY
        return cv2.cvtColor(colour, conversion)

    def _detect_charuco(self, grayscale):
        """Detect board corners using the OpenCV 4.6-compatible ArUco API."""
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
            grayscale, self._dictionary, parameters=self._detector_parameters)
        if marker_ids is None or len(marker_ids) == 0:
            return None, None
        count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            marker_corners, marker_ids, grayscale, self._board)
        if count is None or count <= 0 or charuco_ids is None:
            return None, None
        return charuco_ids.reshape(-1), charuco_corners.reshape(-1, 2)

    def _image_callback(self, image):
        """Filter one frame and append a holdout-only calibration validation sample."""
        grayscale = self._image_to_gray(image)
        if grayscale is None:
            return
        corner_ids, corners = self._detect_charuco(grayscale)
        if corner_ids is None or len(corner_ids) < self._minimum_corners:
            self._log_rejection('too few ChArUco corners')
            return

        if self._previous_ids is not None:
            motion_px, common_count = median_common_corner_motion(
                self._previous_ids, self._previous_corners, corner_ids, corners)
            if (
                common_count >= self._minimum_motion_common_corners
                and motion_px > self._maximum_motion_px
            ):
                self._previous_ids = corner_ids.copy()
                self._previous_corners = corners.copy()
                self._log_rejection(
                    f'interframe motion {motion_px:.1f}px exceeds '
                    f'{self._maximum_motion_px:.1f}px')
                return
        self._previous_ids = corner_ids.copy()
        self._previous_corners = corners.copy()

        coverage = coverage_metrics(corners, image.width, image.height)
        if is_coverage_duplicate(
                coverage, self._accepted_samples, self._minimum_differences):
            self._log_rejection('coverage duplicates an accepted pose')
            return
        fit_ids, fit_corners, holdout_ids, holdout_corners = split_pose_fit_and_holdout(
            corner_ids, corners)
        if len(fit_ids) < 4 or len(holdout_ids) < 4:
            self._log_rejection('insufficient deterministic pose-fit/holdout corners')
            return
        rmse_px = holdout_reprojection_rmse(
            self._board,
            fit_ids,
            fit_corners,
            holdout_ids,
            holdout_corners,
            self._camera_matrix,
            self._dist_coeffs,
        )
        if rmse_px is None:
            self._log_rejection('solvePnP failed')
            return

        self._accepted_samples.append(AcceptedSample(coverage, rmse_px))
        self.get_logger().info(
            f'accepted={len(self._accepted_samples)}/{self._target_samples} '
            f'holdout_rmse_px={rmse_px:.3f} {format_coverage(coverage)}')
        if len(self._accepted_samples) >= self._target_samples:
            self.report_summary()
            self.get_logger().info('Target accepted-pose count reached; stopping validator.')
            rclpy.shutdown()

    def report_summary(self):
        """Report accepted count, holdout RMSE quantiles, and coverage ranges once."""
        if self._summary_reported:
            return
        self._summary_reported = True
        if not self._accepted_samples:
            self.get_logger().info('Calibration reuse summary: accepted=0; no holdout RMSE.')
            return
        rmses = np.array(
            [sample.holdout_rmse_px for sample in self._accepted_samples], dtype=float)
        coverage = np.array(
            [[
                sample.coverage.x,
                sample.coverage.y,
                sample.coverage.size,
                sample.coverage.skew,
            ] for sample in self._accepted_samples],
            dtype=float,
        )
        ranges = [
            f'{label}=[{coverage[:, index].min():.3f},{coverage[:, index].max():.3f}]'
            for index, label in enumerate(('X', 'Y', 'Size', 'Skew'))
        ]
        self.get_logger().info(
            'Calibration reuse summary: '
            f'accepted={len(self._accepted_samples)} '
            f'median_holdout_rmse_px={np.median(rmses):.3f} '
            f'mean_holdout_rmse_px={np.mean(rmses):.3f} '
            f'p95_holdout_rmse_px={np.percentile(rmses, 95):.3f} '
            f'max_holdout_rmse_px={np.max(rmses):.3f} '
            + ' '.join(ranges))


def main(args=None):
    """Run the temporary legacy-calibration reuse validator."""
    rclpy.init(args=args)
    node = CalibrationReuseValidator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
