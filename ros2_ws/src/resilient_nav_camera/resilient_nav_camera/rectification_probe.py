"""Monitor C920 raw, rectified and CameraInfo metadata without image processing."""

from collections import deque
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from resilient_nav_camera.c920_probe import FrameTimingStats
from sensor_msgs.msg import CameraInfo, Image


EXPECTED_WIDTH = 1280
EXPECTED_HEIGHT = 720
EXPECTED_FRAME_ID = 'c920_camera_optical_frame'
EXPECTED_DISTORTION_MODEL = 'plumb_bob'
EXPECTED_CAMERA_MATRIX = [
    932.82988067562258, 0.0, 637.24258737387333,
    0.0, 931.57268412771748, 373.17256990919134,
    0.0, 0.0, 1.0,
]
EXPECTED_DISTORTION_COEFFICIENTS = [
    0.022262962552999279, -0.038141806307307465,
    0.002411186067623861, -0.0038095868601148979,
    -0.20962046543174492,
]
EXPECTED_RECTIFICATION_MATRIX = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
EXPECTED_PROJECTION_MATRIX = [
    932.82988067562258, 0.0, 637.24258737387333, 0.0,
    0.0, 931.57268412771748, 373.17256990919134, 0.0,
    0.0, 0.0, 1.0, 0.0,
]
SUMMARY_PERIOD_NS = 5_000_000_000
STAMP_HISTORY_SIZE = 100


def header_stamp_ns(header):
    """Return a ROS header stamp as nanoseconds."""
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def image_metadata_issues(image):
    """Check the expected image dimensions, frame ID and nonzero timestamp."""
    issues = []
    if image.width != EXPECTED_WIDTH:
        issues.append(f'width={image.width} (expected {EXPECTED_WIDTH})')
    if image.height != EXPECTED_HEIGHT:
        issues.append(f'height={image.height} (expected {EXPECTED_HEIGHT})')
    if image.header.frame_id != EXPECTED_FRAME_ID:
        issues.append(
            f'frame_id={image.header.frame_id!r} (expected {EXPECTED_FRAME_ID!r})')
    if header_stamp_ns(image.header) == 0:
        issues.append('header.stamp is zero')
    return issues


def camera_info_issues(camera_info):
    """Check that CameraInfo matches the package's immutable formal YAML values."""
    issues = []
    if camera_info.width != EXPECTED_WIDTH:
        issues.append(f'width={camera_info.width} (expected {EXPECTED_WIDTH})')
    if camera_info.height != EXPECTED_HEIGHT:
        issues.append(f'height={camera_info.height} (expected {EXPECTED_HEIGHT})')
    if camera_info.distortion_model != EXPECTED_DISTORTION_MODEL:
        issues.append(
            f'distortion_model={camera_info.distortion_model!r} '
            f'(expected {EXPECTED_DISTORTION_MODEL!r})')
    if camera_info.header.frame_id != EXPECTED_FRAME_ID:
        issues.append(
            f'frame_id={camera_info.header.frame_id!r} '
            f'(expected {EXPECTED_FRAME_ID!r})')
    if header_stamp_ns(camera_info.header) == 0:
        issues.append('header.stamp is zero')
    for field, actual, expected in [
        ('K', camera_info.k, EXPECTED_CAMERA_MATRIX),
        ('D', camera_info.d, EXPECTED_DISTORTION_COEFFICIENTS),
        ('R', camera_info.r, EXPECTED_RECTIFICATION_MATRIX),
        ('P', camera_info.p, EXPECTED_PROJECTION_MATRIX),
    ]:
        if len(actual) != len(expected) or not np.allclose(
                actual, expected, rtol=0.0, atol=1.0e-12):
            issues.append(f'{field} differs from c920_camera_info.yaml')
    return issues


class MatchedStampTracker:
    """Count raw/rect image pairs whose header timestamps are identical."""

    def __init__(self, history_size=STAMP_HISTORY_SIZE):
        """Initialize bounded raw and rect timestamp histories."""
        self._history_size = history_size
        self._raw_stamps = deque()
        self._rect_stamps = deque()
        self.matched_pair_count = 0

    def add_raw(self, stamp_ns):
        """Store one raw timestamp and count a matching pending rect timestamp."""
        self._add(stamp_ns, self._raw_stamps, self._rect_stamps)

    def add_rect(self, stamp_ns):
        """Store one rect timestamp and count a matching pending raw timestamp."""
        self._add(stamp_ns, self._rect_stamps, self._raw_stamps)

    def _add(self, stamp_ns, own_stamps, other_stamps):
        """Match a timestamp if possible, otherwise retain bounded history."""
        try:
            other_stamps.remove(stamp_ns)
        except ValueError:
            own_stamps.append(stamp_ns)
            if len(own_stamps) > self._history_size:
                own_stamps.popleft()
        else:
            self.matched_pair_count += 1


class RectificationProbe(Node):
    """Observe metadata and rate consistency of the optional C920 rectification chain."""

    def __init__(self):
        """Subscribe to raw, rectified and CameraInfo topics without reading pixels."""
        super().__init__('rectification_probe')
        self._raw_timing = FrameTimingStats()
        self._rect_timing = FrameTimingStats()
        self._stamp_tracker = MatchedStampTracker()
        self._raw_count = 0
        self._rect_count = 0
        self._camera_info_count = 0
        self._raw_mismatch_count = 0
        self._rect_mismatch_count = 0
        self._camera_info_mismatch_count = 0
        self._last_summary_ns = time.monotonic_ns()
        self.create_subscription(
            Image,
            '/camera/c920/image_raw',
            self._raw_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            '/camera/c920/image_rect',
            self._rect_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            '/camera/c920/camera_info',
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0, self._maybe_log_summary)

    def _raw_callback(self, image):
        """Check raw metadata and reception timing without accessing image.data."""
        self._raw_count += 1
        self._raw_timing.add_receive_time(time.monotonic_ns())
        self._stamp_tracker.add_raw(header_stamp_ns(image.header))
        self._log_image_issues('raw', image, is_rectified=False)

    def _rect_callback(self, image):
        """Check rectified metadata and reception timing without accessing image.data."""
        self._rect_count += 1
        self._rect_timing.add_receive_time(time.monotonic_ns())
        self._stamp_tracker.add_rect(header_stamp_ns(image.header))
        self._log_image_issues('rect', image, is_rectified=True)

    def _camera_info_callback(self, camera_info):
        """Check loaded calibration metadata against the formal package resource."""
        self._camera_info_count += 1
        issues = camera_info_issues(camera_info)
        if issues:
            self._camera_info_mismatch_count += 1
            self.get_logger().warning('CameraInfo check failed: ' + '; '.join(issues))

    def _log_image_issues(self, stream_name, image, is_rectified):
        """Record metadata failures while keeping raw and rect counters separate."""
        issues = image_metadata_issues(image)
        if not issues:
            return
        if is_rectified:
            self._rect_mismatch_count += 1
        else:
            self._raw_mismatch_count += 1
        self.get_logger().warning(
            f'{stream_name} image metadata check failed: ' + '; '.join(issues))

    def _maybe_log_summary(self):
        """Log image rates and metadata-health counts every five seconds."""
        now_ns = time.monotonic_ns()
        if now_ns - self._last_summary_ns < SUMMARY_PERIOD_NS:
            return
        self._last_summary_ns = now_ns
        raw_rate = self._format_rate(self._raw_timing.summary())
        rect_rate = self._format_rate(self._rect_timing.summary())
        self.get_logger().info(
            'C920 rectification summary: '
            f'raw_frames={self._raw_count} raw_rate_hz={raw_rate} '
            f'rect_frames={self._rect_count} rect_rate_hz={rect_rate} '
            f'camera_info_messages={self._camera_info_count} '
            f'matched_raw_rect_stamps={self._stamp_tracker.matched_pair_count} '
            f'raw_metadata_mismatches={self._raw_mismatch_count} '
            f'rect_metadata_mismatches={self._rect_mismatch_count} '
            f'camera_info_mismatches={self._camera_info_mismatch_count}')

    @staticmethod
    def _format_rate(summary):
        """Format a frame-rate summary while handling fewer than two messages."""
        if summary is None:
            return 'n/a'
        return f'{summary["frame_rate_hz"]:.3f}'


def main(args=None):
    """Run the C920 rectification metadata and frame-rate probe."""
    rclpy.init(args=args)
    node = RectificationProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
