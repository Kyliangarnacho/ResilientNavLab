"""Report C920 image-stream timing and metadata without reading image payloads."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


NANOSECONDS_PER_SECOND = 1_000_000_000
EXPECTED_WIDTH = 1280
EXPECTED_HEIGHT = 720
EXPECTED_ENCODING = 'rgb8'
EXPECTED_STEP = EXPECTED_WIDTH * 3
EXPECTED_FRAME_ID = 'c920_camera_optical_frame'
SUMMARY_PERIOD_NS = 5 * NANOSECONDS_PER_SECOND


class FrameTimingStats:
    """Accumulate receive-time intervals with constant memory use."""

    def __init__(self):
        """Initialize an empty interval accumulator."""
        self.previous_receive_ns = None
        self.interval_count = 0
        self.interval_sum_ns = 0
        self.min_interval_ns = None
        self.max_interval_ns = None

    def add_receive_time(self, receive_ns):
        """Store one reception time and return its interval, if any."""
        previous_receive_ns = self.previous_receive_ns
        self.previous_receive_ns = receive_ns
        if previous_receive_ns is None:
            return None

        interval_ns = receive_ns - previous_receive_ns
        self.interval_count += 1
        self.interval_sum_ns += interval_ns
        if self.min_interval_ns is None or interval_ns < self.min_interval_ns:
            self.min_interval_ns = interval_ns
        if self.max_interval_ns is None or interval_ns > self.max_interval_ns:
            self.max_interval_ns = interval_ns
        return interval_ns

    def summary(self):
        """Return frame-rate and interval metrics, or ``None`` before two frames."""
        if self.interval_count == 0:
            return None

        average_interval_ns = self.interval_sum_ns / self.interval_count
        return {
            'frame_rate_hz': NANOSECONDS_PER_SECOND / average_interval_ns,
            'average_interval_ms': average_interval_ns / 1_000_000,
            'min_interval_ms': self.min_interval_ns / 1_000_000,
            'max_interval_ms': self.max_interval_ns / 1_000_000,
        }


class ImageMetadataChecker:
    """Check the configured image contract and monotonic header stamps."""

    def __init__(self):
        """Initialize expected metadata and header-stamp tracking."""
        self.previous_stamp_ns = None
        self.metadata_mismatch_count = 0
        self.stamp_regression_count = 0
        self.last_issues = []

    def check(self, image):
        """Check image metadata only; image payload bytes are deliberately unused."""
        issues = []
        expected_fields = {
            'width': EXPECTED_WIDTH,
            'height': EXPECTED_HEIGHT,
            'encoding': EXPECTED_ENCODING,
            'step': EXPECTED_STEP,
            'frame_id': EXPECTED_FRAME_ID,
        }
        actual_fields = {
            'width': image.width,
            'height': image.height,
            'encoding': image.encoding,
            'step': image.step,
            'frame_id': image.header.frame_id,
        }
        for field, expected_value in expected_fields.items():
            actual_value = actual_fields[field]
            if actual_value != expected_value:
                issues.append(
                    f'{field}={actual_value!r} (expected {expected_value!r})')

        stamp_ns = (
            image.header.stamp.sec * NANOSECONDS_PER_SECOND
            + image.header.stamp.nanosec
        )
        if (
            self.previous_stamp_ns is not None
            and stamp_ns < self.previous_stamp_ns
        ):
            self.stamp_regression_count += 1
            issues.append(
                f'header.stamp regressed from {self.previous_stamp_ns} to {stamp_ns}')
        self.previous_stamp_ns = stamp_ns

        if issues:
            self.metadata_mismatch_count += 1
        self.last_issues = issues
        return issues, stamp_ns


class C920Probe(Node):
    """Monitor C920 reception timing and Image header metadata."""

    def __init__(self):
        """Subscribe to the C920 raw-image topic and initialize reporting state."""
        super().__init__('c920_probe')
        self._timing_stats = FrameTimingStats()
        self._metadata_checker = ImageMetadataChecker()
        self._frame_count = 0
        self._last_image_metadata = None
        self._last_summary_ns = time.monotonic_ns()
        self.create_subscription(
            Image,
            '/camera/c920/image_raw',
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0, self._maybe_log_summary)

    def _image_callback(self, image):
        """Update timing and metadata checks without decoding the image data."""
        receive_ns = time.monotonic_ns()
        self._timing_stats.add_receive_time(receive_ns)
        issues, stamp_ns = self._metadata_checker.check(image)
        self._frame_count += 1
        self._last_image_metadata = {
            'width': image.width,
            'height': image.height,
            'encoding': image.encoding,
            'step': image.step,
            'frame_id': image.header.frame_id,
            'stamp_ns': stamp_ns,
        }
        if issues:
            self.get_logger().warning('C920 image metadata check failed: ' + '; '.join(issues))

    def _maybe_log_summary(self):
        """Log a wall-clock summary every five seconds."""
        now_ns = time.monotonic_ns()
        if now_ns - self._last_summary_ns < SUMMARY_PERIOD_NS:
            return
        self._last_summary_ns = now_ns

        timing = self._timing_stats.summary()
        if timing is None or self._last_image_metadata is None:
            self.get_logger().info(
                'C920 summary: no complete frame interval received in the last 5 s; '
                f'frames_total={self._frame_count}')
            return

        metadata = self._last_image_metadata
        frame_rate_hz = timing['frame_rate_hz']
        average_interval_ms = timing['average_interval_ms']
        min_interval_ms = timing['min_interval_ms']
        max_interval_ms = timing['max_interval_ms']
        width = metadata['width']
        height = metadata['height']
        encoding = metadata['encoding']
        step = metadata['step']
        frame_id = metadata['frame_id']
        stamp_ns = metadata['stamp_ns']
        self.get_logger().info(
            'C920 summary: '
            f'frames_total={self._frame_count} '
            f'receive_rate_hz={frame_rate_hz:.3f} '
            f'interval_ms(avg/min/max)={average_interval_ms:.3f}/'
            f'{min_interval_ms:.3f}/{max_interval_ms:.3f} '
            f'size={width}x{height} '
            f'encoding={encoding!r} step={step} '
            f'frame_id={frame_id!r} '
            f'header_stamp_ns={stamp_ns} '
            f'metadata_mismatches={self._metadata_checker.metadata_mismatch_count} '
            f'stamp_regressions={self._metadata_checker.stamp_regression_count}')


def main(args=None):
    """Run the C920 metadata-only probe."""
    rclpy.init(args=args)
    node = C920Probe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
