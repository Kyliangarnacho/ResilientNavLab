"""Unit tests for C920 timing and metadata checks."""

from builtin_interfaces.msg import Time
from resilient_nav_camera.c920_probe import (
    EXPECTED_ENCODING,
    EXPECTED_FRAME_ID,
    EXPECTED_HEIGHT,
    EXPECTED_STEP,
    EXPECTED_WIDTH,
    FrameTimingStats,
    ImageMetadataChecker,
)
from sensor_msgs.msg import Image


def make_image(stamp_ns, **overrides):
    """Return an Image carrying baseline C920 metadata without payload bytes."""
    image = Image()
    image.width = overrides.get('width', EXPECTED_WIDTH)
    image.height = overrides.get('height', EXPECTED_HEIGHT)
    image.encoding = overrides.get('encoding', EXPECTED_ENCODING)
    image.step = overrides.get('step', EXPECTED_STEP)
    image.header.frame_id = overrides.get('frame_id', EXPECTED_FRAME_ID)
    image.header.stamp = Time(
        sec=stamp_ns // 1_000_000_000,
        nanosec=stamp_ns % 1_000_000_000,
    )
    return image


def test_timing_summary_uses_monotonic_receive_intervals():
    """The accumulator reports rate and interval extrema from receive times."""
    stats = FrameTimingStats()

    assert stats.add_receive_time(1_000_000_000) is None
    assert stats.add_receive_time(1_050_000_000) == 50_000_000
    assert stats.add_receive_time(1_150_000_000) == 100_000_000

    summary = stats.summary()
    assert summary['frame_rate_hz'] == 13.333333333333334
    assert summary['average_interval_ms'] == 75.0
    assert summary['min_interval_ms'] == 50.0
    assert summary['max_interval_ms'] == 100.0


def test_metadata_checker_accepts_the_configured_c920_contract():
    """Baseline metadata passes and does not inspect or require image data."""
    checker = ImageMetadataChecker()

    issues, stamp_ns = checker.check(make_image(1_234_000_567))

    assert issues == []
    assert stamp_ns == 1_234_000_567
    assert checker.metadata_mismatch_count == 0
    assert checker.stamp_regression_count == 0


def test_metadata_checker_reports_field_mismatches_and_stamp_regressions():
    """Unexpected metadata and decreasing header stamps are counted separately."""
    checker = ImageMetadataChecker()
    checker.check(make_image(2_000_000_000))

    issues, _ = checker.check(make_image(
        1_900_000_000,
        width=640,
        encoding='mono8',
        step=640,
        frame_id='unexpected_frame',
    ))

    assert any(issue.startswith('width=') for issue in issues)
    assert any(issue.startswith('encoding=') for issue in issues)
    assert any(issue.startswith('step=') for issue in issues)
    assert any(issue.startswith('frame_id=') for issue in issues)
    assert any(issue.startswith('header.stamp regressed') for issue in issues)
    assert checker.metadata_mismatch_count == 1
    assert checker.stamp_regression_count == 1
