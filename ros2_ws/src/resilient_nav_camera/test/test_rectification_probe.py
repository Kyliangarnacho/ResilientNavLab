"""Unit tests for C920 raw/rectified CameraInfo metadata checks."""

from builtin_interfaces.msg import Time
from resilient_nav_camera.rectification_probe import (
    camera_info_issues,
    EXPECTED_CAMERA_MATRIX,
    EXPECTED_DISTORTION_COEFFICIENTS,
    EXPECTED_DISTORTION_MODEL,
    EXPECTED_FRAME_ID,
    EXPECTED_HEIGHT,
    EXPECTED_PROJECTION_MATRIX,
    EXPECTED_RECTIFICATION_MATRIX,
    EXPECTED_WIDTH,
    image_metadata_issues,
    MatchedStampTracker,
)
from sensor_msgs.msg import CameraInfo, Image


def make_image(stamp_ns, **overrides):
    """Return Image metadata matching the expected C920 raw/rect contract."""
    image = Image()
    image.width = overrides.get('width', EXPECTED_WIDTH)
    image.height = overrides.get('height', EXPECTED_HEIGHT)
    image.header.frame_id = overrides.get('frame_id', EXPECTED_FRAME_ID)
    image.header.stamp = Time(
        sec=stamp_ns // 1_000_000_000,
        nanosec=stamp_ns % 1_000_000_000,
    )
    return image


def make_camera_info(stamp_ns, **overrides):
    """Return CameraInfo metadata matching the formal installed C920 resource."""
    camera_info = CameraInfo()
    camera_info.width = overrides.get('width', EXPECTED_WIDTH)
    camera_info.height = overrides.get('height', EXPECTED_HEIGHT)
    camera_info.distortion_model = overrides.get(
        'distortion_model', EXPECTED_DISTORTION_MODEL)
    camera_info.k = overrides.get('k', EXPECTED_CAMERA_MATRIX)
    camera_info.d = overrides.get('d', EXPECTED_DISTORTION_COEFFICIENTS)
    camera_info.r = overrides.get('r', EXPECTED_RECTIFICATION_MATRIX)
    camera_info.p = overrides.get('p', EXPECTED_PROJECTION_MATRIX)
    camera_info.header.frame_id = overrides.get('frame_id', EXPECTED_FRAME_ID)
    camera_info.header.stamp = Time(
        sec=stamp_ns // 1_000_000_000,
        nanosec=stamp_ns % 1_000_000_000,
    )
    return camera_info


def test_image_metadata_accepts_expected_raw_and_rect_headers():
    """Both image streams require matching dimensions, frame ID and timestamps."""
    assert image_metadata_issues(make_image(1_000_000_000)) == []


def test_image_metadata_reports_dimension_frame_and_stamp_mismatches():
    """Image checks describe all expected fields without accessing payload data."""
    issues = image_metadata_issues(make_image(0, width=640, frame_id='wrong_frame'))

    assert any(issue.startswith('width=') for issue in issues)
    assert any(issue.startswith('frame_id=') for issue in issues)
    assert 'header.stamp is zero' in issues


def test_camera_info_accepts_the_formal_legacy_calibration_values():
    """Ensure CameraInfo carries K/D/R/P and plumb_bob from the package YAML."""
    assert camera_info_issues(make_camera_info(1_000_000_000)) == []


def test_camera_info_reports_changed_calibration_values():
    """Changed distortion and K are reported instead of silently accepted."""
    issues = camera_info_issues(make_camera_info(
        1_000_000_000,
        distortion_model='equidistant',
        k=[0.0] * 9,
        d=[0.0] * 5,
    ))

    assert any(issue.startswith('distortion_model=') for issue in issues)
    assert 'K differs from c920_camera_info.yaml' in issues
    assert 'D differs from c920_camera_info.yaml' in issues


def test_stamp_tracker_matches_raw_and_rect_stamps_in_either_order():
    """Match timestamps without assuming raw/rect callback ordering."""
    tracker = MatchedStampTracker(history_size=2)

    tracker.add_rect(20)
    tracker.add_raw(20)
    tracker.add_raw(30)
    tracker.add_rect(30)

    assert tracker.matched_pair_count == 2
