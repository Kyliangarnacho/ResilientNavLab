"""Tests for the pixel-identical camera freeze test source."""

from builtin_interfaces.msg import Time
import numpy as np
from resilient_nav_health_assessment.camera_freeze_source import (
    FrozenImageCache,
)
from sensor_msgs.msg import Image


class FakeBridge:
    """Return a configured NumPy image for conversion validation."""

    def __init__(self, image):
        self.image = image
        self.calls = 0

    def imgmsg_to_cv2(self, message, desired_encoding):
        self.calls += 1
        assert desired_encoding == 'passthrough'
        return self.image


def _source_image(value=0):
    pixels = (
        np.arange(18, dtype=np.uint8).reshape(2, 3, 3) + value
    )
    message = Image()
    message.header.stamp = Time(sec=10, nanosec=20)
    message.header.frame_id = 'c920_camera_optical_frame'
    message.height = 2
    message.width = 3
    message.encoding = 'rgb8'
    message.is_bigendian = 0
    message.step = 9
    message.data = pixels.tobytes()
    return message, pixels


def test_no_output_is_created_before_a_valid_source_image():
    cache = FrozenImageCache()

    assert cache.ready is False
    assert cache.make_message(Time(sec=1)) is None


def test_invalid_source_image_is_rejected_safely():
    cache = FrozenImageCache()
    message, pixels = _source_image()
    message.data = bytes(4)

    cached = cache.try_cache(message, FakeBridge(pixels))

    assert cached is False
    assert cache.ready is False
    assert 'shorter than step * height' in cache.last_error
    assert cache.make_message(Time(sec=2)) is None


def test_frozen_outputs_keep_exact_pixels_and_advance_stamp():
    cache = FrozenImageCache()
    source, pixels = _source_image()
    bridge = FakeBridge(pixels)

    assert cache.try_cache(source, bridge) is True
    first = cache.make_message(Time(sec=20, nanosec=100))
    second = cache.make_message(Time(sec=20, nanosec=200))

    assert bridge.calls == 1
    assert bytes(first.data) == bytes(source.data)
    assert bytes(second.data) == bytes(source.data)
    assert bytes(first.data) == bytes(second.data)
    assert first.header.stamp.nanosec == 100
    assert second.header.stamp.nanosec == 200
    assert second.header.stamp.nanosec > first.header.stamp.nanosec


def test_frozen_output_preserves_required_source_metadata():
    cache = FrozenImageCache()
    source, pixels = _source_image()
    assert cache.try_cache(source, FakeBridge(pixels)) is True

    output = cache.make_message(Time(sec=30))

    assert output.header.frame_id == source.header.frame_id
    assert output.encoding == source.encoding
    assert output.width == source.width
    assert output.height == source.height
    assert output.step == source.step
    assert output.is_bigendian == source.is_bigendian
    assert output.header.stamp != source.header.stamp


def test_first_valid_frame_remains_the_only_frozen_template():
    cache = FrozenImageCache()
    first_source, first_pixels = _source_image(0)
    second_source, second_pixels = _source_image(20)

    assert cache.try_cache(first_source, FakeBridge(first_pixels)) is True
    assert cache.try_cache(second_source, FakeBridge(second_pixels)) is False
    output = cache.make_message(Time(sec=40))

    assert bytes(output.data) == bytes(first_source.data)
    assert bytes(output.data) != bytes(second_source.data)
