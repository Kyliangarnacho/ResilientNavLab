"""Tests for classification-free camera health feature extraction."""

import numpy as np
import pytest

from resilient_nav_health_assessment.camera_health_features import (
    compute_camera_health_features,
)


FEATURE_KEYS = {
    'mean_gray',
    'gray_std',
    'p05',
    'p95',
    'dark_ratio',
    'bright_ratio',
    'laplacian_variance',
    'edge_density',
    'entropy',
    'frame_diff_mean',
    'frame_fingerprint',
}


def _checkerboard(size=64, cell_size=4):
    rows, columns = np.indices((size, size))
    return (((rows // cell_size + columns // cell_size) % 2) * 255).astype(
        np.uint8
    )


def _box_blur(image, kernel_size=7):
    radius = kernel_size // 2
    padded = np.pad(image.astype(np.float64), radius, mode='edge')
    blurred = np.zeros_like(image, dtype=np.float64)
    for row_offset in range(kernel_size):
        for column_offset in range(kernel_size):
            blurred += padded[
                row_offset:row_offset + image.shape[0],
                column_offset:column_offset + image.shape[1],
            ]
    return np.rint(blurred / kernel_size ** 2).astype(np.uint8)


def test_pure_black_image_features():
    features = compute_camera_health_features(np.zeros((32, 48), dtype=np.uint8))

    assert set(features) == FEATURE_KEYS
    assert features['mean_gray'] == 0.0
    assert features['gray_std'] == 0.0
    assert features['p05'] == 0.0
    assert features['p95'] == 0.0
    assert features['dark_ratio'] == 1.0
    assert features['bright_ratio'] == 0.0
    assert features['laplacian_variance'] == 0.0
    assert features['edge_density'] == 0.0
    assert features['entropy'] == 0.0
    assert features['frame_diff_mean'] is None
    assert len(features['frame_fingerprint']) == 64


def test_pure_white_image_features():
    features = compute_camera_health_features(
        np.full((32, 48), 255, dtype=np.uint8)
    )

    assert features['mean_gray'] == 255.0
    assert features['gray_std'] == 0.0
    assert features['dark_ratio'] == 0.0
    assert features['bright_ratio'] == 1.0
    assert features['laplacian_variance'] == 0.0
    assert features['edge_density'] == 0.0
    assert features['entropy'] == 0.0


def test_texture_has_spatial_detail_and_intensity_diversity():
    features = compute_camera_health_features(_checkerboard())

    assert features['gray_std'] > 100.0
    assert features['laplacian_variance'] > 0.0
    assert features['edge_density'] > 0.0
    assert features['entropy'] == pytest.approx(1.0)


def test_blur_reduces_checkerboard_laplacian_variance():
    texture = _checkerboard()
    sharp_features = compute_camera_health_features(texture)
    blurred_features = compute_camera_health_features(_box_blur(texture))

    assert blurred_features['laplacian_variance'] < (
        sharp_features['laplacian_variance']
    )


def test_low_texture_image_has_small_spread_and_no_edges():
    low_texture = np.tile(np.arange(100, 108, dtype=np.uint8), (64, 8))
    features = compute_camera_health_features(low_texture)

    assert features['gray_std'] < 3.0
    assert features['p95'] - features['p05'] < 10.0
    assert features['laplacian_variance'] < 20.0
    assert features['edge_density'] == 0.0


def test_repeated_frames_have_zero_difference_and_equal_fingerprints():
    frame = _checkerboard()
    first = compute_camera_health_features(frame)
    repeated = compute_camera_health_features(frame.copy(), frame)

    assert repeated['frame_diff_mean'] == 0.0
    assert repeated['frame_fingerprint'] == first['frame_fingerprint']


def test_different_frames_have_difference_and_distinct_fingerprints():
    first_frame = _checkerboard()
    second_frame = 255 - first_frame
    first = compute_camera_health_features(first_frame)
    second = compute_camera_health_features(second_frame, first_frame)

    assert second['frame_diff_mean'] == 255.0
    assert second['frame_fingerprint'] != first['frame_fingerprint']


@pytest.mark.parametrize(
    'image,channel_order,expected',
    [
        (np.full((8, 9, 1), 64, dtype=np.uint8), 'rgb', 64.0),
        (np.full((8, 9, 3), [255, 0, 0], dtype=np.uint8), 'rgb', 76.245),
        (np.full((8, 9, 3), [0, 0, 255], dtype=np.uint8), 'bgr', 76.245),
        (np.full((8, 9, 4), [255, 0, 0, 1], dtype=np.uint8), 'rgb', 76.245),
    ],
)
def test_common_gray_and_color_layouts(image, channel_order, expected):
    features = compute_camera_health_features(
        image, channel_order=channel_order
    )

    assert features['mean_gray'] == pytest.approx(expected)


def test_float_zero_to_one_images_are_scaled_to_gray_levels():
    image = np.full((8, 8), 0.5, dtype=np.float32)

    features = compute_camera_health_features(image)

    assert features['mean_gray'] == pytest.approx(127.5)


@pytest.mark.parametrize(
    'image,error_type,message',
    [
        ([[0]], TypeError, 'NumPy ndarray'),
        (np.empty((0, 2), dtype=np.uint8), ValueError, 'must not be empty'),
        (np.zeros((2,), dtype=np.uint8), ValueError, 'must have shape'),
        (np.zeros((2, 2, 2), dtype=np.uint8), ValueError, '1, 3, or 4 channels'),
        (np.array([[np.nan]]), ValueError, 'only finite values'),
        (np.array([[-1]], dtype=np.int16), ValueError, 'non-negative'),
        (np.array([[256]], dtype=np.uint16), ValueError, '0..1 or 0..255'),
        (np.array([[1 + 2j]]), TypeError, 'real numeric dtype'),
    ],
)
def test_invalid_images_raise_clear_errors(image, error_type, message):
    with pytest.raises(error_type, match=message):
        compute_camera_health_features(image)


def test_invalid_channel_order_and_previous_shape_raise_clear_errors():
    color = np.zeros((8, 8, 3), dtype=np.uint8)
    with pytest.raises(ValueError, match='channel_order'):
        compute_camera_health_features(color, channel_order='xyz')

    with pytest.raises(ValueError, match='height and width'):
        compute_camera_health_features(
            np.zeros((8, 8), dtype=np.uint8),
            np.zeros((7, 8), dtype=np.uint8),
        )
