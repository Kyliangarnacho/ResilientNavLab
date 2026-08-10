"""Extract classification-free camera health features from NumPy images."""

from hashlib import sha256

import numpy as np


_DARK_LEVEL = 0.10 * 255.0
_BRIGHT_LEVEL = 0.90 * 255.0
_EDGE_GRADIENT = 0.10 * 255.0
_RGB_WEIGHTS = np.array([0.299, 0.587, 0.114], dtype=np.float64)


def _validate_and_scale(image, name):
    """Return a finite numeric image scaled to the conventional 0..255 range."""
    if not isinstance(image, np.ndarray):
        raise TypeError(f'{name} must be a NumPy ndarray')
    if image.size == 0:
        raise ValueError(f'{name} must not be empty')
    if image.ndim not in (2, 3):
        raise ValueError(f'{name} must have shape HxW, HxWx1, HxWx3, or HxWx4')
    if image.ndim == 3 and image.shape[2] not in (1, 3, 4):
        raise ValueError(f'{name} must have 1, 3, or 4 channels')
    if image.dtype.kind not in 'uif':
        raise TypeError(f'{name} must have a real numeric dtype')

    scaled = image.astype(np.float64, copy=False)
    if not np.all(np.isfinite(scaled)):
        raise ValueError(f'{name} must contain only finite values')

    minimum = float(np.min(scaled))
    maximum = float(np.max(scaled))
    if minimum < 0.0:
        raise ValueError(f'{name} values must be non-negative')
    if image.dtype.kind == 'f' and maximum <= 1.0:
        scaled = scaled * 255.0
    elif maximum > 255.0:
        raise ValueError(f'{name} values must be within 0..1 or 0..255')
    return scaled


def _to_gray(image, name, channel_order):
    """Validate an image and convert common gray/RGB/BGR layouts to gray."""
    scaled = _validate_and_scale(image, name)
    if scaled.ndim == 2:
        return scaled
    if scaled.shape[2] == 1:
        return scaled[:, :, 0]
    if channel_order not in ('rgb', 'bgr'):
        raise ValueError("channel_order must be 'rgb' or 'bgr'")

    color = scaled[:, :, :3]
    weights = _RGB_WEIGHTS
    if channel_order == 'bgr':
        weights = weights[::-1]
    return np.tensordot(color, weights, axes=([2], [0]))


def _laplacian(gray):
    """Calculate a four-neighbour discrete Laplacian with edge padding."""
    padded = np.pad(gray, 1, mode='edge')
    return (
        padded[:-2, 1:-1]
        + padded[2:, 1:-1]
        + padded[1:-1, :-2]
        + padded[1:-1, 2:]
        - 4.0 * gray
    )


def _gradient_magnitude(gray):
    """Calculate central-difference gradient magnitude with edge padding."""
    padded = np.pad(gray, 1, mode='edge')
    dx = 0.5 * (padded[1:-1, 2:] - padded[1:-1, :-2])
    dy = 0.5 * (padded[2:, 1:-1] - padded[:-2, 1:-1])
    return np.hypot(dx, dy)


def _entropy(gray):
    """Calculate 256-bin Shannon entropy in bits."""
    histogram, _ = np.histogram(gray, bins=256, range=(0.0, 256.0))
    probabilities = histogram[histogram > 0] / gray.size
    entropy = float(-np.sum(probabilities * np.log2(probabilities)))
    return max(0.0, entropy)


def _fingerprint(gray):
    """Hash the canonical image dimensions and rounded 8-bit gray pixels."""
    canonical = np.rint(gray).clip(0.0, 255.0).astype(np.uint8)
    digest = sha256()
    digest.update(np.asarray(canonical.shape, dtype='<u8').tobytes())
    digest.update(canonical.tobytes(order='C'))
    return digest.hexdigest()


def compute_camera_health_features(
    image,
    previous_image=None,
    *,
    channel_order='rgb',
):
    """
    Return descriptive image features without assigning a health state.

    Integer images use the 0..255 intensity scale. Floating-point images may
    use either 0..1 or 0..255. RGB, BGR, RGBA and BGRA arrays are selected via
    ``channel_order``; alpha is ignored. ``dark_ratio`` and ``bright_ratio``
    describe the bottom and top intensity deciles. ``edge_density`` is the
    fraction of pixels whose central-difference gradient spans at least one
    intensity decile. These are feature definitions, not HEALTHY/FAULT rules.

    ``frame_diff_mean`` is ``None`` when no previous frame is supplied.
    Otherwise it is the mean absolute gray-level difference, and both frames
    must have the same height and width.
    """
    gray = _to_gray(image, 'image', channel_order)
    previous_gray = None
    if previous_image is not None:
        previous_gray = _to_gray(
            previous_image, 'previous_image', channel_order
        )
        if previous_gray.shape != gray.shape:
            raise ValueError('previous_image must match image height and width')

    frame_diff_mean = None
    if previous_gray is not None:
        frame_diff_mean = float(np.mean(np.abs(gray - previous_gray)))

    return {
        'mean_gray': float(np.mean(gray)),
        'gray_std': float(np.std(gray)),
        'p05': float(np.percentile(gray, 5)),
        'p95': float(np.percentile(gray, 95)),
        'dark_ratio': float(np.mean(gray <= _DARK_LEVEL)),
        'bright_ratio': float(np.mean(gray >= _BRIGHT_LEVEL)),
        'laplacian_variance': float(np.var(_laplacian(gray))),
        'edge_density': float(np.mean(
            _gradient_magnitude(gray) >= _EDGE_GRADIENT
        )),
        'entropy': _entropy(gray),
        'frame_diff_mean': frame_diff_mean,
        'frame_fingerprint': _fingerprint(gray),
    }
