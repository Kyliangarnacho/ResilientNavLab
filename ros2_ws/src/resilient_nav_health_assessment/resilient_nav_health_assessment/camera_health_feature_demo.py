"""Demonstrate camera feature meanings with deterministic synthetic images."""

from dataclasses import dataclass

import numpy as np

from resilient_nav_health_assessment.camera_health_features import (
    compute_camera_health_features,
)


@dataclass(frozen=True)
class DemoScenario:
    """One synthetic image and its optional frame-comparison reference."""

    name: str
    image: np.ndarray
    previous_image: np.ndarray | None = None


def _checkerboard(size, cell_size):
    """Return a black-and-white checkerboard image."""
    rows, columns = np.indices((size, size))
    return (((rows // cell_size + columns // cell_size) % 2) * 255).astype(
        np.uint8
    )


def _gaussian_kernel(sigma, radius):
    """Return a normalized one-dimensional Gaussian kernel."""
    coordinates = np.arange(-radius, radius + 1, dtype=np.float64)
    kernel = np.exp(-(coordinates ** 2) / (2.0 * sigma ** 2))
    return kernel / np.sum(kernel)


def _convolve_reflect(image, kernel, axis):
    """Convolve one image axis using reflected boundary pixels."""
    radius = kernel.size // 2
    padding = [(0, 0)] * image.ndim
    padding[axis] = (radius, radius)
    padded = np.pad(image, padding, mode='reflect')
    windows = np.lib.stride_tricks.sliding_window_view(
        padded, kernel.size, axis=axis
    )
    return np.tensordot(windows, kernel, axes=([-1], [0]))


def _gaussian_blur(image, sigma=2.0, radius=6):
    """Create a visibly blurred image without adding an image library."""
    kernel = _gaussian_kernel(sigma, radius)
    horizontal = _convolve_reflect(
        image.astype(np.float64), kernel, axis=1
    )
    blurred = _convolve_reflect(horizontal, kernel, axis=0)
    return np.rint(blurred).clip(0.0, 255.0).astype(np.uint8)


def build_demo_scenarios(size=64):
    """Construct the fixed image set used by the feature demonstration."""
    if size < 16:
        raise ValueError('size must be at least 16 pixels')

    black = np.zeros((size, size), dtype=np.uint8)
    white = np.full((size, size), 255, dtype=np.uint8)
    uniform_gray = np.full((size, size), 128, dtype=np.uint8)
    gradient_row = np.rint(np.linspace(0.0, 255.0, size)).astype(np.uint8)
    gradient = np.tile(gradient_row, (size, 1))
    checkerboard = _checkerboard(size, cell_size=4)
    blurred_checkerboard = _gaussian_blur(checkerboard)

    slight_change = checkerboard.copy()
    center = size // 2
    patch = slight_change[center - 2:center + 2, center - 2:center + 2]
    slight_change[center - 2:center + 2, center - 2:center + 2] = np.where(
        patch < 128, patch + 8, patch - 8
    )

    return [
        DemoScenario('pure_black', black),
        DemoScenario('pure_white', white),
        DemoScenario('uniform_gray', uniform_gray),
        DemoScenario('gray_gradient', gradient),
        DemoScenario('checkerboard', checkerboard),
        DemoScenario(
            'checkerboard_blur', blurred_checkerboard, checkerboard
        ),
        DemoScenario('repeated_frame', checkerboard.copy(), checkerboard),
        DemoScenario('slight_change', slight_change, checkerboard),
    ]


def evaluate_demo_scenarios(scenarios=None):
    """Evaluate demo images exclusively through the existing feature API."""
    if scenarios is None:
        scenarios = build_demo_scenarios()

    results = []
    for scenario in scenarios:
        features = compute_camera_health_features(
            scenario.image, scenario.previous_image
        )
        fingerprint_same = None
        if scenario.previous_image is not None:
            previous_features = compute_camera_health_features(
                scenario.previous_image
            )
            fingerprint_same = (
                features['frame_fingerprint']
                == previous_features['frame_fingerprint']
            )
        results.append({
            'scenario': scenario.name,
            **features,
            'fingerprint_same': fingerprint_same,
        })
    return results


def _format_number(value, precision):
    """Format an optional numeric table cell."""
    if value is None:
        return 'n/a'
    return f'{value:.{precision}f}'


def format_demo_table(results):
    """Format evaluated scenarios as a compact, fixed-column text table."""
    headings = (
        'scenario',
        'mean_gray',
        'gray_std',
        'dark_ratio',
        'bright_ratio',
        'laplacian_variance',
        'edge_density',
        'entropy',
        'frame_diff_mean',
        'fingerprint_same',
    )
    widths = (18, 9, 8, 10, 12, 18, 12, 8, 15, 16)

    header = ' '.join(
        f'{heading:<{width}}' for heading, width in zip(headings, widths)
    )
    separator = ' '.join('-' * width for width in widths)
    lines = [header, separator]
    for result in results:
        fingerprint_same = result['fingerprint_same']
        fingerprint_text = 'n/a'
        if fingerprint_same is not None:
            fingerprint_text = 'yes' if fingerprint_same else 'no'
        cells = (
            result['scenario'],
            _format_number(result['mean_gray'], 2),
            _format_number(result['gray_std'], 2),
            _format_number(result['dark_ratio'], 3),
            _format_number(result['bright_ratio'], 3),
            _format_number(result['laplacian_variance'], 2),
            _format_number(result['edge_density'], 3),
            _format_number(result['entropy'], 3),
            _format_number(result['frame_diff_mean'], 3),
            fingerprint_text,
        )
        lines.append(' '.join(
            f'{cell:<{width}}' for cell, width in zip(cells, widths)
        ))
    return '\n'.join(lines)


def main():
    """Print feature values for all deterministic demonstration scenes."""
    print(format_demo_table(evaluate_demo_scenarios()))


if __name__ == '__main__':
    main()
