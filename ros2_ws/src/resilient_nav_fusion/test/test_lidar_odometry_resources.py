"""Static wiring checks for LiDAR translation fallback."""

from pathlib import Path

import pytest
import yaml

from resilient_nav_fusion.lidar_odometry import _bounded_planar_velocity


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_FILE = PACKAGE_ROOT / 'config' / 'lidar_odometry.yaml'
LAUNCH_FILE = PACKAGE_ROOT / 'launch' / 'phase8_adaptive_ekf.launch.py'
SETUP_FILE = PACKAGE_ROOT / 'setup.py'


def test_lidar_odometry_uses_faulted_scan_and_private_output():
    """The matcher consumes the isolated scan chain, not wheel odometry."""
    with CONFIG_FILE.open('r', encoding='utf-8') as stream:
        parameters = yaml.safe_load(stream)[
            'lidar_odometry'
        ]['ros__parameters']

    assert parameters['scan_topic'] == '/faulted/scan'
    assert parameters['output_topic'] == '/lidar/odometry/twist'
    assert parameters['status_topic'] == '/lidar/odometry/status'
    assert parameters['minimum_linear_variance'] > 0.0
    assert parameters['max_rmse_m'] > 0.0
    assert parameters['robust_mad_scale'] > 0.0
    assert parameters['robust_residual_floor_m'] > 0.0
    assert parameters['min_point_to_line_observability'] > 0.0
    assert parameters['normal_neighbor_max_distance_m'] > 0.0
    assert parameters['maximum_linear_speed_mps'] == 1.0


def test_lidar_odometry_rejects_matches_above_robot_speed_bound():
    """A low-RMSE aliased match cannot publish an implausible velocity."""
    assert _bounded_planar_velocity(0.02, 0.0, 0.1, 1.0) == pytest.approx(
        (0.2, 0.0)
    )
    assert _bounded_planar_velocity(0.11, 0.0, 0.1, 1.0) is None


def test_adaptive_launch_starts_single_lidar_odometry_source():
    """The adaptive launch owns one scan-matching source."""
    source = LAUNCH_FILE.read_text(encoding='utf-8')

    assert source.count("executable='lidar_odometry'") == 1
    assert source.count("'lidar_odometry.yaml'") == 1


def test_lidar_odometry_console_script_is_installed():
    """The launch target is exported as a console script."""
    source = SETUP_FILE.read_text(encoding='utf-8')

    assert "'lidar_odometry = '" in source
    assert 'resilient_nav_fusion.lidar_odometry:main' in source
