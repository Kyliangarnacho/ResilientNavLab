"""Static contract checks for the Phase 8 adaptive EKF resources."""

import ast
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_FILE = PACKAGE_ROOT / 'config' / 'adaptive_ekf.yaml'
LAUNCH_FILE = PACKAGE_ROOT / 'launch' / 'phase8_adaptive_ekf.launch.py'
SETUP_FILE = PACKAGE_ROOT / 'setup.py'


def load_adaptive_parameters():
    """Load the EKF configuration under its explicit node key."""
    with CONFIG_FILE.open('r', encoding='utf-8') as config_stream:
        config = yaml.safe_load(config_stream)
    return config['adaptive_ekf_filter_node']['ros__parameters']


def test_adaptive_ekf_uses_only_adapter_measurements_and_no_tf():
    """The EKF is a policy consumer, not another health-decision point."""
    parameters = load_adaptive_parameters()

    assert parameters['odom0'] == '/fusion/input/wheel/odometry'
    assert parameters['imu0'] == '/fusion/input/imu/data'
    assert parameters['twist0'] == '/fusion/input/wheel/yaw_rate'
    assert parameters['twist1'] == '/fusion/input/lidar/velocity'
    assert parameters['publish_tf'] is False
    assert parameters['world_frame'] == 'odom'
    assert parameters['odom_frame'] == 'odom'
    assert parameters['base_link_frame'] == 'base_footprint'


def test_adaptive_ekf_uses_wheel_yaw_vx_and_independent_yaw_rates():
    """Use wheel yaw/vx, IMU yaw-rate, and fallback wheel yaw-rate."""
    parameters = load_adaptive_parameters()

    assert parameters['odom0_config'] == [
        False, False, False, False, False, True, True, False, False,
        False, False, False, False, False, False,
    ]
    yaw_rate_only = [
        False, False, False, False, False, False, False, False, False,
        False, False, True, False, False, False,
    ]
    assert parameters['imu0_config'] == yaw_rate_only
    assert parameters['twist0_config'] == yaw_rate_only
    assert parameters['twist1_config'] == [
        False, False, False, False, False, False, True, False, False,
        False, False, False, False, False, False,
    ]


def test_minimal_launch_starts_adapter_and_ekf_with_fixed_output():
    """Launch must wire exactly the adapter plus a non-TF adaptive EKF."""
    source = LAUNCH_FILE.read_text(encoding='utf-8')
    ast.parse(source)

    assert "package='resilient_nav_fusion'" in source
    assert "executable='measurement_adapter'" in source
    assert "executable='lidar_odometry'" in source
    assert "'lidar_odometry.yaml'" in source
    assert "package='robot_localization'" in source
    assert "executable='ekf_node'" in source
    assert "name='adaptive_ekf_filter_node'" in source
    assert "('odometry/filtered', '/odometry/adaptive')" in source
    assert "'adaptive_ekf.yaml'" in source


def test_setup_installs_adaptive_launch_and_config():
    """Installed package shares the resources required by ros2 launch."""
    source = SETUP_FILE.read_text(encoding='utf-8')

    assert "os.path.join('share', package_name, 'launch')" in source
    assert "glob(os.path.join('launch', '*.launch.py'))" in source
    assert "os.path.join('share', package_name, 'config')" in source
    assert "glob(os.path.join('config', '*.yaml'))" in source
    assert "'models', 'physical_reliability_rf_v2'" in source
