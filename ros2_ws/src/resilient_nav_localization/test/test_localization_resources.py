"""Static tests for the stage 4 EKF localization resources."""

import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_XML_PATH = PACKAGE_ROOT / 'package.xml'
CMAKE_PATH = PACKAGE_ROOT / 'CMakeLists.txt'
EKF_CONFIG_PATH = PACKAGE_ROOT / 'config' / 'ekf.yaml'
EKF_LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'phase4_ekf_demo.launch.py'


def ekf_parameters():
    """Load and return the EKF ROS parameter dictionary."""
    with EKF_CONFIG_PATH.open(encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file)
    return config['ekf_filter_node']['ros__parameters']


def test_package_manifest_and_install_rules():
    """The package should declare runtime dependencies and install resources."""
    root = ET.parse(PACKAGE_XML_PATH).getroot()
    dependencies = {
        element.text
        for element in root.findall('exec_depend')
    }

    assert root.findtext('name') == 'resilient_nav_localization'
    assert {'robot_localization', 'resilient_nav_simulation'} <= dependencies

    cmake_source = CMAKE_PATH.read_text(encoding='utf-8')
    assert 'DIRECTORY config launch' in cmake_source
    assert 'ament_add_pytest_test' in cmake_source


def test_ekf_yaml_uses_local_planar_frames_and_sim_time():
    """The EKF should publish a planar local estimate in the odom frame."""
    parameters = ekf_parameters()

    assert parameters['use_sim_time'] is True
    assert parameters['frequency'] == 20.0
    assert parameters['two_d_mode'] is True
    assert parameters['publish_tf'] is True
    assert parameters['world_frame'] == 'odom'
    assert parameters['odom_frame'] == 'odom'
    assert parameters['base_link_frame'] == 'base_footprint'


def test_ekf_inputs_use_only_wheel_vx_and_imu_yaw_rate():
    """The minimal fusion vector should avoid duplicate pose information."""
    parameters = ekf_parameters()

    assert parameters['odom0'] == '/wheel/odometry'
    assert parameters['imu0'] == '/imu/data'
    assert parameters['odom0_config'] == [
        False, False, False,
        False, False, False,
        True, False, False,
        False, False, False,
        False, False, False,
    ]
    assert parameters['imu0_config'] == [
        False, False, False,
        False, False, False,
        False, False, False,
        False, False, True,
        False, False, False,
    ]
    assert parameters['imu0_remove_gravitational_acceleration'] is False
    assert 'process_noise_covariance' not in parameters
    assert 'initial_estimate_covariance' not in parameters


def test_ekf_launch_reuses_rgbd_and_switches_tf_ownership():
    """The complete launch should remap wheel odom and disable old TF."""
    launch_source = EKF_LAUNCH_PATH.read_text(encoding='utf-8')
    ast.parse(launch_source)

    assert "'phase4_rgbd_demo.launch.py'" in launch_source
    assert "'odom_ros_topic': '/wheel/odometry'" in launch_source
    assert "'start_odom_tf_broadcaster': 'false'" in launch_source
    assert "package='robot_localization'" in launch_source
    assert "executable='ekf_node'" in launch_source
    assert "name='ekf_filter_node'" in launch_source
    assert "('odometry/filtered', '/odometry/filtered')" in launch_source
    assert "executable='odom_tf_broadcaster'" not in launch_source


def test_launch_and_config_names_match_installed_entrypoint():
    """The requested launch and configuration filenames should be exact."""
    assert EKF_CONFIG_PATH.is_file()
    assert EKF_LAUNCH_PATH.name == 'phase4_ekf_demo.launch.py'
