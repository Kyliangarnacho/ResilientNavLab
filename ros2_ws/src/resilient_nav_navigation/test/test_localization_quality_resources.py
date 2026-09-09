"""Static ROS packaging contracts for localization-quality monitoring."""

import ast
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_monitor_resources_are_installed_and_launchable():
    """The node, configuration, and launch file are installed together."""
    setup_source = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')
    launch_source = (
        PACKAGE_ROOT / 'launch' / 'localization_quality_monitor.launch.py'
    ).read_text(encoding='utf-8')
    ast.parse(launch_source)

    assert "'localization_quality'" in setup_source
    assert "'localization_quality_monitor'" in setup_source
    assert (
        'localization_quality_monitor = localization_quality_monitor:main'
        in setup_source
    )
    assert "executable='localization_quality_monitor'" in launch_source
    assert 'localization_quality_monitor.yaml' in launch_source


def test_monitor_declares_only_runtime_observation_topics():
    """The runtime monitor cannot consume truth or publish control."""
    source = (
        PACKAGE_ROOT / 'localization_quality_monitor.py'
    ).read_text(encoding='utf-8').lower()

    topics = ('/amcl_pose', '/tf', '/health/scan', '/localization/quality')
    for topic in topics:
        assert topic in source
    for forbidden in (
        'faultstatus',
        '/fault_injection/',
        'scenario_id',
        'scenario_seed',
        '/evaluation/',
        'ground_truth',
        '/cmd_vel',
    ):
        assert forbidden not in source


def test_monitor_config_keeps_ordered_conservative_thresholds():
    """Default soft thresholds precede their hard lost counterparts."""
    config = yaml.safe_load(
        (
            PACKAGE_ROOT / 'config' / 'localization_quality_monitor.yaml'
        ).read_text(encoding='utf-8')
    )['localization_quality_monitor']['ros__parameters']

    assert config['startup_grace_sec'] <= 3.0
    assert config['recovery_clean_observations'] == 2
    for prefix in ('pose_age', 'tf_age', 'scan_health_age'):
        assert config[f'{prefix}_warning_sec'] < config[f'{prefix}_lost_sec']
    assert (
        config['position_variance_warning']
        < config['position_variance_lost']
    )
    assert config['yaw_variance_warning'] < config['yaw_variance_lost']


def test_navigation_package_depends_on_runtime_interface_and_tf_messages():
    """Runtime message dependencies are explicit in the package manifest."""
    root = ET.parse(PACKAGE_ROOT / 'package.xml').getroot()
    dependencies = {element.text for element in root.findall('exec_depend')}

    assert {'resilient_nav_interfaces', 'tf2_msgs'} <= dependencies
