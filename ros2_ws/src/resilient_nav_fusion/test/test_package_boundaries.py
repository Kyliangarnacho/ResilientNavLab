"""Static checks that the policy package remains ROS-free and read-only."""

from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_manifest_declares_adapter_and_launch_dependencies_only():
    """The package may launch EKF, but cannot depend on injection or truth APIs."""
    root = ET.parse(PACKAGE_ROOT / 'package.xml').getroot()
    dependencies = {
        element.text
        for element in root.findall('depend') + root.findall('exec_depend')
    }

    assert root.findtext('name') == 'resilient_nav_fusion'
    assert dependencies == {
        'geometry_msgs',
        'launch',
        'launch_ros',
        'nav_msgs',
        'rclpy',
        'resilient_nav_fault_injection',
        'resilient_nav_health_assessment',
        'resilient_nav_interfaces',
        'resilient_nav_simulation',
            'ros_gz_bridge',
            'robot_localization',
            'rviz2',
            'sensor_msgs',
        'std_msgs',
        'tf2_msgs',
    }


def test_policy_remains_ros_free_and_adapter_avoids_truth_or_ekf():
    """Keep the policy pure and the ROS boundary free of truth and EKF links."""
    source = (PACKAGE_ROOT / 'resilient_nav_fusion' / 'fusion_policy.py').read_text(
        encoding='utf-8'
    )
    normalized = source.lower()

    for forbidden in (
        'rclpy',
        'robot_localization',
        'faultstatus',
        'scenario_id',
        'scenario_seed',
        'parameters_yaml',
        'cmd_vel',
    ):
        assert forbidden not in normalized
    adapter = (PACKAGE_ROOT / 'resilient_nav_fusion' / 'measurement_adapter.py').read_text(
        encoding='utf-8'
    ).lower()
    for forbidden in (
        'robot_localization',
        'faultstatus',
        'scenario_id',
        'scenario_seed',
        'parameters_yaml',
        'cmd_vel',
    ):
        assert forbidden not in adapter


def test_adaptive_resources_do_not_reference_fault_or_truth_inputs():
    """Adaptive EKF consumes only adapter-owned, truth-free measurements."""
    config = (PACKAGE_ROOT / 'config' / 'adaptive_ekf.yaml').read_text(
        encoding='utf-8'
    ).lower()
    launch = (PACKAGE_ROOT / 'launch' / 'phase8_adaptive_ekf.launch.py').read_text(
        encoding='utf-8'
    ).lower()

    for source in (config, launch):
        for forbidden in (
            '/faulted/',
            'faultstatus',
            'scenario_id',
            'scenario_seed',
            'parameters_yaml',
            'cmd_vel',
        ):
            assert forbidden not in source


def test_adapter_uses_sensor_qos_for_faulted_measurement_streams():
    """Best-effort fault injectors remain compatible with adapter inputs."""
    adapter = (PACKAGE_ROOT / 'resilient_nav_fusion' / 'measurement_adapter.py').read_text(
        encoding='utf-8'
    )

    assert 'from rclpy.qos import qos_profile_sensor_data' in adapter
    assert adapter.count('qos_profile_sensor_data,') == 2
