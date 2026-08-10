"""Static isolation checks for optional camera fault truth recording."""

from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
CALIBRATE_SOURCE = (
    PACKAGE_DIR
    / 'resilient_nav_health_assessment'
    / 'camera_health_calibrate.py'
)
MONITOR_SOURCE = (
    PACKAGE_DIR
    / 'resilient_nav_health_assessment'
    / 'camera_health_monitor.py'
)
REPORT_SOURCE = (
    PACKAGE_DIR
    / 'resilient_nav_health_assessment'
    / 'camera_fault_feature_report.py'
)
SETUP_FILE = PACKAGE_DIR / 'setup.py'


def test_fault_status_is_confined_to_calibration_and_offline_analysis():
    calibrate_source = CALIBRATE_SOURCE.read_text(encoding='utf-8')
    monitor_source = MONITOR_SOURCE.read_text(encoding='utf-8')

    assert 'from resilient_nav_interfaces.msg import FaultStatus' in (
        calibrate_source
    )
    assert "self.declare_parameter('record_fault_truth', False)" in (
        calibrate_source
    )
    assert 'FaultStatus' not in monitor_source
    assert '/fault_injection/status' not in monitor_source


def test_offline_report_has_no_ros_or_monitor_dependency():
    report_source = REPORT_SOURCE.read_text(encoding='utf-8')

    assert 'import rclpy' not in report_source
    assert 'camera_health_monitor' not in report_source
    assert 'FaultStatus' not in report_source
    assert 'thresholds_generated' in report_source


def test_fault_feature_report_is_installed_as_a_console_tool():
    setup_source = SETUP_FILE.read_text(encoding='utf-8')

    assert "'camera_fault_feature_report = '" in setup_source
    assert 'camera_fault_feature_report:main' in setup_source
