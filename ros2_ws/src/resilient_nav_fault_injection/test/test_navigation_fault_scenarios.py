"""Contracts for the bounded fault-aware navigation scenarios."""

from pathlib import Path

import yaml


SCENARIO_DIR = Path(__file__).resolve().parents[1] / 'config' / 'scenarios'


def parameters(name, node):
    """Load one injector parameter mapping."""
    data = yaml.safe_load(
        (SCENARIO_DIR / name).read_text(encoding='utf-8')
    )
    return data[node]['ros__parameters']


def test_single_sensor_fault_windows_are_bounded_and_recover():
    """Wheel and LiDAR faults start after motion and last 7--10 seconds."""
    cases = (
        ('navigation_wheel_freeze.yaml', 'wheel_fault_injector', 'freeze'),
        ('navigation_wheel_bias.yaml', 'wheel_fault_injector', 'bias'),
        ('navigation_lidar_dropout.yaml', 'scan_fault_injector', 'dropout'),
    )
    for filename, node, model in cases:
        config = parameters(filename, node)
        assert config['model'] == model
        assert config['start_time_sec'] == 18.0
        assert 7.0 <= config['end_time_sec'] - config['start_time_sec'] <= 10.0


def test_simultaneous_fault_runs_increase_severity_without_time_overlap_drift():
    """Mild, moderate, and severe pairs share one simultaneous window."""
    mild_wheel = parameters(
        'navigation_wheel_imu_mild.yaml', 'wheel_fault_injector'
    )
    mild_imu = parameters(
        'navigation_wheel_imu_mild.yaml', 'imu_fault_injector'
    )
    moderate_wheel = parameters(
        'navigation_wheel_imu_moderate.yaml', 'wheel_fault_injector'
    )
    moderate_imu = parameters(
        'navigation_wheel_imu_moderate.yaml', 'imu_fault_injector'
    )
    severe_wheel = parameters(
        'navigation_wheel_imu_severe.yaml', 'wheel_fault_injector'
    )
    severe_imu = parameters(
        'navigation_wheel_imu_severe.yaml', 'imu_fault_injector'
    )

    for wheel, imu in (
        (mild_wheel, mild_imu),
        (moderate_wheel, moderate_imu),
        (severe_wheel, severe_imu),
    ):
        assert wheel['start_time_sec'] == imu['start_time_sec']
        assert wheel['end_time_sec'] == imu['end_time_sec']
    assert mild_wheel['linear_bias_mps'] < moderate_wheel['linear_bias_mps']
    assert mild_imu['bias_rad_s'] < moderate_imu['bias_rad_s']
    assert severe_wheel['model'] == 'freeze'
    assert severe_imu['model'] == 'dropout'
    assert severe_imu['dropout_probability'] == 1.0
