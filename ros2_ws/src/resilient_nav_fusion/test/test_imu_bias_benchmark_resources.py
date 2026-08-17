"""Static contracts for the isolated, controlled IMU-bias benchmark launch."""

import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_benchmark_launch_parameterizes_existing_phase6_scenarios_and_both_estimators():
    """The launch composes existing fault/health resources without retuning them."""
    source = (
        PACKAGE_ROOT / 'launch' / 'phase8_imu_bias_benchmark.launch.py'
    ).read_text(encoding='utf-8')
    ast.parse(source)

    for required in (
        'phase6_health_evaluation.launch.py',
        'imu_bias_ekf_comparison.yaml',
        "LaunchConfiguration('scenario_file')",
        "LaunchConfiguration('benchmark_name')",
        "LaunchConfiguration('fault_sensor')",
        "LaunchConfiguration('fault_status_sensor')",
        "LaunchConfiguration('health_topic')",
        "LaunchConfiguration('motion_duration_sec')",
        "LaunchConfiguration('benchmark_timeout_wall_sec')",
        "LaunchConfiguration('min_complete_sim_time_sec')",
        'phase8_adaptive_ekf.launch.py',
        'phase8_ground_truth.launch.py',
        "executable='localization_evaluator'",
        "executable='imu_benchmark_runner'",
        "'resilient_nav_simulation'",
        "'motion_test'",
        'EmitEvent(event=Shutdown',
    ):
        assert required in source


def test_benchmark_recorder_is_evaluation_only_and_runtime_chain_stays_truth_free():
    """Truth events stay in the recorder, never in policy, adapter, or EKF."""
    recorder = (
        PACKAGE_ROOT / 'resilient_nav_fusion' / 'imu_bias_benchmark_runner.py'
    ).read_text(encoding='utf-8').lower()
    assert '/fault_injection/status' in recorder
    assert '/evaluation/localization_metrics' in recorder
    assert 'measurement_adapter' not in recorder
    assert 'robot_localization' not in recorder
    assert 'clocktype.steady_time' in recorder

    for relative_path in (
        'resilient_nav_fusion/fusion_policy.py',
        'resilient_nav_fusion/measurement_adapter.py',
        'config/adaptive_ekf.yaml',
    ):
        source = (PACKAGE_ROOT / relative_path).read_text(encoding='utf-8').lower()
        assert '/evaluation/' not in source
        assert 'faultstatus' not in source
