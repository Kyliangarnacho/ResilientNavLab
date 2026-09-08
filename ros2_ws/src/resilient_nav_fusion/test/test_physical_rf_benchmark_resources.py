"""Static contracts for the final physical RF A/B benchmark launch."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE_ROOT / 'launch' / 'physical_disturbance_rf_benchmark.launch.py'


def test_benchmark_wires_one_fixed_and_one_adaptive_chain():
    source = LAUNCH.read_text(encoding='utf-8')

    assert "'/odometry/fixed'" in source
    assert "'phase8_adaptive_ekf.launch.py'" in source
    assert "'phase8_ground_truth.launch.py'" in source
    assert "executable='localization_evaluator'" in source
    assert "'/fusion/reliability'" in source


def test_benchmark_uses_passthrough_sensor_chain_and_self_terminates():
    source = LAUNCH.read_text(encoding='utf-8')

    assert "'enabled': False" in source
    assert "for sensor in ('wheel', 'imu', 'scan')" in source
    assert "executable='physical_disturbance_route'" not in source
    assert "'physical_disturbance_route'" in source
    assert 'OnProcessExit' in source
    assert "Shutdown(reason='physical RF benchmark complete')" in source
    assert "('route_profile', 'standard')" in source
    assert "'zone_length_m'" in source
    assert "'zone_width_m'" in source
    assert "('disturbance_pulse_count', '1')" in source
    assert "('disturbance_interval_sec', '3.0')" in source


def test_benchmark_exposes_runtime_fallback_threshold_without_truth_input():
    source = LAUNCH.read_text(encoding='utf-8')
    adaptive_source = (
        PACKAGE_ROOT / 'launch' / 'phase8_adaptive_ekf.launch.py'
    ).read_text(encoding='utf-8')

    assert "('fallback_reliability_threshold', '0.10')" in source
    assert "'fallback_reliability_threshold'" in adaptive_source


def test_experiment_parameters_are_not_forwarded_to_measurement_adapter():
    source = LAUNCH.read_text(encoding='utf-8')
    adapter_block = (
        PACKAGE_ROOT / 'launch' / 'phase8_adaptive_ekf.launch.py'
    ).read_text(encoding='utf-8').lower()

    assert "'scenario'" in source
    for forbidden in (
        'scenario_id',
        'scenario_seed',
        'parameters_yaml',
        'ground_truth',
    ):
        assert forbidden not in adapter_block
