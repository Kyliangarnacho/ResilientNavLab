"""Resource checks for the phase 6 health evaluation launch."""

from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_DIR / 'launch' / 'phase6_health_evaluation.launch.py'
SETUP_FILE = PACKAGE_DIR / 'setup.py'


def test_phase6_launch_starts_phase5_monitor_and_evaluator():
    """The unified launch must compose phase 5 with both health nodes."""
    launch_source = LAUNCH_FILE.read_text(encoding='utf-8')

    assert "'phase5_fault_injection.launch.py'" in launch_source
    assert "executable='sensor_health_monitor'" in launch_source
    assert "'health_monitor.yaml'" in launch_source
    assert 'OpaqueFunction(function=create_health_evaluator)' in launch_source
    assert "SetParameter(name='use_sim_time', value=True)" in launch_source


def test_phase6_launch_declares_and_forwards_required_arguments():
    """The phase 6 entry point exposes the required phase controls."""
    launch_source = LAUNCH_FILE.read_text(encoding='utf-8')

    for argument in [
        'scenario_file',
        'use_rviz',
        'record_bag',
        'evaluator_output_json',
    ]:
        assert f"'{argument}'" in launch_source
    assert "'scenario_file': scenario_file" in launch_source
    assert "'use_rviz': use_rviz" in launch_source
    assert "'record_bag': record_bag" in launch_source
    assert "default_value='/tmp/phase6_health_evaluation.json'" in launch_source


def test_setup_installs_phase6_launch_and_health_configs():
    """The installed package share must contain its launch and YAML files."""
    setup_source = SETUP_FILE.read_text(encoding='utf-8')

    assert "os.path.join('share', package_name, 'launch')" in setup_source
    assert "glob(os.path.join('launch', '*.launch.py'))" in setup_source
    assert "os.path.join('share', package_name, 'config')" in setup_source
    assert "glob(os.path.join('config', '*.yaml'))" in setup_source
