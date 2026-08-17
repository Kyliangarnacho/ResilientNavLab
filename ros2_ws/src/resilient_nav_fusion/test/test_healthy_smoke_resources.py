"""Static checks for the self-cleaning healthy smoke launch."""

import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / 'launch' / 'phase8_healthy_smoke.launch.py'
SCENARIO_FILE = PACKAGE_ROOT / 'config' / 'healthy_passthrough.yaml'


def test_healthy_smoke_launch_uses_passthrough_upstream_and_self_terminates():
    """No-fault upstream and probe exit must close the complete test graph."""
    source = LAUNCH_FILE.read_text(encoding='utf-8')
    ast.parse(source)

    assert "'phase6_health_evaluation.launch.py'" in source
    assert "'healthy_passthrough.yaml'" in source
    assert "executable='measurement_adapter'" in source
    assert "executable='ekf_node'" in source
    assert "executable='healthy_smoke_probe'" in source
    assert 'OnProcessExit' in source
    assert 'Shutdown' in source
    assert "'/odometry/adaptive'" in source
    assert "'/tmp/phase8_healthy_smoke_evidence.json'" in source


def test_probe_timeout_uses_monotonic_wall_time_not_sim_clock():
    """Clock resets while Gazebo starts cannot exhaust the smoke timeout."""
    source = (
        PACKAGE_ROOT / 'resilient_nav_fusion' / 'healthy_smoke_probe.py'
    ).read_text(encoding='utf-8')

    assert 'from time import monotonic' in source
    assert 'self._start_monotonic = monotonic()' in source
    assert 'monotonic() - self._start_monotonic' in source


def test_passthrough_scenario_has_no_truth_or_active_fault_configuration():
    """Phase 5 defaults create disabled pass-through injectors from this file."""
    source = SCENARIO_FILE.read_text(encoding='utf-8').lower()

    assert source.rstrip().endswith('{}')
    for forbidden in ('faultstatus', 'scenario_id', 'scenario_seed', 'model:'):
        assert forbidden not in source
