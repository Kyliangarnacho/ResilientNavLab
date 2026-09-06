"""Static guardrails for the bounded Task 1 closeout harness."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE_ROOT / 'launch' / 'brne_task1_closeout.launch.py'
OBSERVER = PACKAGE_ROOT / 'resilient_nav_brne' / 'brne_task1_closeout_observer.py'
SHADOW = PACKAGE_ROOT / 'resilient_nav_brne' / 'brne_shadow_node.py'


def test_closeout_harness_is_one_phase10_shadow_run_with_observer_started_first():
    source = LAUNCH.read_text(encoding='utf-8')
    assert "'simple_reachable'" in source
    assert "'use_recovery': 'false'" in source
    assert source.index('observer,') < source.index('navigation,') < source.index('runner,')
    assert 'brne_task1_closeout_observer' in source
    assert 'venv_python' in source


def test_closeout_observer_and_shadow_keep_formal_cmd_vel_out_of_brne_outputs():
    observer = OBSERVER.read_text(encoding='utf-8')
    shadow = SHADOW.read_text(encoding='utf-8')
    assert "'/brne/cmd_vel_raw'" in observer
    assert "'/cmd_vel'" in observer  # passive graph audit only
    assert 'create_publisher(Twist, \'/cmd_vel\'' not in shadow
    assert "create_publisher(Twist, '/brne/cmd_vel_raw'" in shadow
