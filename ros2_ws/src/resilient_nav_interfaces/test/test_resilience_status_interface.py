"""Static contracts for the GT-free Resilience Supervisor output."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MESSAGE_PATH = PACKAGE_ROOT / 'msg' / 'ResilienceStatus.msg'


def test_resilience_status_is_registered_and_exposes_navigation_gate():
    """The supervisor output has explicit action-gating semantics."""
    cmake = (PACKAGE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
    source = MESSAGE_PATH.read_text(encoding='utf-8')

    assert '"msg/ResilienceStatus.msg"' in cmake
    assert 'bool navigation_allowed' in source
    for declaration in (
        'uint8 STARTING=0',
        'uint8 NAVIGATE=1',
        'uint8 DEGRADED=2',
        'uint8 HOLD=3',
    ):
        assert declaration in source


def test_resilience_status_cannot_carry_benchmark_truth():
    """The runtime output contains no experiment-answer fields."""
    source = MESSAGE_PATH.read_text(encoding='utf-8').lower()
    for forbidden in (
        'faultstatus', 'scenario', 'seed', 'parameters_yaml', 'ground_truth',
    ):
        assert forbidden not in source
