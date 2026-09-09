"""Static contracts for the GT-free localization-quality interface."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MESSAGE_PATH = PACKAGE_ROOT / 'msg' / 'LocalizationQuality.msg'


def test_localization_quality_is_registered_and_has_explicit_states():
    """The generated message exposes the four localization states."""
    cmake = (PACKAGE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
    source = MESSAGE_PATH.read_text(encoding='utf-8')

    assert '"msg/LocalizationQuality.msg"' in cmake
    assert 'bool localization_usable' in source
    assert 'uint8 LOCALIZATION_PROVISIONAL=0' in source
    assert 'uint8 LOCALIZATION_OK=1' in source
    assert 'uint8 LOCALIZATION_DEGRADED=2' in source
    assert 'uint8 LOCALIZATION_LOST=3' in source


def test_localization_quality_interface_does_not_leak_experiment_truth():
    """No benchmark answer is representable in the runtime interface."""
    source = MESSAGE_PATH.read_text(encoding='utf-8').lower()

    for forbidden in (
        'faultstatus',
        'fault_status',
        'scenario',
        'seed',
        'parameters_yaml',
        'ground_truth',
    ):
        assert forbidden not in source
