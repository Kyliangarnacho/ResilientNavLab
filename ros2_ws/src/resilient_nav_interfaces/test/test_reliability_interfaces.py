"""Static contracts for GT-free runtime reliability interfaces."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_reliability_interfaces_are_registered_and_truth_free():
    cmake = (PACKAGE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
    paths = [
        PACKAGE_ROOT / 'msg' / 'LidarOdometryStatus.msg',
        PACKAGE_ROOT / 'msg' / 'MeasurementReliability.msg',
    ]
    for path in paths:
        assert f'"msg/{path.name}"' in cmake
        normalized = path.read_text(encoding='utf-8').lower()
        for forbidden in (
            'faultstatus',
            'scenario',
            'seed',
            'parameters_yaml',
            'ground_truth',
        ):
            assert forbidden not in normalized


def test_reliability_message_exposes_component_probabilities():
    source = (
        PACKAGE_ROOT / 'msg' / 'MeasurementReliability.msg'
    ).read_text(encoding='utf-8')

    assert 'bool rf_ready' in source
    assert 'float32 wheel_translation' in source
    assert 'float32 wheel_rotation' in source
    assert 'float32 imu_yaw_rate' in source
    assert 'float32 lidar_translation' in source
