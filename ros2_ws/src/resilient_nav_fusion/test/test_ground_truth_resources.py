"""Static truth-channel isolation and bridge-resource checks."""

import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_ground_truth_launch_uses_verified_model_tf_source_and_pose_v_bridge():
    """The source is model-specific Gazebo TF, not an estimated ROS pose."""
    source = (PACKAGE_ROOT / 'launch' / 'phase8_ground_truth.launch.py').read_text(
        encoding='utf-8'
    )
    ast.parse(source)

    assert "package='ros_gz_bridge'" in source
    assert 'gz.msgs.Pose_V' in source
    assert 'TFMessage[gz.msgs.Pose_V' in source
    assert '/model/resilient_nav_robot/tf' in source
    assert '/evaluation/gazebo_model_tf' in source
    assert "executable='ground_truth_pose_adapter'" in source


def test_ground_truth_code_isolation_keeps_truth_out_of_runtime_fusion_chain():
    """Fusion, health adaptation, and EKF resources cannot import truth output."""
    truth_sources = [
        PACKAGE_ROOT / 'resilient_nav_fusion' / 'ground_truth_pose.py',
        PACKAGE_ROOT / 'resilient_nav_fusion' / 'ground_truth_pose_adapter.py',
        PACKAGE_ROOT / 'launch' / 'phase8_ground_truth.launch.py',
    ]
    runtime_sources = [
        PACKAGE_ROOT / 'resilient_nav_fusion' / 'fusion_policy.py',
        PACKAGE_ROOT / 'resilient_nav_fusion' / 'measurement_adapter.py',
        PACKAGE_ROOT / 'config' / 'adaptive_ekf.yaml',
    ]

    for path in truth_sources:
        source = path.read_text(encoding='utf-8').lower()
        for forbidden in ('faultstatus', 'scenario_id', 'scenario_seed', 'cmd_vel'):
            assert forbidden not in source
    for path in runtime_sources:
        source = path.read_text(encoding='utf-8').lower()
        assert 'ground_truth_pose' not in source
        assert '/evaluation/' not in source
