"""Static boundary contracts for the evaluation-only trajectory overlay."""

import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_path_adapter_reads_only_evaluation_or_odometry_outputs_and_uses_odom():
    """Visualization cannot subscribe to health, fusion, fault, or truth controls."""
    source = (
        PACKAGE_ROOT / 'resilient_nav_fusion' / 'trajectory_path_adapter.py'
    ).read_text(encoding='utf-8')
    ast.parse(source)

    for topic in (
        '/evaluation/ground_truth_pose',
        '/odometry/faulted',
        '/odometry/adaptive',
        "f'/evaluation/path/{name}'",
        "'expected_frame', 'odom'",
        "'expected_child_frame', 'base_footprint'",
    ):
        assert topic in source
    for forbidden in ('/fusion/', '/health/', '/faulted/', 'FaultStatus', 'SensorHealth'):
        assert forbidden not in source
    assert 'self._path_publishers' in source
    assert 'self._publishers' not in source


def test_overlay_launch_and_rviz_config_expose_three_paths_in_odom_frame():
    """The optional GUI is a display layer, not another estimator launch."""
    launch = (PACKAGE_ROOT / 'launch' / 'phase8_trajectory_overlay.launch.py').read_text(
        encoding='utf-8'
    )
    config = (PACKAGE_ROOT / 'config' / 'phase8_trajectories.rviz').read_text(
        encoding='utf-8'
    )
    ast.parse(launch)
    assert "executable='trajectory_path_adapter'" in launch
    assert "package='rviz2'" in launch
    assert 'IfCondition(use_rviz)' in launch
    assert 'Fixed Frame: odom' in config
    for topic in ('ground_truth', 'fixed', 'adaptive'):
        assert f'/evaluation/path/{topic}' in config
