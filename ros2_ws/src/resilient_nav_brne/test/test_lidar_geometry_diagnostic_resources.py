from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_diagnostic_demo_keeps_ground_truth_in_evaluator_only_path():
    launch_source = (
        PACKAGE_ROOT / 'launch' / 'lidar_geometry_diagnostic_demo.launch.py'
    ).read_text(encoding='utf-8')

    assert "executable='lidar_projection_evaluator'" in launch_source
    assert "global_obstacle_layer_enabled': 'false'" in launch_source
    assert '/brne/static_scan' not in launch_source
    assert "executable='brne_shadow_node'" not in launch_source
    assert "executable='brne_control_gate'" not in launch_source
    assert "executable='motion_test'" not in launch_source


def test_diagnostic_rviz_compares_odom_projections_and_displays_costmap():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'rviz' / 'lidar_geometry_diagnostic.rviz').read_text(
            encoding='utf-8'
        )
    )
    manager = config['Visualization Manager']
    displays = {display['Name']: display for display in manager['Displays']}

    assert manager['Global Options']['Fixed Frame'] == 'odom'
    assert displays['Global Costmap']['Enabled']
    assert displays['Global Costmap']['Topic']['Value'] == '/global_costmap/costmap'
    assert displays['GT Projection (green)']['Topic']['Value'] == (
        '/evaluation/lidar_points_gt'
    )
    assert displays['Exact-stamp EKF Projection (magenta)']['Topic']['Value'] == (
        '/evaluation/lidar_points_ekf'
    )
