"""Static contracts for the RPP counterpart of BRNE V1 / Scene 1."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE_ROOT / 'launch' / 'rpp_scene1_comparison_demo.launch.py'
COORDINATOR = PACKAGE_ROOT / 'resilient_nav_brne' / 'rpp_scene1_goal_coordinator.py'
SETUP = PACKAGE_ROOT / 'setup.py'
MANIFEST = PACKAGE_ROOT / 'package.xml'


def test_overlay_reuses_frozen_phase10_bt_rpp_and_exact_scene1_geometry():
    source = LAUNCH.read_text(encoding='utf-8')
    assert 'phase10_bt_navigation_smoke.launch.py' in source
    assert "'use_recovery': 'false'" in source
    for expected in (
        "'spawn_x': '-3.5'",
        "'spawn_y': '-3.5'",
        "'spawn_yaw': '0.0'",
        "'initial_pose_x': '0.0'",
        "'initial_pose_y': '0.0'",
        "'initial_pose_yaw': '0.0'",
        "'x': -3.0",
        "'y': -4.5",
        "'z': 0.60",
        "'Y': 1.5707963267948966",
        "'speed': 0.25",
        "'target_world_y': -2.50",
        "'start_delay_sec': 0.1",
    ):
        assert expected in source
    assert "'use_rviz': use_rviz" in source
    assert 'brne_crossing_pedestrian.sdf' in source
    assert '@std_msgs/msg/Float64]gz.msgs.Double' in source


def test_overlay_has_one_nav2_control_owner_and_starts_no_brne_control_chain():
    source = LAUNCH.read_text(encoding='utf-8')
    assert "executable='brne_crossing_pedestrian_driver'" in source
    assert "executable='rpp_scene1_goal_coordinator'" in source
    for forbidden in (
        "executable='brne_periodic_planner'",
        "executable='brne_shadow_input_adapter'",
        "executable='brne_pedestrian_odometry_adapter'",
        "executable='brne_shadow_node'",
        "executable='brne_control_gate'",
        "'/cmd_vel'",
    ):
        assert forbidden not in source
    assert "'ready_topic': '/rpp/ready'" in source
    assert "'plan_topic': '/plan'" in source
    assert "'odometry_topic': '/rpp/pedestrian/odometry'" in source


def test_coordinator_waits_for_active_navigation_then_latches_ready_only_on_goal_acceptance():
    source = COORDINATOR.read_text(encoding='utf-8')
    assert "'/lifecycle_manager_navigation/is_active'" in source
    assert 'Trigger' in source
    assert 'NavigateToPose' in source
    assert "request.behavior_tree = ''" in source
    assert 'if not response.success:' in source
    assert 'if not goal_handle.accepted:' in source
    assert 'self._publish_ready(True)' in source
    assert 'READY_QOS' in source
    for forbidden in ('ComputePathToPose', 'FollowPath', "'/cmd_vel'", 'Twist'):
        assert forbidden not in source


def test_coordinator_is_packaged_with_its_existing_runtime_dependencies_only():
    assert 'rpp_scene1_goal_coordinator = ' in SETUP.read_text(encoding='utf-8')
    manifest = MANIFEST.read_text(encoding='utf-8')
    assert '<depend>std_srvs</depend>' in manifest
