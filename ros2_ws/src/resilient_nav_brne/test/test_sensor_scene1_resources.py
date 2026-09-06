"""Static boundaries for the sensor-input BRNE Scene 1 demo."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE_ROOT / 'launch' / 'brne_sensor_scene1_demo.launch.py'
NODE = PACKAGE_ROOT / 'resilient_nav_brne' / 'brne_lidar_dynamic_agent_node.py'
CORE = PACKAGE_ROOT / 'resilient_nav_brne' / 'lidar_dynamic_tracker.py'
RVIZ = PACKAGE_ROOT / 'rviz' / 'brne_sensor_scene1_demo.rviz'
SETUP = PACKAGE_ROOT / 'setup.py'
MANIFEST = PACKAGE_ROOT / 'package.xml'
PLANNER_LAUNCH = (
    PACKAGE_ROOT.parent
    / 'resilient_nav_navigation'
    / 'launch'
    / 'phase10_planner_smoke.launch.py'
)
PERIODIC_PLANNER = (
    PACKAGE_ROOT / 'resilient_nav_brne' / 'brne_periodic_planner.py'
)


def test_sensor_scene1_uses_x_point_eight_crossing_and_keeps_existing_brne_chain():
    source = LAUNCH.read_text(encoding='utf-8')
    assert 'low_pass' not in source
    for expected in (
        "'spawn_x': '-3.5'",
        "'spawn_y': '-3.5'",
        "'x': -2.7",
        "'y': -4.5",
        "'target_world_y': -1.5",
        "'goal_x': 2.0",
        "'config' / 'brne_v1_runtime.yaml'",
        "parameters=[str(runtime_profile), {'use_sim_time': True}]",
        "executable='brne_shadow_node'",
        "executable='brne_control_gate'",
    ):
        assert expected in source
    assert 'commitment' not in source
    assert "DeclareLaunchArgument('crossing_" not in source


def test_brne_agent_input_is_restored_to_the_real_sensor_tracker():
    source = LAUNCH.read_text(encoding='utf-8')
    assert "executable='brne_lidar_dynamic_agent_node'" in source
    assert "'output_topic': '/brne/sensor_pedestrians_unused'" not in source
    assert "executable='brne_pedestrian_odometry_adapter'" not in source
    assert "'/scenario/brne_pedestrian/odometry'" in source
    assert "'odometry_topic': '/scenario/brne_pedestrian/odometry'" in source


def test_sensor_node_uses_scan_timestamped_odom_tf_and_existing_message_contract():
    source = NODE.read_text(encoding='utf-8')
    assert "self.declare_parameter('scan_topic', '/scan')" in source
    assert "self.declare_parameter('output_frame', 'odom')" in source
    assert "self.declare_parameter('output_topic', '/brne/pedestrians')" in source
    assert 'Time.from_msg(scan.header.stamp)' in source
    assert 'lookup_transform(' in source
    assert 'self._pending_scan is None' in source
    assert 'PedestrianArray()' in source
    assert 'common_drift=' in source
    assert 'dynamic_agents=' in source
    assert "declare_parameter('maximum_dynamic_agent_range', 4.0)" in source
    assert "declare_parameter('velocity_ema_stability_window', 3)" in source
    assert (
        "declare_parameter('velocity_ema_max_direction_change_rad', 0.35)"
        in source
    )
    assert (
        "declare_parameter('velocity_ema_outlier_direction_change_rad', 0.70)"
        in source
    )
    assert "declare_parameter('candidate_minimum_confirmations', 3)" in source
    assert "declare_parameter('candidate_minimum_displacement', 0.02)" in source
    assert "declare_parameter('stationary_maximum_speed', 0.04)" in source
    assert "declare_parameter('stationary_confirmation_frames', 8)" in source
    assert "declare_parameter('filtered_scan_topic', '/brne/static_scan')" in source
    assert 'self.tracker.costmap_exclusion_agents()' in source
    assert 'ranges_without_dynamic_agents(' in source
    assert 'self.filtered_scan_publisher.publish(filtered_scan)' in source


def test_only_global_planner_uses_the_static_filtered_scan():
    scene_launch = LAUNCH.read_text(encoding='utf-8')
    planner_launch = PLANNER_LAUNCH.read_text(encoding='utf-8')
    assert "'planner_scan_topic': '/brne/static_scan'" in scene_launch
    assert "DeclareLaunchArgument('planner_scan_topic', default_value='/scan')" in (
        planner_launch
    )
    assert "('/scan', planner_scan_topic)" in planner_launch


def test_sensor_scene1_keeps_periodic_global_replanning_enabled():
    scene_launch = LAUNCH.read_text(encoding='utf-8')
    planner_launch = PLANNER_LAUNCH.read_text(encoding='utf-8')
    assert "'planner_plan_topic': '/brne/navfn_plan_raw'" not in scene_launch
    assert "'path_freeze_enabled': True" not in scene_launch
    assert "DeclareLaunchArgument('planner_plan_topic', default_value='/plan')" in (
        planner_launch
    )
    assert "('/plan', planner_plan_topic)" in planner_launch
    assert 'path_freeze' not in PERIODIC_PLANNER.read_text(encoding='utf-8')


def test_tracking_core_stays_ros_free_and_uses_thin_v1_rules():
    source = CORE.read_text(encoding='utf-8')
    for forbidden in ('rclpy', 'sensor_msgs', 'tf2_ros', 'nav_msgs'):
        assert forbidden not in source
    assert 'np.median' in source
    assert '_greedy_associations' in source
    assert 'dynamic_minimum_speed: float = 0.08' in source
    assert 'minimum_confirmations: int = 6' in source
    assert 'direction_consistency: float = 0.75' in source
    assert 'minimum_direction_step: float = 0.005' in source
    assert 'velocity_ema_alpha: float = 0.25' in source
    assert 'velocity_ema_stability_window: int = 3' in source
    assert 'velocity_ema_max_direction_change_rad: float = 0.35' in source
    assert 'velocity_ema_outlier_direction_change_rad: float = 0.70' in source
    assert 'candidate_minimum_confirmations: int = 3' in source
    assert 'candidate_minimum_displacement: float = 0.02' in source
    assert 'stationary_maximum_speed: float = 0.04' in source
    assert 'stationary_confirmation_frames: int = 8' in source
    assert '_reject_velocity_direction_outlier(' in source
    assert '_ema_velocity(' in source
    assert "declare_parameter('velocity_ema_alpha', 0.25)" in NODE.read_text(
        encoding='utf-8'
    )
    assert "declare_parameter('minimum_direction_step', 0.005)" in (
        NODE.read_text(encoding='utf-8')
    )


def test_sensor_scene1_installs_node_dependencies_and_visual_evidence():
    setup = SETUP.read_text(encoding='utf-8')
    manifest = MANIFEST.read_text(encoding='utf-8')
    rviz = RVIZ.read_text(encoding='utf-8')
    assert 'brne_lidar_dynamic_agent_node = ' in setup
    assert '<depend>sensor_msgs</depend>' in manifest
    assert '<depend>visualization_msgs</depend>' in manifest
    assert 'Name: Robot LiDAR' in rviz
    assert 'Value: /scan' in rviz
    assert 'Name: Sensor Dynamic Agents' in rviz
    assert 'Value: /brne/sensor_dynamic_agents' in rviz
    assert 'Name: Global Costmap' in rviz
    assert 'Value: /global_costmap/costmap' in rviz
