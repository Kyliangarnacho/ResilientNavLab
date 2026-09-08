"""Contracts for the narrow, sequential two-pedestrian Scene 2 overlay."""

from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE_ROOT / 'launch' / 'brne_scene2_demo.launch.py'
MODEL = PACKAGE_ROOT / 'models' / 'brne_crossing_pedestrian_2.sdf'
PLANNER = PACKAGE_ROOT / 'resilient_nav_brne' / 'shadow_planner.py'
INTERACTION = PACKAGE_ROOT / 'resilient_nav_brne' / 'interaction_state.py'
ADAPTER = PACKAGE_ROOT / 'resilient_nav_brne' / 'brne_pedestrian_odometry_adapter.py'
COORDINATOR = PACKAGE_ROOT / 'resilient_nav_brne' / 'brne_scene2_coordinator.py'
SETUP = PACKAGE_ROOT / 'setup.py'


def test_scene2_uses_sensor_input_and_scene1_tuned_brne_profile():
    source = LAUNCH.read_text(encoding='utf-8')
    for expected in (
        "'goal_x': 2.8", "'x': -2.5", "'y': -4.5",
        "'x': -1.4", "'y': -2.5", "'Y': -1.5707963267948966",
        "'robot_min_map_x': 0.8", "'scene_delay_sec': 0.4",
        "'config' / 'brne_v1_runtime.yaml'",
        "parameters=[str(runtime_profile), {'use_sim_time': True}]",
        "'global_obstacle_layer_enabled': 'false'",
        "'world_y_direction': -1",
    ):
        assert expected in source
    assert "executable='brne_lidar_dynamic_agent_node'" in source
    assert "executable='brne_pedestrian_odometry_adapter'" not in source
    assert "executable='brne_control_gate'" in source
    assert "arguments=['--ros-args', '--log-level', log_level]" in source
    assert 'TimerAction(period=5.0, actions=[spawn_pedestrian1, spawn_pedestrian2])' in source
    assert 'controller_server' not in source
    assert 'rpp_scene1_goal_coordinator' not in source
    assert 'commitment' not in source
    assert "DeclareLaunchArgument('crossing_" not in source


def test_scene2_preserves_ids_and_uses_one_aggregating_pedestrian_writer():
    source = LAUNCH.read_text(encoding='utf-8')
    adapter = ADAPTER.read_text(encoding='utf-8')
    planner = PLANNER.read_text(encoding='utf-8')
    interaction = INTERACTION.read_text(encoding='utf-8')
    assert "'pedestrian_ids_csv': '1,2'" not in source
    assert "'input_topics_csv': '/brne/pedestrian/odometry,/brne/pedestrian2/odometry'" not in source
    assert 'input_child_frames_csv' in adapter
    assert '_csv_values' in adapter
    assert 'pedestrian sources require non-negative unique IDs' in adapter
    assert 'sorted(self._latest.items())' in adapter
    assert 'now_wall - received_at <= self.maximum_stamp_age_sec' in adapter
    assert 'pedestrian_id' in planner
    assert 'commitment' not in planner
    assert 'pedestrian_id' in interaction
    assert 'min_distance_seen' in interaction


def test_scene2_pedestrian2_is_a_distinct_constrained_reverse_crossing_model():
    root = ET.parse(MODEL).getroot()
    assert root.find(".//model[@name='brne_pedestrian_2']") is not None
    joint = root.find(".//joint[@name='crossing_joint']")
    assert joint is not None
    assert joint.attrib['type'] == 'prismatic'
    assert joint.findtext('parent') == 'world'
    assert joint.findtext('axis/xyz') == '1 0 0'
    assert joint.find('axis/xyz').attrib['expressed_in'] == '__model__'
    assert joint.findtext('axis/limit/lower') == '0.0'
    assert joint.findtext('axis/limit/upper') == '2.0'
    assert '/model/brne_pedestrian_2/joint/crossing_joint/cmd_vel' in MODEL.read_text(encoding='utf-8')
    assert '/model/brne_pedestrian_2/odometry' in MODEL.read_text(encoding='utf-8')


def test_scene2_coordinator_uses_only_robot_progress_and_scenario_delay():
    source = COORDINATOR.read_text(encoding='utf-8')
    assert "'/amcl_pose'" in source
    assert 'self._robot_map_x > self.robot_min_map_x' in source
    assert 'now_sec - self._progress_gate_at >= self.scene_delay_sec' in source
    assert 'commitment' not in source
    assert 'brne_scene2_coordinator = ' in SETUP.read_text(encoding='utf-8')
