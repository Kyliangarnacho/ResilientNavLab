"""Static boundaries for the minimal BRNE closed-loop demo composition."""

from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE_ROOT / 'launch' / 'brne_closed_loop_demo.launch.py'
MODEL = PACKAGE_ROOT / 'models' / 'brne_crossing_pedestrian.sdf'
GATE = PACKAGE_ROOT / 'resilient_nav_brne' / 'brne_control_gate.py'
PLANNER = PACKAGE_ROOT / 'resilient_nav_brne' / 'brne_periodic_planner.py'
PEDESTRIAN = (
    PACKAGE_ROOT / 'resilient_nav_brne' / 'brne_pedestrian_odometry_adapter.py'
)
SHADOW_NODE = PACKAGE_ROOT / 'resilient_nav_brne' / 'brne_shadow_node.py'
DRIVER = PACKAGE_ROOT / 'resilient_nav_brne' / 'brne_crossing_pedestrian_driver.py'
STATE = PACKAGE_ROOT / 'resilient_nav_brne' / 'pedestrian_state.py'
RVIZ = PACKAGE_ROOT / 'rviz' / 'brne_closed_loop_demo.rviz'
STABILITY_LAUNCH = PACKAGE_ROOT / 'launch' / 'brne_pedestrian_stability.launch.py'
STABILITY_PROBE = (
    PACKAGE_ROOT / 'resilient_nav_brne' / 'brne_pedestrian_stability_probe.py'
)


def test_demo_uses_phase10_planner_only_and_never_starts_rpp_or_bt():
    source = LAUNCH.read_text(encoding='utf-8')
    assert 'phase10_planner_smoke.launch.py' in source
    assert 'brne_periodic_planner' in source
    assert 'brne_shadow_input_adapter' in source
    assert 'brne_shadow_node' in source
    assert "'spawn_x': '-3.5'" in source
    assert "'spawn_y': '-3.5'" in source
    assert "'spawn_yaw': '0.0'" in source
    assert "'use_rviz': 'false'" in source
    assert 'brne_closed_loop_demo_rviz' in source
    assert 'brne_closed_loop_demo.rviz' in source
    assert "'y': -4.5" in source
    assert "'z': 0.60" in source
    assert 'controller_server' not in source
    assert 'bt_navigator' not in source
    assert 'brne_synthetic_pedestrian_source' not in source
    assert "'config' / 'brne_v1_runtime.yaml'" in source
    assert "parameters=[str(runtime_profile), {'use_sim_time': True}]" in source
    assert 'commitment' not in source


def test_gate_is_default_disarmed_and_has_stale_numeric_and_owner_guards():
    launch = LAUNCH.read_text(encoding='utf-8')
    gate = GATE.read_text(encoding='utf-8')
    assert "'arm_brne',\n            default_value='false'" in launch
    assert "self.declare_parameter('armed', False)" in gate
    assert 'get_publishers_info_by_topic' in gate
    assert 'time.monotonic() - self._raw_received_at' in gate
    assert 'validated_diff_drive_command' in gate
    assert 'destroy_publisher' in gate


def test_periodic_planner_requests_real_path_and_never_emulates_control():
    source = PLANNER.read_text(encoding='utf-8')
    assert 'ActionClient(' in source
    assert 'ComputePathToPose' in source
    assert 'request.use_start = False' in source
    assert "'/compute_path_to_pose'" in source
    assert 'path_freeze' not in source
    assert 'PedestrianArray' not in source
    assert "'/cmd_vel'" not in source


def test_pedestrian_adapter_uses_the_pinned_child_frame_contract_directly():
    adapter = PEDESTRIAN.read_text(encoding='utf-8')
    state = STATE.read_text(encoding='utf-8')
    assert 'TwistFrameVerifier' not in adapter
    assert 'verification_' not in adapter
    assert 'TwistFrameVerifier' not in state
    assert 'world_velocity = rotate_xy(twist_x, twist_y, world_yaw)' in state
    assert "'/brne/pedestrians'" in adapter


def test_pedestrian_cannot_start_until_brne_jit_is_ready():
    driver = DRIVER.read_text(encoding='utf-8')
    node = SHADOW_NODE.read_text(encoding='utf-8')
    assert "'/brne/ready'" in driver
    assert 'self._brne_ready' in driver
    assert 'self._brne_ready\n            and' in driver
    assert 'self.planner.warm_up()' in node
    assert "'/brne/ready'" in node
    assert 'DurabilityPolicy.TRANSIENT_LOCAL' in node


def test_model_is_a_world_constrained_single_axis_actor_with_minimal_plugins():
    root = ET.parse(MODEL).getroot()
    plugins = {
        plugin.attrib['filename']
        for plugin in root.findall('.//plugin')
    }
    assert plugins == {
        'gz-sim-joint-controller-system',
        'gz-sim-odometry-publisher-system',
    }
    assert root.find('.//collision') is not None
    assert root.find('.//kinematic') is None
    assert root.findtext('.//mass') == '40.0'
    assert root.findtext('.//cylinder/radius') == '0.18'
    assert root.findtext('.//cylinder/length') == '1.20'
    joint = root.find(".//joint[@name='crossing_joint']")
    assert joint is not None
    assert joint.attrib['type'] == 'prismatic'
    assert joint.findtext('parent') == 'world'
    assert joint.findtext('child') == 'body'
    axis = joint.find('axis/xyz')
    assert axis is not None
    assert axis.text == '1 0 0'
    assert axis.attrib['expressed_in'] == '__model__'
    assert joint.findtext('axis/limit/lower') == '0.0'
    assert joint.findtext('axis/limit/upper') == '3.0'
    assert joint.findtext('axis/limit/velocity') == '0.25'
    assert root.findtext(".//plugin[@name='gz::sim::systems::JointController']/joint_name") == 'crossing_joint'
    assert root.findtext(".//plugin[@name='gz::sim::systems::JointController']/topic") == (
        '/model/brne_pedestrian/joint/crossing_joint/cmd_vel'
    )
    assert root.findtext('.//odom_frame') == 'gazebo_world'
    assert root.findtext('.//robot_base_frame') == 'brne_pedestrian'
    assert root.findtext('.//dimensions') == '3'


def test_joint_velocity_driver_and_standalone_probe_preserve_the_runtime_boundary():
    launch = LAUNCH.read_text(encoding='utf-8')
    driver = DRIVER.read_text(encoding='utf-8')
    stability_launch = STABILITY_LAUNCH.read_text(encoding='utf-8')
    probe = STABILITY_PROBE.read_text(encoding='utf-8')

    for source in (launch, stability_launch):
        assert '/model/brne_pedestrian/joint/crossing_joint/cmd_vel' in source
        assert '@std_msgs/msg/Float64]gz.msgs.Double' in source
        assert '/brne/pedestrian/joint_velocity' in source
    assert 'Float64(data=speed)' in driver
    assert "'/brne/pedestrian/joint_velocity'" in driver
    assert 'Twist' not in driver
    assert 'Odometry' in driver
    assert 'observation_duration_sec' in probe
    assert "'observation_duration_sec': 60.0" in stability_launch
    assert 'max_abs_roll_rad' in probe
    assert 'max_abs_pitch_rad' in probe
    assert 'brne_pedestrian_stability_result.json' in probe
    assert "'unexpected odometry child frame'" in probe
    assert 'brne_pedestrian_stability_probe' in probe


def test_rviz_displays_the_existing_brne_selected_prediction_in_red():
    source = RVIZ.read_text(encoding='utf-8')
    assert 'Fixed Frame: map' in source
    assert 'Class: rviz_default_plugins/Polygon' in source
    assert 'Name: Robot Footprint' in source
    assert 'Value: /global_costmap/published_footprint' in source
    assert 'Name: Nav2 Global Plan' in source
    assert 'Color: 0; 170; 255' in source
    assert 'Value: /plan' in source
    assert 'BRNE Selected Prediction' in source
    assert 'Value: /brne/optimal_path' in source
    assert 'Color: 255; 0; 0' in source
    assert 'Line Width: 0.10' in source
    assert 'Distance: 4.5' in source
    assert 'X: 0.5' in source
    assert 'Class: rviz_default_plugins/RobotModel' in source
    assert 'Class: rviz_default_plugins/TF' in source
