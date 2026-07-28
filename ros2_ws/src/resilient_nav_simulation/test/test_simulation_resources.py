"""Static tests for the phase 2 and phase 3 simulation resources."""

import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORLD_PATH = PACKAGE_ROOT / 'worlds' / 'phase2_world.sdf'
BRIDGE_PATH = PACKAGE_ROOT / 'config' / 'bridge.yaml'
LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'phase2_world.launch.py'
SPAWN_LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'phase3_spawn.launch.py'
DEMO_LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'phase3_demo.launch.py'
DEMO_RVIZ_PATH = PACKAGE_ROOT / 'rviz' / 'phase3_demo.rviz'


def test_sdf_exists_and_is_valid_xml():
    """The phase 2 SDF should exist and parse as XML."""
    assert WORLD_PATH.is_file()
    root = ET.parse(WORLD_PATH).getroot()
    assert root.tag == 'sdf'


def test_world_name_and_static_entities():
    """The world should contain the named static box and cylinder."""
    root = ET.parse(WORLD_PATH).getroot()
    world = root.find("./world[@name='resilient_lab']")

    assert world is not None

    box_model = world.find("./model[@name='box_obstacle']")
    cylinder_model = world.find("./model[@name='cylinder_checkpoint']")

    assert box_model is not None
    assert box_model.findtext('static') == 'true'
    assert box_model.find('.//geometry/box') is not None

    assert cylinder_model is not None
    assert cylinder_model.findtext('static') == 'true'
    assert cylinder_model.find('.//geometry/cylinder') is not None


def test_ground_has_explicit_contact_friction():
    """The reused ground should define reproducible ODE friction values."""
    root = ET.parse(WORLD_PATH).getroot()
    ground_collision = root.find(
        "./world/model[@name='ground_plane']"
        "/link/collision[@name='ground_collision']"
    )

    assert ground_collision is not None
    assert ground_collision.findtext('surface/friction/ode/mu') == '1.0'
    assert ground_collision.findtext('surface/friction/ode/mu2') == '1.0'


def test_bridge_config_contains_only_clock_bridge():
    """The bridge config should define one Gazebo-to-ROS clock bridge."""
    with BRIDGE_PATH.open(encoding='utf-8') as config_file:
        bridge_config = yaml.safe_load(config_file)

    assert bridge_config == [
        {
            'ros_topic_name': '/clock',
            'gz_topic_name': '/clock',
            'ros_type_name': 'rosgraph_msgs/msg/Clock',
            'gz_type_name': 'gz.msgs.Clock',
            'direction': 'GZ_TO_ROS',
            'qos_profile': 'CLOCK',
        }
    ]


def test_launch_contains_required_startup_parts():
    """The launch file should start Gazebo, the bridge, and heartbeat."""
    assert LAUNCH_PATH.is_file()
    launch_source = LAUNCH_PATH.read_text(encoding='utf-8')

    assert 'ros_gz_sim' in launch_source
    assert "'phase2_world.sdf'" in launch_source
    assert (
        "get_package_share_directory('resilient_nav_simulation')"
        in launch_source
    )
    assert "package='ros_gz_bridge'" in launch_source
    assert "'bridge.yaml'" in launch_source
    assert "package='resilient_nav_monitor'" in launch_source
    assert "executable='system_heartbeat'" in launch_source
    assert "'use_sim_time': True" in launch_source


def test_launch_starts_gazebo_gui_and_runs_simulation_by_default():
    """Gazebo should start running with its GUI instead of server-only mode."""
    launch_source = LAUNCH_PATH.read_text(encoding='utf-8')

    assert "'gz_args': f'-r {world_path}'" in launch_source
    assert "f'-r -s " not in launch_source
    assert '--headless-rendering' not in launch_source


def test_phase3_launch_reuses_world_and_spawns_robot():
    """The stage 3 launch should reuse phase 2 and spawn from Xacro."""
    assert SPAWN_LAUNCH_PATH.is_file()
    launch_source = SPAWN_LAUNCH_PATH.read_text(encoding='utf-8')

    assert "'phase2_world.launch.py'" in launch_source
    assert (
        "get_package_share_directory('resilient_nav_description')"
        in launch_source
    )
    assert "'resilient_nav_robot.urdf.xacro'" in launch_source
    assert "package='robot_state_publisher'" in launch_source
    assert "package='ros_gz_sim'" in launch_source
    assert "executable='create'" in launch_source
    assert "'world': 'resilient_lab'" in launch_source
    assert "'topic': '/robot_description'" in launch_source
    assert "default_value='0.25'" in launch_source
    assert "' gazebo_model_name:='" in launch_source
    assert 'ros2_control' not in launch_source
    assert 'gz.msgs.LaserScan' not in launch_source
    assert 'gz.msgs.Image' not in launch_source


def test_phase3_launch_bridges_standard_robot_topics_without_gazebo_tf():
    """The robot bridge should expose only the authorized Gazebo interfaces."""
    launch_source = SPAWN_LAUNCH_PATH.read_text(encoding='utf-8')

    assert "package='ros_gz_bridge'" in launch_source
    assert "name='robot_bridge'" in launch_source
    assert "'/model/', entity_name, '/cmd_vel'" in launch_source
    assert "'@geometry_msgs/msg/Twist]gz.msgs.Twist'" in launch_source
    assert "'/model/', entity_name, '/odometry'" in launch_source
    assert "'@nav_msgs/msg/Odometry[gz.msgs.Odometry'" in launch_source
    assert "'/world/resilient_lab/model/'" in launch_source
    assert "'@sensor_msgs/msg/JointState[gz.msgs.Model'" in launch_source
    assert "(cmd_vel_gz_topic, '/cmd_vel')" in launch_source
    assert "(odometry_gz_topic, '/odom')" in launch_source
    assert "(joint_state_gz_topic, '/joint_states')" in launch_source
    assert 'gz.msgs.Pose_V' not in launch_source


def test_phase3_launch_starts_ros_odom_tf_broadcaster():
    """The stage 3 launch should publish odom TF from the ROS odometry."""
    launch_source = SPAWN_LAUNCH_PATH.read_text(encoding='utf-8')

    assert "package='resilient_nav_monitor'" in launch_source
    assert "executable='odom_tf_broadcaster'" in launch_source
    assert "name='odom_tf_broadcaster'" in launch_source
    assert "parameters=[{'use_sim_time': True}]" in launch_source


def test_phase3_demo_reuses_spawn_chain_and_only_adds_rviz():
    """The demo should wrap the spawn chain without duplicate publishers."""
    assert DEMO_LAUNCH_PATH.is_file()
    launch_source = DEMO_LAUNCH_PATH.read_text(encoding='utf-8')

    assert "'phase3_spawn.launch.py'" in launch_source
    assert "package='rviz2'" in launch_source
    assert "executable='rviz2'" in launch_source
    assert "condition=IfCondition(use_rviz)" in launch_source
    assert "default_value='true'" in launch_source
    assert "parameters=[{'use_sim_time': True}]" in launch_source
    assert "package='robot_state_publisher'" not in launch_source
    assert "package='joint_state_publisher'" not in launch_source
    assert "package='joint_state_publisher_gui'" not in launch_source
    assert "package='ros_gz_bridge'" not in launch_source
    assert "executable='odom_tf_broadcaster'" not in launch_source


def test_phase3_demo_rviz_uses_odom_and_required_displays():
    """The demo config should show the robot TF tree over an odom grid."""
    assert DEMO_RVIZ_PATH.is_file()
    with DEMO_RVIZ_PATH.open(encoding='utf-8') as config_file:
        rviz_config = yaml.safe_load(config_file)

    manager = rviz_config['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == 'odom'

    displays = {
        display['Class']: display
        for display in manager['Displays']
    }
    assert displays['rviz_default_plugins/Grid']['Enabled'] is True
    assert (
        displays['rviz_default_plugins/RobotModel']['Enabled']
        is True
    )
    assert displays['rviz_default_plugins/TF']['Enabled'] is True

    odometry = displays['rviz_default_plugins/Odometry']
    assert odometry['Topic']['Value'] == '/odom'
    assert odometry['Enabled'] is False
