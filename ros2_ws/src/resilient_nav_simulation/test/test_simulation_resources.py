"""Static tests for the phase 2 and phase 3 simulation resources."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORLD_PATH = PACKAGE_ROOT / 'worlds' / 'phase2_world.sdf'
PHASE9_WORLD_PATH = PACKAGE_ROOT / 'worlds' / 'phase9_slam_world.sdf'
BRIDGE_PATH = PACKAGE_ROOT / 'config' / 'bridge.yaml'
LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'phase2_world.launch.py'
SPAWN_LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'phase3_spawn.launch.py'
DEMO_LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'phase3_demo.launch.py'
DEMO_RVIZ_PATH = PACKAGE_ROOT / 'rviz' / 'phase3_demo.rviz'
SENSOR_BRIDGE_PATH = (
    PACKAGE_ROOT / 'config' / 'phase4_sensor_bridge.yaml'
)
PHASE4_LAUNCH_PATH = (
    PACKAGE_ROOT / 'launch' / 'phase4_imu_lidar_demo.launch.py'
)
RGBD_BRIDGE_PATH = PACKAGE_ROOT / 'config' / 'phase4_rgbd_bridge.yaml'
RGBD_LAUNCH_PATH = (
    PACKAGE_ROOT / 'launch' / 'phase4_rgbd_demo.launch.py'
)
PHASE4_RVIZ_PATH = PACKAGE_ROOT / 'rviz' / 'phase4_sensors.rviz'
CMAKE_PATH = PACKAGE_ROOT / 'CMakeLists.txt'


def test_sdf_exists_and_is_valid_xml():
    """The phase 2 SDF should exist and parse as XML."""
    assert WORLD_PATH.is_file()
    root = ET.parse(WORLD_PATH).getroot()
    assert root.tag == 'sdf'


def test_phase9_mapping_world_has_static_asymmetric_scan_geometry():
    """The dedicated Phase 9 world supplies walls, corners, and occlusions."""
    assert PHASE9_WORLD_PATH.is_file()
    root = ET.parse(PHASE9_WORLD_PATH).getroot()
    world = root.find("./world[@name='resilient_lab']")

    assert world is not None
    model_names = {model.attrib['name'] for model in world.findall('./model')}
    assert {
        'wall_north', 'wall_south', 'wall_east', 'wall_west',
        'l_horizontal', 'l_vertical', 't_horizontal', 't_vertical',
        'central_rectangular_obstacle', 'northwest_occluder',
        'east_cylinder', 'small_asymmetric_block',
    } <= model_names
    assert all(
        model.findtext('static') == 'true'
        for model in world.findall('./model')
        if model.attrib['name'] != 'ground_plane'
    )
    assert world.find("./model[@name='east_cylinder']//geometry/cylinder") is not None
    assert world.find("./model[@name='l_horizontal']//geometry/box") is not None


def test_phase9_world_keeps_the_route_inside_the_lidar_scale():
    """The room and route start remain well inside the 12 m LiDAR envelope."""
    root = ET.parse(PHASE9_WORLD_PATH).getroot()
    north_wall = root.find("./world/model[@name='wall_north']/pose")
    south_wall = root.find("./world/model[@name='wall_south']/pose")

    assert north_wall.text.split()[1] == '5.75'
    assert south_wall.text.split()[1] == '-5.75'


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


def test_world_loads_harmonic_sensor_systems():
    """The reused world should load rendering sensors and the IMU system."""
    root = ET.parse(WORLD_PATH).getroot()
    plugins = {
        plugin.attrib['name']: plugin
        for plugin in root.findall('./world/plugin')
    }

    sensors = plugins['gz::sim::systems::Sensors']
    assert sensors.attrib['filename'] == 'gz-sim-sensors-system'
    assert sensors.findtext('render_engine') == 'ogre2'
    imu = plugins['gz::sim::systems::Imu']
    assert imu.attrib['filename'] == 'gz-sim-imu-system'


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
    assert "LaunchConfiguration('world')" in launch_source
    assert "'gz_args': ['-r ', world_path]" in launch_source
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

    assert "'gz_args': ['-r ', world_path]" in launch_source
    assert "f'-r -s " not in launch_source
    assert '--headless-rendering' not in launch_source


def test_phase3_launch_reuses_world_and_spawns_robot():
    """The stage 3 launch should reuse phase 2 and spawn from Xacro."""
    assert SPAWN_LAUNCH_PATH.is_file()
    launch_source = SPAWN_LAUNCH_PATH.read_text(encoding='utf-8')

    assert "'phase2_world.launch.py'" in launch_source
    assert "launch_arguments={'world': world_path}.items()" in launch_source
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
    assert '(odometry_gz_topic, odom_ros_topic)' in launch_source
    assert "(joint_state_gz_topic, '/joint_states')" in launch_source
    assert 'gz.msgs.Pose_V' not in launch_source


def test_phase3_launch_starts_ros_odom_tf_broadcaster():
    """The stage 3 launch should publish odom TF from the ROS odometry."""
    launch_source = SPAWN_LAUNCH_PATH.read_text(encoding='utf-8')

    assert "package='resilient_nav_monitor'" in launch_source
    assert "executable='odom_tf_broadcaster'" in launch_source
    assert "name='odom_tf_broadcaster'" in launch_source
    assert "parameters=[{'use_sim_time': True}]" in launch_source
    assert 'condition=IfCondition(start_odom_tf_broadcaster)' in launch_source
    assert "'odom_ros_topic'" in launch_source
    assert "default_value='/odom'" in launch_source
    assert "'start_odom_tf_broadcaster'" in launch_source


def test_phase3_demo_reuses_spawn_chain_and_only_adds_rviz():
    """The demo should wrap the spawn chain without duplicate publishers."""
    assert DEMO_LAUNCH_PATH.is_file()
    launch_source = DEMO_LAUNCH_PATH.read_text(encoding='utf-8')

    assert "'phase3_spawn.launch.py'" in launch_source
    assert "package='rviz2'" in launch_source
    assert "executable='rviz2'" in launch_source
    assert 'condition=IfCondition(use_rviz)' in launch_source
    assert "default_value='true'" in launch_source
    assert "parameters=[{'use_sim_time': True}]" in launch_source
    assert "'odom_ros_topic': odom_ros_topic" in launch_source
    assert (
        "'start_odom_tf_broadcaster': start_odom_tf_broadcaster"
        in launch_source
    )
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


def test_phase4_sensor_bridge_is_strictly_gazebo_to_ros():
    """Only IMU and LaserScan should cross the phase 4 sensor bridge."""
    with SENSOR_BRIDGE_PATH.open(encoding='utf-8') as config_file:
        bridge_config = yaml.safe_load(config_file)

    assert bridge_config == [
        {
            'ros_topic_name': '/imu/data',
            'gz_topic_name': '/imu/data',
            'ros_type_name': 'sensor_msgs/msg/Imu',
            'gz_type_name': 'gz.msgs.IMU',
            'direction': 'GZ_TO_ROS',
            'qos_profile': 'SENSOR_DATA',
        },
        {
            'ros_topic_name': '/scan',
            'gz_topic_name': '/scan',
            'ros_type_name': 'sensor_msgs/msg/LaserScan',
            'gz_type_name': 'gz.msgs.LaserScan',
            'direction': 'GZ_TO_ROS',
            'qos_profile': 'SENSOR_DATA',
        },
    ]
    assert all(
        entry['ros_type_name'] != 'sensor_msgs/msg/PointCloud2'
        for entry in bridge_config
    )


def test_phase4_demo_reuses_phase3_and_only_adds_sensor_bridge():
    """The stage 4 launch should wrap stage 3 without duplicating it."""
    assert PHASE4_LAUNCH_PATH.is_file()
    launch_source = PHASE4_LAUNCH_PATH.read_text(encoding='utf-8')

    assert "'phase3_demo.launch.py'" in launch_source
    assert "'world': world_path" in launch_source
    assert "name='sensor_bridge'" in launch_source
    assert "'phase4_sensor_bridge.yaml'" in launch_source
    assert "'odom_ros_topic': odom_ros_topic" in launch_source
    assert (
        "'start_odom_tf_broadcaster': start_odom_tf_broadcaster"
        in launch_source
    )
    assert "package='robot_state_publisher'" not in launch_source
    assert "executable='create'" not in launch_source
    assert "executable='odom_tf_broadcaster'" not in launch_source
    assert 'sensor_msgs/msg/PointCloud2' not in launch_source


def test_rgbd_bridge_maps_native_topics_to_four_stable_ros_topics():
    """RGB-D images and shared calibration should cross GZ to ROS only."""
    with RGBD_BRIDGE_PATH.open(encoding='utf-8') as config_file:
        bridge_config = yaml.safe_load(config_file)

    assert bridge_config == [
        {
            'ros_topic_name': '/camera/color/image_raw',
            'gz_topic_name': '/camera/image',
            'ros_type_name': 'sensor_msgs/msg/Image',
            'gz_type_name': 'gz.msgs.Image',
            'direction': 'GZ_TO_ROS',
            'qos_profile': 'SENSOR_DATA',
        },
        {
            'ros_topic_name': '/camera/color/camera_info',
            'gz_topic_name': '/camera/camera_info',
            'ros_type_name': 'sensor_msgs/msg/CameraInfo',
            'gz_type_name': 'gz.msgs.CameraInfo',
            'direction': 'GZ_TO_ROS',
            'qos_profile': 'SENSOR_DATA',
        },
        {
            'ros_topic_name': '/camera/depth/image_raw',
            'gz_topic_name': '/camera/depth_image',
            'ros_type_name': 'sensor_msgs/msg/Image',
            'gz_type_name': 'gz.msgs.Image',
            'direction': 'GZ_TO_ROS',
            'qos_profile': 'SENSOR_DATA',
        },
        {
            'ros_topic_name': '/camera/depth/camera_info',
            'gz_topic_name': '/camera/camera_info',
            'ros_type_name': 'sensor_msgs/msg/CameraInfo',
            'gz_type_name': 'gz.msgs.CameraInfo',
            'direction': 'GZ_TO_ROS',
            'qos_profile': 'SENSOR_DATA',
        },
    ]
    assert all(
        entry['ros_type_name'] != 'sensor_msgs/msg/PointCloud2'
        for entry in bridge_config
    )


def test_rgbd_demo_reuses_imu_lidar_demo_and_loads_sensor_rviz():
    """The RGB-D demo should extend, not duplicate, the existing chain."""
    assert RGBD_LAUNCH_PATH.is_file()
    launch_source = RGBD_LAUNCH_PATH.read_text(encoding='utf-8')

    assert "'phase4_imu_lidar_demo.launch.py'" in launch_source
    assert "'use_rviz': 'false'" in launch_source
    assert "name='rgbd_bridge'" in launch_source
    assert "'phase4_rgbd_bridge.yaml'" in launch_source
    assert "'phase4_sensors.rviz'" in launch_source
    assert "package='rviz2'" in launch_source
    assert "'use_sensor_rviz'" in launch_source
    assert "default_value='true'" in launch_source
    assert "parameters=[{'use_sim_time': True}]" in launch_source
    assert "package='robot_state_publisher'" not in launch_source
    assert "executable='create'" not in launch_source
    assert "executable='odom_tf_broadcaster'" not in launch_source
    assert 'sensor_msgs/msg/PointCloud2' not in launch_source
    assert "'odom_ros_topic': odom_ros_topic" in launch_source
    assert (
        "'start_odom_tf_broadcaster': start_odom_tf_broadcaster"
        in launch_source
    )


def test_phase4_rviz_has_required_sensor_displays_and_qos():
    """The dedicated RViz config should show the accepted sensor baseline."""
    assert PHASE4_RVIZ_PATH.is_file()
    with PHASE4_RVIZ_PATH.open(encoding='utf-8') as config_file:
        rviz_config = yaml.safe_load(config_file)

    manager = rviz_config['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == 'odom'
    displays = {
        display['Name']: display
        for display in manager['Displays']
    }

    assert displays['RobotModel']['Class'] == (
        'rviz_default_plugins/RobotModel'
    )
    assert displays['RobotModel']['Enabled'] is True
    assert displays['TF']['Class'] == 'rviz_default_plugins/TF'
    assert displays['TF']['Enabled'] is True

    laser = displays['LaserScan']
    assert laser['Class'] == 'rviz_default_plugins/LaserScan'
    assert laser['Enabled'] is True
    assert laser['Topic']['Value'] == '/scan'
    assert laser['Topic']['Reliability Policy'] == 'Best Effort'

    color = displays['Color Image']
    assert color['Class'] == 'rviz_default_plugins/Image'
    assert color['Enabled'] is True
    assert color['Topic']['Value'] == '/camera/color/image_raw'

    depth = displays['Depth Image (manual)']
    assert depth['Class'] == 'rviz_default_plugins/Image'
    assert depth['Enabled'] is False
    assert depth['Topic']['Value'] == '/camera/depth/image_raw'

    filtered_odom = displays['Filtered Odometry']
    assert filtered_odom['Class'] == 'rviz_default_plugins/Odometry'
    assert filtered_odom['Enabled'] is True
    assert filtered_odom['Topic']['Value'] == '/odometry/filtered'

    raw_odom = displays['Raw Wheel Odometry']
    assert raw_odom['Class'] == 'rviz_default_plugins/Odometry'
    assert raw_odom['Enabled'] is False
    assert raw_odom['Topic']['Value'] == '/wheel/odometry'


def test_rgbd_launch_and_rviz_are_installed_by_package_directory_rule():
    """The CMake rule should install directories containing both files."""
    cmake_source = CMAKE_PATH.read_text(encoding='utf-8')

    assert 'DIRECTORY config launch rviz worlds' in cmake_source
    assert 'scripts/phase9_mapping_route.py' in cmake_source
    assert 'scripts/motion_safety.py' in cmake_source
    assert RGBD_LAUNCH_PATH.parent.name == 'launch'
    assert PHASE4_RVIZ_PATH.parent.name == 'rviz'


def test_stage3_defaults_remain_unchanged_after_ekf_parameterization():
    """The EKF options must preserve the standalone stage 3 defaults."""
    launch_source = SPAWN_LAUNCH_PATH.read_text(encoding='utf-8')

    assert "(cmd_vel_gz_topic, '/cmd_vel')" in launch_source
    assert '(odometry_gz_topic, odom_ros_topic)' in launch_source
    assert "(joint_state_gz_topic, '/joint_states')" in launch_source
    assert "executable='odom_tf_broadcaster'" in launch_source
    assert "default_value='/odom'" in launch_source
    assert "default_value='true'" in launch_source
    assert '/wheel/odometry' not in launch_source
    assert 'ekf' not in launch_source.lower()

    sensor_bridge_source = PHASE4_LAUNCH_PATH.read_text(encoding='utf-8')
    assert "'phase4_sensor_bridge.yaml'" in sensor_bridge_source
    with SENSOR_BRIDGE_PATH.open(encoding='utf-8') as config_file:
        sensor_bridge = yaml.safe_load(config_file)
    assert [entry['ros_topic_name'] for entry in sensor_bridge] == [
        '/imu/data',
        '/scan',
    ]
