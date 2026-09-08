"""Static contracts for the healthy Slam Toolbox integration resources."""

import ast
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_FILE = PACKAGE_ROOT / 'config' / 'mapper_params_online_async.yaml'
LOCALIZATION_CONFIG_FILE = PACKAGE_ROOT / 'config' / 'mapper_params_localization.yaml'
LAUNCH_FILE = PACKAGE_ROOT / 'launch' / 'phase9_online_async_mapping.launch.py'
LOCALIZATION_LAUNCH_FILE = PACKAGE_ROOT / 'launch' / 'phase9_localization.launch.py'
SMOKE_LAUNCH_FILE = (
    PACKAGE_ROOT / 'launch' / 'phase9_healthy_mapping_smoke.launch.py'
)
LOCALIZATION_SMOKE_LAUNCH_FILE = (
    PACKAGE_ROOT / 'launch' / 'phase9_healthy_localization_smoke.launch.py'
)
RVIZ_FILE = PACKAGE_ROOT / 'rviz' / 'phase9_mapping.rviz'
MAPS_README = PACKAGE_ROOT / 'maps' / 'README.md'
PHASE9_MAPS = PACKAGE_ROOT / 'maps' / 'phase9'
SETUP_FILE = PACKAGE_ROOT / 'setup.py'
PACKAGE_XML = PACKAGE_ROOT / 'package.xml'
PROBE_FILE = PACKAGE_ROOT / 'resilient_nav_slam' / 'slam_probe.py'
POSE_ADAPTER_FILE = PACKAGE_ROOT / 'resilient_nav_slam' / 'slam_pose_adapter.py'
MAP_ADAPTER_FILE = PACKAGE_ROOT / 'resilient_nav_slam' / 'slam_map_adapter.py'
EVALUATOR_FILE = PACKAGE_ROOT / 'resilient_nav_slam' / 'slam_evaluator_node.py'
EVALUATION_LAUNCH_FILE = PACKAGE_ROOT / 'launch' / 'phase9_evaluation.launch.py'


def load_parameters():
    """Load the installed Jazzy-schema configuration under its node key."""
    with CONFIG_FILE.open('r', encoding='utf-8') as config_stream:
        config = yaml.safe_load(config_stream)
    return config['slam_toolbox']['ros__parameters']


def load_localization_parameters():
    """Load the localization configuration under its node key."""
    with LOCALIZATION_CONFIG_FILE.open('r', encoding='utf-8') as config_stream:
        config = yaml.safe_load(config_stream)
    return config['slam_toolbox']['ros__parameters']


def test_mapping_configuration_freezes_the_healthy_frame_and_scan_contract():
    """Keep the sole SLAM input and TF frames aligned with Phase 9 architecture."""
    parameters = load_parameters()

    assert parameters['use_sim_time'] is True
    assert parameters['map_frame'] == 'map'
    assert parameters['odom_frame'] == 'odom'
    assert parameters['base_frame'] == 'base_footprint'
    assert parameters['scan_topic'] == '/scan'
    assert parameters['mode'] == 'mapping'
    assert parameters['transform_publish_period'] > 0.0


def test_mapping_configuration_conservatively_preserves_local_jazzy_baseline():
    """Guard representative non-tuned values from the verified local baseline."""
    parameters = load_parameters()

    assert parameters['solver_plugin'] == 'solver_plugins::CeresSolver'
    assert parameters['map_update_interval'] == 5.0
    assert parameters['resolution'] == 0.05
    assert parameters['restamp_tf'] is False
    assert parameters['tf_buffer_duration'] == 30.0
    assert parameters['scan_buffer_size'] == 10
    assert parameters['do_loop_closing'] is True


def test_mapping_configuration_excludes_truth_health_fault_and_agent_inputs():
    """The healthy SLAM baseline must not receive protected side channels."""
    config_source = CONFIG_FILE.read_text(encoding='utf-8').lower()

    for forbidden in (
        'faultstatus',
        '/fault_injection/status',
        'scenario_id',
        'scenario_seed',
        '/evaluation/',
        'sensorhealth',
        'robot agent',
        '/faulted/',
        '/fusion/input/',
    ):
        assert forbidden not in config_source


def test_localization_configuration_preserves_the_mapping_contract():
    """Localization changes only mode; its pose-graph path is launch supplied."""
    mapping_parameters = load_parameters()
    localization_parameters = load_localization_parameters()

    for name in (
        'use_sim_time', 'map_frame', 'odom_frame', 'base_frame', 'scan_topic',
        'transform_publish_period', 'resolution', 'tf_buffer_duration',
    ):
        assert localization_parameters[name] == mapping_parameters[name]
    assert localization_parameters['mode'] == 'localization'
    assert localization_parameters['map_start_at_dock'] is True
    assert 'map_file_name' not in localization_parameters


def test_localization_launch_uses_package_share_posegraph_base_path():
    """Load M8 serialization by base path, never the display-only PGM/YAML."""
    launch_source = LOCALIZATION_LAUNCH_FILE.read_text(encoding='utf-8')
    ast.parse(launch_source)

    assert "get_package_share_directory('resilient_nav_slam')" in launch_source
    assert "'localization_slam_toolbox_node'" in launch_source
    assert "'mapper_params_localization.yaml'" in launch_source
    assert "'map_file_name'" in launch_source
    assert "'maps' / 'phase9' / 'posegraph' / 'phase9_posegraph'" in launch_source
    assert 'phase9_map.pgm' not in launch_source
    assert 'phase9_map.yaml' not in launch_source
    assert '/home/kylian' not in launch_source


def test_localization_launch_supports_an_explicit_start_pose_before_scans():
    """M11 may supply a safe map-frame guess without changing M10 defaults."""
    launch_source = LOCALIZATION_LAUNCH_FILE.read_text(encoding='utf-8')

    assert 'OpaqueFunction(function=_localization_actions)' in launch_source
    assert "'use_start_pose'" in launch_source
    assert "'map_start_at_dock': False" in launch_source
    assert "'map_start_pose': start_pose" in launch_source
    assert "float(LaunchConfiguration('initial_pose_x').perform(context))" in launch_source
    assert "float(LaunchConfiguration('initial_pose_y').perform(context))" in launch_source
    assert "float(LaunchConfiguration('initial_pose_yaw').perform(context))" in launch_source


def test_localization_smoke_forwards_spawn_and_optional_start_pose():
    """Different safe spawns retain M10 defaults unless M11 enables a guess."""
    launch_source = LOCALIZATION_SMOKE_LAUNCH_FILE.read_text(encoding='utf-8')

    for argument in (
        'spawn_x', 'spawn_y', 'spawn_yaw', 'use_start_pose',
        'initial_pose_x', 'initial_pose_y', 'initial_pose_yaw',
    ):
        assert "LaunchConfiguration('" + argument + "')" in launch_source
        assert "'" + argument + "': " in launch_source
    assert "default_value='false'" in launch_source


def test_localization_smoke_preserves_the_healthy_tf_owners():
    """The static localization smoke must not reintroduce legacy TF owners."""
    launch_source = LOCALIZATION_SMOKE_LAUNCH_FILE.read_text(encoding='utf-8')
    ast.parse(launch_source)

    assert "'phase4_imu_lidar_demo.launch.py'" in launch_source
    assert "'phase9_localization.launch.py'" in launch_source
    assert "'start_odom_tf_broadcaster': 'false'" in launch_source
    assert "package='robot_localization'" in launch_source
    assert "executable='ekf_node'" in launch_source
    assert "package='rviz2'" in launch_source
    for forbidden in (
        "executable='odom_tf_broadcaster'", 'motion_test', 'initialpose',
        'fault_injection', 'sensorhealth', 'agent-core', 'ground_truth',
        'evaluator', 'nav2',
    ):
        assert forbidden not in launch_source.lower()


def test_minimal_launch_only_delegates_to_installed_online_async_with_config():
    """Do not combine Gazebo, EKF, RViz, probes, or evaluators into this launch."""
    launch_source = LAUNCH_FILE.read_text(encoding='utf-8')
    ast.parse(launch_source)

    assert "get_package_share_directory('slam_toolbox')" in launch_source
    assert "'online_async_launch.py'" in launch_source
    assert "'mapper_params_online_async.yaml'" in launch_source
    assert "'slam_params_file'" in launch_source
    assert "'use_sim_time': 'true'" in launch_source
    for forbidden in ('node(', 'from launch_ros'):
        assert forbidden not in launch_source.lower()


def test_smoke_launch_uses_only_existing_healthy_chain_and_frozen_tf_owners():
    """The smoke composition must not introduce a second odom or map owner."""
    launch_source = SMOKE_LAUNCH_FILE.read_text(encoding='utf-8')
    ast.parse(launch_source)

    assert "'phase4_imu_lidar_demo.launch.py'" in launch_source
    assert "'phase9_slam_world.sdf'" in launch_source
    assert "'world': world_path" in launch_source
    assert "default_value='-3.5'" in launch_source
    assert "'use_rviz': 'false'" in launch_source
    assert 'GroupAction(' in launch_source
    assert 'scoped=True' in launch_source
    assert "'odom_ros_topic': '/wheel/odometry/raw'" in launch_source
    assert "executable='wheel_odometry_uncertainty'" in launch_source
    assert "'start_odom_tf_broadcaster': 'false'" in launch_source
    assert "package='robot_localization'" in launch_source
    assert "executable='ekf_node'" in launch_source
    assert "'ekf.yaml'" in launch_source
    assert "'phase9_online_async_mapping.launch.py'" in launch_source
    assert "package='rviz2'" in launch_source
    assert "'phase9_mapping.rviz'" in launch_source
    assert "name='phase9_mapping_rviz'" in launch_source
    for forbidden in (
        "package='resilient_nav_monitor'",
        "executable='odom_tf_broadcaster'",
        'motion_test',
        'fault_injection',
        'sensorhealth',
        'agent-core',
        'ground_truth',
        'evaluator',
        'nav2',
    ):
        assert forbidden not in launch_source.lower()


def test_setup_installs_all_static_mapping_resources():
    """Install config, launch, RViz, and preserve nested maps resources."""
    setup_source = SETUP_FILE.read_text(encoding='utf-8')

    for resource_directory in ('config', 'launch', 'rviz'):
        assert "os.path.join('share', package_name, '" in setup_source
        assert "'" + resource_directory + "'" in setup_source
    assert "glob(os.path.join('config', '*.yaml'))" in setup_source
    assert "glob(os.path.join('launch', '*.launch.py'))" in setup_source
    assert "glob(os.path.join('rviz', '*.rviz'))" in setup_source
    assert 'def map_data_files()' in setup_source
    assert "os.walk('maps')" in setup_source
    assert "os.path.join('share', package_name, directory)" in setup_source
    assert 'os.path.join(directory, filename)' in setup_source


def test_rviz_and_maps_resources_describe_the_persisted_baseline():
    """RViz visualizes native SLAM state and maps retain the M8 baseline assets."""
    rviz_source = RVIZ_FILE.read_text(encoding='utf-8')
    maps_source = MAPS_README.read_text(encoding='utf-8').lower()

    assert 'Fixed Frame: map' in rviz_source
    assert 'Value: /map' in rviz_source
    assert 'Value: /scan' in rviz_source
    assert 'Reliability Policy: Best Effort' in rviz_source
    assert 'rviz_default_plugins/RobotModel' in rviz_source
    assert 'rviz_default_plugins/TF' in rviz_source
    assert 'slam_toolbox::SlamToolboxPlugin' in rviz_source
    assert 'phase9/occupancy' in maps_source
    assert 'phase9/posegraph' in maps_source
    assert 'ground truth' in maps_source


def test_persisted_phase9_map_resources_are_nonempty():
    """Keep the saved map and pose-graph assets available as package resources."""
    expected_assets = (
        PHASE9_MAPS / 'metadata.yaml',
        PHASE9_MAPS / 'occupancy' / 'phase9_map.yaml',
        PHASE9_MAPS / 'occupancy' / 'phase9_map.pgm',
        PHASE9_MAPS / 'posegraph' / 'phase9_posegraph.data',
        PHASE9_MAPS / 'posegraph' / 'phase9_posegraph.posegraph',
    )

    for asset in expected_assets:
        assert asset.is_file()
        assert asset.stat().st_size > 0


def test_package_declares_the_external_runtime_and_static_test_dependency():
    """This package is an integration layer, with no copied Slam Toolbox code."""
    package_source = PACKAGE_XML.read_text(encoding='utf-8')

    assert '<exec_depend>slam_toolbox</exec_depend>' in package_source
    assert '<exec_depend>resilient_nav_localization</exec_depend>' in package_source
    assert '<exec_depend>resilient_nav_fusion</exec_depend>' in package_source
    assert '<exec_depend>resilient_nav_simulation</exec_depend>' in package_source
    assert '<exec_depend>robot_localization</exec_depend>' in package_source
    assert '<exec_depend>nav_msgs</exec_depend>' in package_source
    assert '<exec_depend>rclpy</exec_depend>' in package_source
    assert '<exec_depend>sensor_msgs</exec_depend>' in package_source
    assert '<exec_depend>std_msgs</exec_depend>' in package_source
    assert '<exec_depend>tf2_ros</exec_depend>' in package_source
    assert '<test_depend>python3-pytest</test_depend>' in package_source
    assert 'slam_probe = resilient_nav_slam.slam_probe:main' in (
        SETUP_FILE.read_text(encoding='utf-8')
    )
    for entry_point in (
        'slam_pose_adapter = resilient_nav_slam.slam_pose_adapter:main',
        'slam_map_adapter = resilient_nav_slam.slam_map_adapter:main',
        'slam_evaluator = resilient_nav_slam.slam_evaluator_node:main',
    ):
        assert entry_point in SETUP_FILE.read_text(encoding='utf-8')


def test_evaluation_boundary_is_read_only_and_kept_out_of_slam_core_launches():
    """Ground Truth may enter only the separate /evaluation/* composition."""
    evaluation_source = EVALUATION_LAUNCH_FILE.read_text(encoding='utf-8').lower()
    mapping_source = SMOKE_LAUNCH_FILE.read_text(encoding='utf-8').lower()
    localization_source = LOCALIZATION_SMOKE_LAUNCH_FILE.read_text(encoding='utf-8').lower()

    assert "'phase8_ground_truth.launch.py'" in evaluation_source
    for executable in ('slam_pose_adapter', 'slam_map_adapter', 'slam_evaluator'):
        assert executable in evaluation_source
    for source in (mapping_source, localization_source):
        assert 'ground_truth' not in source
        assert 'evaluator' not in source
        assert '/evaluation/' not in source

    for adapter_source in (
        POSE_ADAPTER_FILE.read_text(encoding='utf-8').lower(),
        MAP_ADAPTER_FILE.read_text(encoding='utf-8').lower(),
    ):
        assert 'create_publisher' in adapter_source
        assert 'create_subscription' not in adapter_source or 'input_topic' in adapter_source
        assert 'set_parameters' not in adapter_source
        assert 'cmd_vel' not in adapter_source
    evaluator_source = EVALUATOR_FILE.read_text(encoding='utf-8').lower()
    assert "'/evaluation/ground_truth_pose'" in evaluator_source
    assert "'/evaluation/slam_pose'" in evaluator_source
    assert "'/evaluation/slam_map_metrics'" in evaluator_source
    assert "'/map'" not in evaluator_source
    assert 'create_publisher' in evaluator_source
    assert 'lookup_transform' not in evaluator_source


def test_slam_probe_is_read_only_and_observes_only_the_frozen_mapping_chain():
    """Keep the probe out of TF ownership, parameter writes, and evaluation."""
    probe_source = PROBE_FILE.read_text(encoding='utf-8')
    ast.parse(probe_source)

    assert "'/scan'" in probe_source
    assert "'/map'" in probe_source
    assert "lookup_transform" in probe_source
    assert "json.dumps" in probe_source
    assert "Parameter('use_sim_time', Parameter.Type.BOOL, True)" in probe_source
    assert 'if rclpy.ok():' in probe_source
    assert 'create_publisher' not in probe_source
    assert 'set_parameters' not in probe_source
    for forbidden in (
        'faultstatus',
        'sensorhealth',
        'ground_truth',
        'evaluator',
        'pose_graph',
        'cmd_vel',
    ):
        assert forbidden not in probe_source.lower()
