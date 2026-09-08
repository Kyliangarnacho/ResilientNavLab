"""Static contracts for the Phase 10 Nav2 localization integration."""

import ast
import hashlib
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SOURCE = PACKAGE_ROOT.parent
CONFIG_FILE = PACKAGE_ROOT / 'config' / 'nav2_localization.yaml'
LAUNCH_FILE = PACKAGE_ROOT / 'launch' / 'phase10_localization.launch.py'
RVIZ_FILE = PACKAGE_ROOT / 'rviz' / 'phase10_localization.rviz'
MAP_SERVER_LAUNCH_FILE = PACKAGE_ROOT / 'launch' / 'phase10_map_server_smoke.launch.py'
SMOKE_LAUNCH_FILE = PACKAGE_ROOT / 'launch' / 'phase10_localization_smoke.launch.py'
EVALUATION_LAUNCH_FILE = PACKAGE_ROOT / 'launch' / 'phase10_amcl_evaluation.launch.py'
PACKAGE_XML = PACKAGE_ROOT / 'package.xml'
SETUP_FILE = PACKAGE_ROOT / 'setup.py'
PHASE9_MAP_ROOT = WORKSPACE_SOURCE / 'resilient_nav_slam' / 'maps' / 'phase9'

FROZEN_ASSET_HASHES = {
    'occupancy/phase9_map.pgm': (
        '548a37d56084ca7a804c96341ef2784f45d22ec4772f5819ea4cd981fa8c3161'
    ),
    'occupancy/phase9_map.yaml': (
        '21499e0fcd079f11a276832ec4622bb1c69a0889dfa9cf7820c08b8c93e45161'
    ),
    'posegraph/phase9_posegraph.data': (
        '586c28fa47cc5def621eeaecd66e564861e10e6faad1b69c5368b1d60e018872'
    ),
    'posegraph/phase9_posegraph.posegraph': (
        'ee7152d9973ed87c26df2dd767fea495173941a9787a70c2cbb70dc6d74b6f20'
    ),
}


def localization_parameters():
    """Return the minimal localization-only Nav2 parameter tree."""
    with CONFIG_FILE.open(encoding='utf-8') as config_stream:
        return yaml.safe_load(config_stream)


def sha256(path):
    """Compute one source resource identity without modifying it."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_localization_config_freezes_only_map_server_and_amcl_contracts():
    """Task 1.2 must not silently introduce later Nav2 servers."""
    config = localization_parameters()

    assert set(config) == {'amcl', 'map_server'}
    amcl = config['amcl']['ros__parameters']
    map_server = config['map_server']['ros__parameters']

    assert map_server == {'frame_id': 'map', 'topic_name': '/map'}
    assert amcl['base_frame_id'] == 'base_footprint'
    assert amcl['odom_frame_id'] == 'odom'
    assert amcl['global_frame_id'] == 'map'
    assert amcl['scan_topic'] == '/scan'
    assert amcl['map_topic'] == '/map'
    assert amcl['robot_model_type'] == 'nav2_amcl::DifferentialMotionModel'
    assert amcl['laser_model_type'] == 'likelihood_field'
    assert amcl['tf_broadcast'] is True
    assert amcl['set_initial_pose'] is False
    assert amcl['always_reset_initial_pose'] is True
    assert amcl['save_pose_rate'] == -1.0
    assert 'yaml_filename' not in map_server


def test_localization_config_excludes_control_and_protected_inputs():
    """Healthy localization must not receive truth, health, or control inputs."""
    config_source = CONFIG_FILE.read_text(encoding='utf-8').lower()

    for forbidden in (
        '/cmd_vel', 'faultstatus', 'fault_injection', 'scenario_id',
        'scenario_seed', '/evaluation/', '/faulted/', '/fusion/',
        'sensorhealth', 'agent', 'planner', 'controller', 'costmap',
    ):
        assert forbidden not in config_source


def test_localization_launch_delegates_to_installed_nav2_bringup():
    """Keep Nav2's Map Server, AMCL, and lifecycle implementation upstream."""
    launch_source = LAUNCH_FILE.read_text(encoding='utf-8')
    ast.parse(launch_source)

    for package_name in (
        'nav2_bringup', 'resilient_nav_navigation', 'resilient_nav_slam',
    ):
        assert "get_package_share_directory('" + package_name + "')" in launch_source
    assert "'localization_launch.py'" in launch_source
    assert "'phase9_map.yaml'" in launch_source
    assert "'nav2_localization.yaml'" in launch_source
    assert "'use_sim_time'" in launch_source
    assert "default_value='true'" in launch_source
    assert "'use_composition'" in launch_source
    assert "'container_name'" in launch_source
    assert "default_value='False'" in launch_source
    assert "package='rviz2'" in launch_source

    for forbidden in (
        "package='nav2_map_server'", "package='nav2_amcl'",
        "package='nav2_lifecycle_manager'", 'slam_toolbox',
        'phase9_localization.launch.py', 'motion_test', 'ground_truth',
        'fault_injection', 'sensorhealth', 'agent-core', '/cmd_vel',
    ):
        assert forbidden not in launch_source.lower()


def test_phase9_saved_assets_retain_their_frozen_identities():
    """Phase 10 consumes the Phase 9 map; it must not replace it."""
    for relative_path, expected_hash in FROZEN_ASSET_HASHES.items():
        assert sha256(PHASE9_MAP_ROOT / relative_path) == expected_hash


def test_rviz_is_localization_only_and_supports_explicit_initial_pose():
    """Keep the first RViz view focused on localization, not navigation."""
    rviz_source = RVIZ_FILE.read_text(encoding='utf-8')

    for expected in (
        'Fixed Frame: map', 'Value: /map', 'Value: /scan',
        'Value: /amcl_pose', 'Value: /particle_cloud',
        'rviz_default_plugins/SetInitialPose', 'Value: /initialpose',
        'nav2_rviz_plugins/ParticleCloud', 'Durability Policy: Transient Local',
    ):
        assert expected in rviz_source
    assert 'rviz_default_plugins/PoseArray' not in rviz_source
    for forbidden in ('Navigation2 Goal', 'Path', 'Costmap', 'cmd_vel'):
        assert forbidden not in rviz_source


def test_map_server_smoke_isolated_from_amcl_and_uses_one_node_lifecycle_manager():
    """Task 1.3 must validate the static map before localization starts."""
    source = MAP_SERVER_LAUNCH_FILE.read_text(encoding='utf-8')
    ast.parse(source)

    for required in (
        "package='nav2_map_server'", "executable='map_server'",
        "package='nav2_lifecycle_manager'", "'node_names': ['map_server']",
        "'yaml_filename': map_yaml", "default_value='false'",
    ):
        assert required in source
    for forbidden in (
        "package='nav2_amcl'", 'phase10_localization.launch.py',
        'slam_toolbox', 'planner', 'controller', 'costmap', '/cmd_vel',
    ):
        assert forbidden not in source.lower()


def test_localization_smoke_reuses_healthy_inputs_and_excludes_duplicate_tf_owners():
    """Task 1.4 composes the verified sensor/EKF chain without SLAM TF."""
    source = SMOKE_LAUNCH_FILE.read_text(encoding='utf-8')
    ast.parse(source)

    for required in (
        "'phase4_imu_lidar_demo.launch.py'", "'phase10_localization.launch.py'",
        "'start_odom_tf_broadcaster': 'false'", "package='robot_localization'",
        "'odom_ros_topic': '/wheel/odometry/raw'",
        "executable='wheel_odometry_uncertainty'",
        "executable='phase10_initial_pose_helper'", "default_value='-3.5'",
        "default_value='0.0'",
    ):
        assert required in source
    for forbidden in (
        'slam_toolbox', 'phase9_localization.launch.py',
        "executable='odom_tf_broadcaster'",
        'planner', 'controller', 'costmap', 'navigate_to_pose', '/cmd_vel',
        'ground_truth', '/evaluation/',
    ):
        assert forbidden not in source.lower()


def test_localization_smoke_scopes_phase4_use_rviz_override():
    """Phase 4's forced-off RViz setting must not leak into Nav2's include."""
    tree = ast.parse(SMOKE_LAUNCH_FILE.read_text(encoding='utf-8'))
    group_calls = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == 'GroupAction'
    ]
    assert len(group_calls) == 1
    group = group_calls[0]
    keywords = {keyword.arg: keyword.value for keyword in group.keywords}
    assert isinstance(keywords['scoped'], ast.Constant)
    assert keywords['scoped'].value is True
    assert isinstance(keywords['forwarding'], ast.Constant)
    assert keywords['forwarding'].value is True
    assert isinstance(keywords['actions'], ast.List)
    assert isinstance(keywords['actions'].elts[0], ast.Name)
    assert keywords['actions'].elts[0].id == 'healthy_inputs'

    nav2_use_rviz = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Dict)
        for key, value in zip(node.keys, node.values)
        if isinstance(key, ast.Constant)
        and key.value == 'use_rviz'
        and isinstance(value, ast.Name)
        and value.id == 'use_rviz'
    ]
    assert len(nav2_use_rviz) == 1


def test_initial_pose_helper_accepts_launch_ros_arguments():
    """The helper must tolerate launch_ros's appended --ros-args block."""
    source = (PACKAGE_ROOT / 'initial_pose_helper.py').read_text(encoding='utf-8')
    assert 'parse_known_args' in source
    assert "declare_parameter('use_sim_time'" not in source
    assert "get_clock().now().nanoseconds <= 0" in source


def test_runtime_tools_do_not_redeclare_ros_builtin_use_sim_time():
    """Jazzy provides use_sim_time; declaring it again fails in launched tools."""
    for name in ('initial_pose_helper.py', 'localization_probe.py', 'amcl_evaluator.py'):
        source = (PACKAGE_ROOT / name).read_text(encoding='utf-8')
        assert "declare_parameter('use_sim_time'" not in source


def test_evaluation_isolated_from_the_localization_runtime():
    """Ground Truth may enter only the dedicated evaluator launch."""
    source = EVALUATION_LAUNCH_FILE.read_text(encoding='utf-8')
    ast.parse(source)
    assert "'phase8_ground_truth.launch.py'" in source
    assert "executable='phase10_amcl_evaluator'" in source
    assert "'map_to_odom_x'" in source

    for primary in (LAUNCH_FILE, SMOKE_LAUNCH_FILE, CONFIG_FILE):
        assert '/evaluation/' not in primary.read_text(encoding='utf-8').lower()


def test_amcl_evaluator_imports_the_exported_trajectory_evaluator():
    """The evaluation launch must not fail on a stale function name."""
    source = (PACKAGE_ROOT / 'amcl_evaluator.py').read_text(encoding='utf-8')
    assert 'evaluate_persisted_map_localization_trajectory' in source
    assert 'evaluate_persisted_map_localization(' not in source


def test_package_metadata_and_install_rules_cover_all_static_resources():
    """The installed package must remain independently discoverable."""
    package_xml = PACKAGE_XML.read_text(encoding='utf-8')
    setup_source = SETUP_FILE.read_text(encoding='utf-8')

    for dependency in (
        'ament_index_python', 'launch', 'launch_ros', 'lifecycle_msgs', 'rcl_interfaces',
        'nav2_amcl', 'nav2_bringup', 'nav2_lifecycle_manager',
        'nav2_costmap_2d', 'nav2_map_server', 'nav2_msgs', 'nav2_rviz_plugins', 'rclpy',
        'resilient_nav_fusion', 'resilient_nav_localization',
        'resilient_nav_simulation', 'resilient_nav_slam', 'robot_localization',
        'rosgraph_msgs', 'tf2_msgs', 'tf2_ros', 'rviz2', 'python3-pytest', 'python3-yaml',
    ):
        assert dependency in package_xml
    assert '<build_type>ament_python</build_type>' in package_xml
    for resource_directory in ('config', 'launch', 'rviz'):
        assert "os.path.join('share', package_name, '" in setup_source
        assert "'" + resource_directory + "'" in setup_source
    for console_script in (
        'phase10_map_server_probe', 'phase10_initial_pose_helper',
        'phase10_localization_probe', 'phase10_amcl_evaluator',
        'phase10_global_costmap_probe', 'phase10_local_costmap_probe',
        'phase10_costmap_joint_probe', 'phase10_costmap_experiment_evaluator',
    ):
        assert console_script in setup_source
    assert "py_modules=[" in setup_source


def test_localization_probe_records_authoritative_dynamic_tf_edges():
    """Probe evidence must separate AMCL map→odom from EKF odom→base."""
    source = (PACKAGE_ROOT / 'localization_probe.py').read_text(encoding='utf-8')
    assert "('map', 'odom')" in source
    assert "('odom', 'base_footprint')" in source
    assert 'map_to_base_footprint_chain' in source
    assert '--allow-no-particles' in source
