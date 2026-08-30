"""Static contracts for the Phase 10 Global Costmap smoke."""

import ast
import math
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SOURCE = PACKAGE_ROOT.parent
CONFIG_FILE = PACKAGE_ROOT / 'config' / 'nav2_costmaps.yaml'
LAUNCH_FILE = PACKAGE_ROOT / 'launch' / 'phase10_global_costmap_smoke.launch.py'
RVIZ_FILE = PACKAGE_ROOT / 'rviz' / 'phase10_global_costmap.rviz'
PROBE_FILE = PACKAGE_ROOT / 'global_costmap_probe.py'
XACRO_FILE = (
    WORKSPACE_SOURCE / 'resilient_nav_description' / 'urdf' / 'resilient_nav_robot.urdf.xacro'
)

EXPECTED_FOOTPRINT = (
    (-0.35, -0.175),
    (-0.10, -0.215),
    (0.10, -0.215),
    (0.15, -0.175),
    (0.15, 0.175),
    (0.10, 0.215),
    (-0.10, 0.215),
    (-0.35, 0.175),
)


def global_parameters():
    with CONFIG_FILE.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)['global_costmap']['global_costmap']['ros__parameters']


def local_parameters():
    with CONFIG_FILE.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)['local_costmap']['local_costmap']['ros__parameters']


def signed_area(points):
    return sum(
        x1 * y2 - x2 * y1
        for (x1, y1), (x2, y2) in zip(points, points[1:] + points[:1])
    ) / 2.0


def test_footprint_is_the_collision_hull_in_base_footprint_coordinates():
    """Freeze the Xacro-derived chassis, wheel, and caster silhouette."""
    global_config = global_parameters()
    local_config = local_parameters()
    footprint = tuple(tuple(point) for point in yaml.safe_load(global_config['footprint']))
    assert footprint == EXPECTED_FOOTPRINT
    assert signed_area(footprint) > 0.0
    assert global_config['footprint_padding'] == 0.01
    assert local_config['footprint'] == global_config['footprint']
    assert local_config['footprint_padding'] == global_config['footprint_padding']
    assert 'robot_radius' not in global_config
    assert 'robot_radius' not in local_config

    xacro = XACRO_FILE.read_text(encoding='utf-8')
    for required in (
        'name="base_length" value="0.50"',
        'name="base_width" value="0.35"',
        'name="wheel_radius" value="0.10"',
        'name="wheel_width" value="0.04"',
        'name="wheel_x" value="0.10"',
        'name="caster_radius" value="0.05"',
        'name="caster_x" value="-0.20"',
        'name="base_footprint_joint" type="fixed"',
        'xyz="${-wheel_x} 0 ${base_z}"',
    ):
        assert required in xacro


def test_global_costmap_uses_only_the_three_frozen_nav2_layers():
    """No Local Costmap, planner, controller, or custom fusion is introduced."""
    config = global_parameters()
    assert config['global_frame'] == 'map'
    assert config['robot_base_frame'] == 'base_footprint'
    assert config['rolling_window'] is False
    assert math.isclose(config['resolution'], 0.05)
    assert config['track_unknown_space'] is True
    assert config['update_frequency'] == 1.0
    assert config['publish_frequency'] == 1.0
    assert config['plugins'] == ['static_layer', 'obstacle_layer', 'inflation_layer']
    assert config['static_layer'] == {
        'plugin': 'nav2_costmap_2d::StaticLayer',
        'map_topic': '/map',
        'map_subscribe_transient_local': True,
    }
    scan = config['obstacle_layer']['scan']
    assert config['obstacle_layer']['plugin'] == 'nav2_costmap_2d::ObstacleLayer'
    assert config['obstacle_layer']['observation_sources'] == 'scan'
    assert scan['topic'] == '/scan'
    assert scan['data_type'] == 'LaserScan'
    assert scan['marking'] is True and scan['clearing'] is True
    assert scan['obstacle_max_range'] < scan['raytrace_max_range']
    assert 'observation_persistence' not in scan
    assert scan['expected_update_rate'] == 0.2
    assert scan['sensor_frame'] == 'lidar_link'
    assert config['inflation_layer'] == {
        'plugin': 'nav2_costmap_2d::InflationLayer',
        'inflation_radius': 0.55,
        'cost_scaling_factor': 3.0,
    }
    assert config['always_send_full_costmap'] is True

    source = CONFIG_FILE.read_text(encoding='utf-8').lower()
    for forbidden in ('planner', 'controller', 'cmd_vel', 'fault', 'ground_truth'):
        assert forbidden not in source


def test_global_costmap_launch_reuses_task1_and_scopes_its_rviz_override():
    """The new RViz flag must remain independent of the nested Task 1 launch."""
    source = LAUNCH_FILE.read_text(encoding='utf-8')
    tree = ast.parse(source)
    for required in (
        "'phase10_localization_smoke.launch.py'",
        "package='nav2_costmap_2d'",
        "executable='nav2_costmap_2d'",
        "namespace='global_costmap'",
        "name='global_costmap'",
        "package='nav2_lifecycle_manager'",
        "'bond_timeout': 0.0",
        "'node_names': ['global_costmap']",
        "'phase10_global_costmap.rviz'",
    ):
        assert required in source
    for forbidden in ('planner_server', 'controller_server', 'bt_navigator', 'slam_toolbox', '/cmd_vel'):
        assert forbidden not in source.lower()

    groups = [
        call for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == 'GroupAction'
    ]
    assert len(groups) == 1
    keywords = {keyword.arg: keyword.value for keyword in groups[0].keywords}
    assert isinstance(keywords['scoped'], ast.Constant) and keywords['scoped'].value is True
    assert isinstance(keywords['forwarding'], ast.Constant) and keywords['forwarding'].value is True


def test_global_costmap_rviz_and_probe_expose_only_costmap_observability():
    """Keep visualization and probe read-only and focused on the Task 2 chain."""
    rviz = RVIZ_FILE.read_text(encoding='utf-8')
    for required in (
        'Fixed Frame: map', 'Value: /map', 'Value: /scan',
        'Value: /global_costmap/costmap',
        'Value: /global_costmap/published_footprint',
        'rviz_default_plugins/Polygon',
    ):
        assert required in rviz

    probe = PROBE_FILE.read_text(encoding='utf-8')
    for required in (
        "'/global_costmap/global_costmap/get_state'",
        "'/global_costmap/costmap'",
        "'/global_costmap/published_footprint'",
        "'map', 'base_footprint'",
        "'lidar_link'",
        'validate_costmap',
        'validate_footprint',
        'scan_map_sanity',
        '--skip-scan-map-sanity',
        'Time.from_msg(scan.header.stamp)',
    ):
        assert required in probe
    for forbidden in ('create_publisher', 'cmd_vel', 'set_parameters', 'fault_injection'):
        assert forbidden not in probe.lower()
