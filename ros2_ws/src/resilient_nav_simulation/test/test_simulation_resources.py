"""Static tests for the phase 2 simulation resources."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORLD_PATH = PACKAGE_ROOT / 'worlds' / 'phase2_world.sdf'
BRIDGE_PATH = PACKAGE_ROOT / 'config' / 'bridge.yaml'
LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'phase2_world.launch.py'


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
    assert "get_package_share_directory('resilient_nav_simulation')" in launch_source
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
