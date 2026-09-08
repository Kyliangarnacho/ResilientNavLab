"""Static and pure-function checks for the two stationary AMCL test scenes."""

import ast
import xml.etree.ElementTree as ET
from pathlib import Path

from amcl_crowd_oscillator import oscillating_command, oscillating_offset

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CLEAN_LAUNCH = PACKAGE_ROOT / 'launch' / 'amcl_clean_baseline_test.launch.py'
MOVING_LAUNCH = PACKAGE_ROOT / 'launch' / 'amcl_moving_crowd_test.launch.py'
FIXED_LAUNCH = PACKAGE_ROOT / 'launch' / 'amcl_fixed_obstacles_test.launch.py'
RVIZ_FILE = PACKAGE_ROOT / 'rviz' / 'amcl_obstacle_test.rviz'
CROWD_MODEL = PACKAGE_ROOT / 'models' / 'amcl_crowd_pedestrian' / 'model.sdf'
FIXED_MODEL = PACKAGE_ROOT / 'models' / 'amcl_fixed_obstacle' / 'model.sdf'


@pytest.mark.parametrize(
    ('elapsed_sec', 'expected'),
    ((0.0, 0.25), (7.99, 0.25), (8.0, -0.25), (16.0, 0.25)),
)
def test_oscillator_reverses_without_a_stop_leg(elapsed_sec, expected):
    """The driver must reverse immediately rather than pause at an endpoint."""
    assert oscillating_command(elapsed_sec, 0.25, 8.0) == expected


@pytest.mark.parametrize(
    ('elapsed_sec', 'expected'),
    ((0.0, 0.0), (4.0, 1.0), (8.0, 2.0), (12.0, 1.0), (16.0, 0.0)),
)
def test_oscillator_pose_is_a_continuous_bounded_triangle_wave(
    elapsed_sec, expected
):
    """Pedestrians must move back and forth without an endpoint pause."""
    assert oscillating_offset(elapsed_sec, 0.25, 2.0) == expected


def test_obstacle_scenes_are_localization_only_and_independent_of_brne():
    """Neither observation scene may launch navigation or an old BRNE scene."""
    for launch_file in (MOVING_LAUNCH, FIXED_LAUNCH):
        source = launch_file.read_text(encoding='utf-8')
        ast.parse(source)
        assert 'phase10_localization_smoke.launch.py' in source
        assert "'auto_initial_pose': 'true'" in source
        assert "'use_rviz': 'false'" in source
        assert 'amcl_obstacle_test.rviz' in source
        assert 'TimerAction(period=5.0' in source
        for forbidden in (
            'resilient_nav_brne', 'planner_server', 'controller_server',
            'bt_navigator', "package='nav2_controller'",
        ):
            assert forbidden not in source

    moving = MOVING_LAUNCH.read_text(encoding='utf-8')
    fixed = FIXED_LAUNCH.read_text(encoding='utf-8')
    assert moving.count("('amcl_test_pedestrian_") == 4
    assert "executable='amcl_crowd_oscillator'" in moving
    assert '/world/resilient_lab/set_pose@' in moving
    assert 'ros_gz_interfaces/srv/SetEntityPose' in moving
    assert "'travel_distance': 2.0" in moving
    assert fixed.count("('amcl_test_fixed_obstacle_") == 4


def test_clean_baseline_adds_no_entities_or_obstacles():
    """The baseline must differ only by omitting test-scene entities."""
    source = CLEAN_LAUNCH.read_text(encoding='utf-8')
    ast.parse(source)
    for required in (
        'phase10_localization_smoke.launch.py',
        'phase9_slam_world.sdf',
        "'spawn_x': '-3.5'",
        "'spawn_y': '-3.5'",
        "'auto_initial_pose': 'true'",
        'amcl_obstacle_test.rviz',
    ):
        assert required in source
    for forbidden in (
        "package='ros_gz_sim'", "executable='create'", 'TimerAction',
        'amcl_test_pedestrian', 'amcl_test_fixed_obstacle',
    ):
        assert forbidden not in source


def test_models_are_new_lidar_visible_bounded_obstacles():
    """The test models must be physical cylinders with the intended motion."""
    crowd_root = ET.parse(CROWD_MODEL).getroot()
    crowd_model = crowd_root.find('model')
    assert crowd_model is not None
    assert crowd_model.findtext('static') == 'true'
    assert crowd_model.find('link/collision/geometry/cylinder') is not None

    fixed_root = ET.parse(FIXED_MODEL).getroot()
    fixed_model = fixed_root.find('model')
    assert fixed_model is not None
    assert fixed_model.findtext('static') == 'true'
    assert fixed_model.find('link/collision/geometry/cylinder') is not None


def test_rviz_exposes_particle_cloud_scan_and_odom_in_map():
    """The shared view must expose all evidence requested for observation."""
    source = RVIZ_FILE.read_text(encoding='utf-8')
    for required in (
        'Fixed Frame: map', 'Value: /scan', 'Value: /amcl_pose',
        'Value: /particle_cloud', 'Value: /odometry/filtered',
        'Name: Odom Frame In Map', 'Reference Frame: odom',
    ):
        assert required in source


def test_setup_installs_scene_models_and_oscillator():
    """Installed launches must be able to resolve models and the driver."""
    source = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')
    for required in (
        "'amcl_crowd_oscillator',",
        "'models', 'amcl_crowd_pedestrian'",
        "'models', 'amcl_fixed_obstacle'",
        'amcl_crowd_oscillator = amcl_crowd_oscillator:main',
    ):
        assert required in source
