"""Static tests for the stage 3 robot description."""

import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
XACRO_PATH = PACKAGE_ROOT / 'urdf' / 'resilient_nav_robot.urdf.xacro'


def expanded_robot(*xacro_arguments):
    """Expand the project Xacro and return its XML root."""
    result = subprocess.run(
        ['xacro', str(XACRO_PATH), *xacro_arguments],
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(result.stdout)


def test_robot_description_expands_with_expected_links():
    """The Xacro should expand to the intended five-link robot."""
    root = expanded_robot()

    assert root.tag == 'robot'
    assert root.attrib['name'] == 'resilient_nav_robot'
    assert {link.attrib['name'] for link in root.findall('link')} == {
        'base_footprint',
        'base_link',
        'left_wheel_link',
        'right_wheel_link',
        'caster_link',
    }


def test_physical_links_have_collision_and_inertia():
    """Every physical link should provide collision and inertial data."""
    root = expanded_robot()

    for link_name in (
        'base_link',
        'left_wheel_link',
        'right_wheel_link',
        'caster_link',
    ):
        link = root.find(f"./link[@name='{link_name}']")
        assert link is not None
        assert link.find('collision') is not None
        assert link.find('inertial') is not None


def test_longitudinal_support_contains_low_center_of_mass():
    """Wheel, caster, clearance, and inertia should resist body tipping."""
    root = expanded_robot()

    base_link = root.find("./link[@name='base_link']")
    footprint_joint = root.find(
        "./joint[@name='base_footprint_joint']"
    )
    left_wheel_joint = root.find("./joint[@name='left_wheel_joint']")
    right_wheel_joint = root.find("./joint[@name='right_wheel_joint']")
    caster_joint = root.find("./joint[@name='caster_joint']")

    assert base_link is not None
    assert footprint_joint is not None
    assert left_wheel_joint is not None
    assert right_wheel_joint is not None
    assert caster_joint is not None

    footprint_origin = [
        float(value)
        for value in footprint_joint.find('origin').attrib['xyz'].split()
    ]
    base_z = footprint_origin[2]
    base_size = [
        float(value)
        for value in base_link.find('collision/geometry/box').attrib[
            'size'
        ].split()
    ]
    base_com = [
        float(value)
        for value in base_link.find('inertial/origin').attrib['xyz'].split()
    ]
    wheel_origins = [
        [
            float(value)
            for value in joint.find('origin').attrib['xyz'].split()
        ]
        for joint in (left_wheel_joint, right_wheel_joint)
    ]
    caster_origin = [
        float(value)
        for value in caster_joint.find('origin').attrib['xyz'].split()
    ]

    wheel_radius = float(
        root.find(
            "./link[@name='left_wheel_link']"
            "/collision/geometry/cylinder"
        ).attrib['radius']
    )
    caster_radius = float(
        root.find(
            "./link[@name='caster_link']/collision/geometry/sphere"
        ).attrib['radius']
    )

    assert wheel_origins[0][0] == pytest.approx(0.10)
    assert wheel_origins[1][0] == pytest.approx(0.10)
    assert footprint_origin[0] == pytest.approx(-0.10)
    assert footprint_origin[0] + wheel_origins[0][0] == pytest.approx(0.0)
    assert caster_origin[0] == pytest.approx(-0.20)
    assert base_z + wheel_origins[0][2] == pytest.approx(wheel_radius)
    assert base_z + caster_origin[2] == pytest.approx(caster_radius)
    assert base_z - base_size[2] / 2.0 == pytest.approx(0.075)
    assert base_com == pytest.approx([-0.05, 0.0, -0.04])

    base_mass = float(base_link.find('inertial/mass').attrib['value'])
    wheel_mass = float(
        root.find("./link[@name='left_wheel_link']/inertial/mass").attrib[
            'value'
        ]
    )
    caster_mass = float(
        root.find("./link[@name='caster_link']/inertial/mass").attrib[
            'value'
        ]
    )
    total_mass = base_mass + 2.0 * wheel_mass + caster_mass
    center_of_mass_x = (
        base_mass * base_com[0]
        + 2.0 * wheel_mass * wheel_origins[0][0]
        + caster_mass * caster_origin[0]
    ) / total_mass

    assert center_of_mass_x - caster_origin[0] > 0.10
    assert wheel_origins[0][0] - center_of_mass_x > 0.10


def test_gazebo_materials_and_friction_cover_physical_links():
    """Gazebo overrides should define rendering and contact friction."""
    root = expanded_robot()
    expected_friction = {
        'base_link': '0.5',
        'left_wheel_link': '1.0',
        'right_wheel_link': '1.0',
        'caster_link': '0.05',
    }
    expected_materials = {
        'base_link': 'Gazebo/Blue',
        'left_wheel_link': 'Gazebo/Black',
        'right_wheel_link': 'Gazebo/Black',
        'caster_link': 'Gazebo/Grey',
    }

    gazebo_by_reference = {
        element.attrib['reference']: element
        for element in root.findall('gazebo')
        if 'reference' in element.attrib
    }

    assert set(gazebo_by_reference) == set(expected_friction)
    for link_name, friction in expected_friction.items():
        gazebo = gazebo_by_reference[link_name]
        assert gazebo.findtext('mu1') == friction
        assert gazebo.findtext('mu2') == friction
        assert gazebo.findtext('material') == expected_materials[link_name]


def test_gazebo_diff_drive_uses_robot_joint_and_geometry_values():
    """DiffDrive should use the wheel joints and dimensions in this Xacro."""
    root = expanded_robot()
    plugin = root.find(
        "./gazebo/plugin[@name='gz::sim::systems::DiffDrive']"
    )

    assert plugin is not None
    assert plugin.attrib['filename'] == 'gz-sim-diff-drive-system'
    assert plugin.findtext('left_joint') == 'left_wheel_joint'
    assert plugin.findtext('right_joint') == 'right_wheel_joint'
    assert float(plugin.findtext('wheel_separation')) == pytest.approx(0.39)
    assert float(plugin.findtext('wheel_radius')) == pytest.approx(0.10)
    assert plugin.findtext('topic') == (
        '/model/resilient_nav_robot/cmd_vel'
    )
    assert plugin.findtext('odom_topic') == (
        '/model/resilient_nav_robot/odometry'
    )
    assert plugin.findtext('frame_id') == 'odom'
    assert plugin.findtext('child_frame_id') == 'base_footprint'


def test_gazebo_joint_states_use_native_topic_and_wheel_joints():
    """JointStatePublisher should expose both wheels on a native topic."""
    root = expanded_robot()
    plugin = root.find(
        "./gazebo/plugin[@name='gz::sim::systems::JointStatePublisher']"
    )

    assert plugin is not None
    assert (
        plugin.attrib['filename']
        == 'gz-sim-joint-state-publisher-system'
    )
    assert plugin.findtext('topic') == (
        '/world/resilient_lab/model/resilient_nav_robot/joint_state'
    )
    assert [element.text for element in plugin.findall('joint_name')] == [
        'left_wheel_joint',
        'right_wheel_joint',
    ]


def test_gazebo_topics_follow_configured_entity_name():
    """A renamed Gazebo entity should retain matching native topics."""
    root = expanded_robot('gazebo_model_name:=test_robot')
    plugins = {
        plugin.attrib['name']: plugin
        for plugin in root.findall('./gazebo/plugin')
    }

    assert plugins['gz::sim::systems::DiffDrive'].findtext('topic') == (
        '/model/test_robot/cmd_vel'
    )
    assert plugins['gz::sim::systems::DiffDrive'].findtext(
        'odom_topic'
    ) == '/model/test_robot/odometry'
    assert plugins[
        'gz::sim::systems::JointStatePublisher'
    ].findtext('topic') == (
        '/world/resilient_lab/model/test_robot/joint_state'
    )


def test_description_has_only_authorized_plugins_and_no_future_systems():
    """This milestone should contain only native drive and joint states."""
    root = expanded_robot()
    plugins = {
        plugin.attrib['name']
        for plugin in root.findall('./gazebo/plugin')
    }

    assert plugins == {
        'gz::sim::systems::DiffDrive',
        'gz::sim::systems::JointStatePublisher',
    }
    assert root.find('.//sensor') is None
    assert root.find('.//transmission') is None
    assert root.find('.//ros2_control') is None
