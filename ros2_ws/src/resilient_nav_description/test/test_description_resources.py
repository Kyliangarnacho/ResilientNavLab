"""Static tests for the stage 3 base and stage 4 mounting frames."""

import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

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


def sensor_for_reference(root, reference):
    """Return the only sensor attached through a Gazebo link override."""
    gazebo = root.find(f"./gazebo[@reference='{reference}']")
    assert gazebo is not None
    sensor = gazebo.find('sensor')
    assert sensor is not None
    return sensor


def test_robot_description_expands_with_expected_links():
    """The Xacro should contain the stage 3 base and six new frames."""
    root = expanded_robot()

    assert root.tag == 'robot'
    assert root.attrib['name'] == 'resilient_nav_robot'
    assert {link.attrib['name'] for link in root.findall('link')} == {
        'base_footprint',
        'base_link',
        'left_wheel_link',
        'right_wheel_link',
        'caster_link',
        'imu_link',
        'lidar_link',
        'camera_mount_link',
        'camera_link',
        'camera_optical_frame',
        'arm_mount_link',
    }


def test_stage4_fixed_frame_tree_and_origins():
    """Every mounting frame should have its intended parent and transform."""
    root = expanded_robot()
    expected_joints = {
        'imu_joint': ('base_link', 'imu_link', [-0.05, 0.0, 0.08]),
        'lidar_joint': ('base_link', 'lidar_link', [0.0, 0.0, 0.20]),
        'camera_mount_joint': (
            'base_link',
            'camera_mount_link',
            [0.18, 0.0, 0.11],
        ),
        'camera_joint': (
            'camera_mount_link',
            'camera_link',
            [0.07, 0.0, 0.04],
        ),
        'camera_optical_joint': (
            'camera_link',
            'camera_optical_frame',
            [0.0, 0.0, 0.0],
        ),
        'arm_mount_joint': (
            'base_link',
            'arm_mount_link',
            [-0.10, 0.0, 0.075],
        ),
    }

    for joint_name, (parent, child, xyz) in expected_joints.items():
        joint = root.find(f"./joint[@name='{joint_name}']")
        assert joint is not None
        assert joint.attrib['type'] == 'fixed'
        assert joint.find('parent').attrib['link'] == parent
        assert joint.find('child').attrib['link'] == child
        actual_xyz = [
            float(value)
            for value in joint.find('origin').attrib['xyz'].split()
        ]
        assert actual_xyz == pytest.approx(xyz)


def test_camera_optical_frame_uses_standard_nonzero_rotation():
    """The optical frame should use x-right, y-down, z-forward axes."""
    root = expanded_robot()
    joint = root.find("./joint[@name='camera_optical_joint']")

    assert joint is not None
    rpy = [
        float(value)
        for value in joint.find('origin').attrib['rpy'].split()
    ]
    assert rpy != pytest.approx([0.0, 0.0, 0.0])
    assert rpy == pytest.approx([-math.pi / 2.0, 0.0, -math.pi / 2.0])


def test_stage4_markers_are_visual_only():
    """Mount markers must not add collision or inertial dynamics."""
    root = expanded_robot()

    for link_name in (
        'imu_link',
        'lidar_link',
        'camera_mount_link',
        'camera_link',
    ):
        link = root.find(f"./link[@name='{link_name}']")
        assert link is not None
        assert link.find('visual') is not None
        assert link.find('collision') is None
        assert link.find('inertial') is None

    for link_name in ('camera_optical_frame', 'arm_mount_link'):
        link = root.find(f"./link[@name='{link_name}']")
        assert link is not None
        assert len(link) == 0


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
            '/collision/geometry/cylinder'
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


def test_stage3_chassis_geometry_and_inertia_are_unchanged():
    """Stage 4 frames must not alter the accepted stage 3 chassis baseline."""
    root = expanded_robot()
    base_link = root.find("./link[@name='base_link']")
    left_wheel = root.find("./link[@name='left_wheel_link']")
    right_wheel = root.find("./link[@name='right_wheel_link']")
    caster = root.find("./link[@name='caster_link']")

    assert base_link is not None
    assert left_wheel is not None
    assert right_wheel is not None
    assert caster is not None

    base_size = [
        float(value)
        for value in base_link.find(
            'collision/geometry/box'
        ).attrib['size'].split()
    ]
    base_mass = float(base_link.find('inertial/mass').attrib['value'])
    base_com = [
        float(value)
        for value in base_link.find('inertial/origin').attrib['xyz'].split()
    ]
    base_inertia = base_link.find('inertial/inertia').attrib

    assert base_size == pytest.approx([0.50, 0.35, 0.15])
    assert base_mass == pytest.approx(5.0)
    assert base_com == pytest.approx([-0.05, 0.0, -0.04])
    assert float(base_inertia['ixx']) == pytest.approx(0.06041666666666667)
    assert float(base_inertia['iyy']) == pytest.approx(0.11354166666666667)
    assert float(base_inertia['izz']) == pytest.approx(0.15520833333333334)
    assert all(
        float(base_inertia[key]) == pytest.approx(0.0)
        for key in ('ixy', 'ixz', 'iyz')
    )

    for wheel, y_position in (
        (left_wheel, 0.195),
        (right_wheel, -0.195),
    ):
        wheel_name = wheel.attrib['name'].removesuffix('_link')
        joint = root.find(f"./joint[@name='{wheel_name}_joint']")
        cylinder = wheel.find('collision/geometry/cylinder')
        origin = [
            float(value)
            for value in joint.find('origin').attrib['xyz'].split()
        ]

        assert joint.attrib['type'] == 'continuous'
        assert joint.find('parent').attrib['link'] == 'base_link'
        assert float(cylinder.attrib['radius']) == pytest.approx(0.10)
        assert float(cylinder.attrib['length']) == pytest.approx(0.04)
        assert origin == pytest.approx([0.10, y_position, -0.05])

    caster_joint = root.find("./joint[@name='caster_joint']")
    assert caster_joint.attrib['type'] == 'fixed'
    assert caster_joint.find('parent').attrib['link'] == 'base_link'
    assert caster_joint.find('child').attrib['link'] == 'caster_link'
    assert float(
        caster.find('collision/geometry/sphere').attrib['radius']
    ) == pytest.approx(0.05)


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

    assert set(expected_friction).issubset(gazebo_by_reference)
    for link_name, friction in expected_friction.items():
        gazebo = gazebo_by_reference[link_name]
        assert gazebo.findtext('mu1') == friction
        assert gazebo.findtext('mu2') == friction
        assert gazebo.findtext('material') == expected_materials[link_name]


def test_gazebo_diff_drive_uses_robot_joint_and_geometry_values():
    """The DiffDrive plugin should use the robot's accepted dimensions."""
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
    """The JointStatePublisher plugin should expose both wheel joints."""
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


def test_imu_sensor_configuration_and_noise():
    """The IMU should publish noisy body data from imu_link at 100 Hz."""
    root = expanded_robot()
    sensor = sensor_for_reference(root, 'imu_link')

    assert sensor.attrib == {'name': 'imu_sensor', 'type': 'imu'}
    assert sensor.findtext('always_on') == 'true'
    assert float(sensor.findtext('update_rate')) == pytest.approx(100.0)
    assert sensor.findtext('topic') == '/imu/data'
    assert sensor.findtext('gz_frame_id') == 'imu_link'
    assert sensor.findtext('imu/enable_orientation') == 'true'

    angular_noises = sensor.findall('imu/angular_velocity/*/noise')
    acceleration_noises = sensor.findall(
        'imu/linear_acceleration/*/noise'
    )
    assert len(angular_noises) == 3
    assert len(acceleration_noises) == 3
    for noise in angular_noises:
        assert noise.attrib['type'] == 'gaussian'
        assert float(noise.findtext('mean')) == pytest.approx(0.0)
        assert float(noise.findtext('stddev')) == pytest.approx(0.0002)
    for noise in acceleration_noises:
        assert noise.attrib['type'] == 'gaussian'
        assert float(noise.findtext('mean')) == pytest.approx(0.0)
        assert float(noise.findtext('stddev')) == pytest.approx(0.01)


def test_lidar_sensor_is_single_layer_with_range_noise():
    """The GPU lidar should produce only one noisy horizontal scan."""
    root = expanded_robot()
    sensor = sensor_for_reference(root, 'lidar_link')
    horizontal = sensor.find('lidar/scan/horizontal')
    vertical = sensor.find('lidar/scan/vertical')
    lidar_range = sensor.find('lidar/range')
    noise = sensor.find('lidar/noise')

    assert sensor.attrib == {
        'name': 'lidar_sensor',
        'type': 'gpu_lidar',
    }
    assert sensor.findtext('always_on') == 'true'
    assert float(sensor.findtext('update_rate')) == pytest.approx(15.0)
    assert sensor.findtext('topic') == '/scan'
    assert sensor.findtext('gz_frame_id') == 'lidar_link'
    assert sensor.findtext('visualize') == 'false'

    assert horizontal is not None
    assert int(horizontal.findtext('samples')) == 639
    assert float(horizontal.findtext('resolution')) == pytest.approx(1.0)
    assert float(horizontal.findtext('min_angle')) == pytest.approx(
        -3.0 * math.pi / 4.0 + math.pi / 426.0
    )
    assert float(horizontal.findtext('max_angle')) == pytest.approx(
        3.0 * math.pi / 4.0
    )
    assert (
        (float(horizontal.findtext('max_angle')) - float(horizontal.findtext('min_angle')))
        / (int(horizontal.findtext('samples')) - 1)
    ) == pytest.approx(math.pi / 426.0)
    assert vertical is not None
    assert int(vertical.findtext('samples')) == 1
    assert float(vertical.findtext('min_angle')) == pytest.approx(0.0)
    assert float(vertical.findtext('max_angle')) == pytest.approx(0.0)
    assert lidar_range is not None
    assert float(lidar_range.findtext('min')) == pytest.approx(0.08)
    assert float(lidar_range.findtext('max')) == pytest.approx(12.0)
    assert float(lidar_range.findtext('resolution')) == pytest.approx(0.01)
    assert noise is not None
    assert noise.findtext('type') == 'gaussian'
    assert float(noise.findtext('mean')) == pytest.approx(0.0)
    assert float(noise.findtext('stddev')) == pytest.approx(0.005)


def test_rgbd_sensor_uses_camera_link_and_optical_message_frame():
    """The forward RGB-D sensor should report the ROS optical frame."""
    root = expanded_robot()
    sensor = sensor_for_reference(root, 'camera_link')
    camera = sensor.find('camera')

    assert sensor.attrib == {
        'name': 'rgbd_camera_sensor',
        'type': 'rgbd_camera',
    }
    assert sensor.findtext('always_on') == 'true'
    assert sensor.findtext('topic') == '/camera'
    assert sensor.findtext('gz_frame_id') == 'camera_optical_frame'
    assert sensor.findtext('visualize') == 'false'
    assert camera is not None
    assert float(camera.findtext('horizontal_fov')) == pytest.approx(1.047)
    assert int(camera.findtext('image/width')) == 640
    assert int(camera.findtext('image/height')) == 480
    assert float(sensor.findtext('update_rate')) == pytest.approx(30.0)
    assert float(camera.findtext('clip/near')) == pytest.approx(0.1)
    assert float(camera.findtext('clip/far')) == pytest.approx(10.0)

    camera_joint = root.find("./joint[@name='camera_joint']")
    assert camera_joint.find('parent').attrib['link'] == 'camera_mount_link'
    assert camera_joint.find('child').attrib['link'] == 'camera_link'
    assert [
        float(value)
        for value in camera_joint.find('origin').attrib['rpy'].split()
    ] == pytest.approx([0.0, 0.0, 0.0])


def test_description_has_only_authorized_plugins_and_sensors():
    """Stage 4 should add only the authorized baseline sensors."""
    root = expanded_robot()
    plugins = {
        plugin.attrib['name']
        for plugin in root.findall('./gazebo/plugin')
    }

    assert plugins == {
        'gz::sim::systems::DiffDrive',
        'gz::sim::systems::JointStatePublisher',
    }
    assert {
        sensor.attrib['name']
        for sensor in root.findall('.//sensor')
    } == {'imu_sensor', 'lidar_sensor', 'rgbd_camera_sensor'}
    assert root.find(".//sensor[@type='camera']") is None
    assert root.find(".//sensor[@type='depth_camera']") is None
    assert len(root.findall(".//sensor[@type='rgbd_camera']")) == 1
    assert root.find('.//transmission') is None
    assert root.find('.//ros2_control') is None
