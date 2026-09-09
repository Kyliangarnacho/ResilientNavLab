"""Launch one parameterized physical-disturbance scene on the Phase 9 world."""

from dataclasses import dataclass
from math import ceil, hypot, isfinite
import os
from pathlib import Path
from tempfile import gettempdir
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


SUPPORTED_SCENARIOS = {
    'normal',
    'low_friction',
    'asymmetric_traction',
    'rough_surface',
    'external_impact',
    'wheel_block',
    'navigation_gauntlet',
}
WHEEL_TRACK_OFFSET_M = 0.195
GAUNTLET_ROUGHNESS_LENGTH_FRACTION = 0.25
GAUNTLET_ROUGHNESS_WIDTH_FRACTION = 0.65
GAUNTLET_MAX_REGION_GAP_M = 0.18


@dataclass(frozen=True)
class PhysicalSceneConfig:
    """Validated physical-scene values used to generate one SDF world."""

    scenario: str
    zone_center_x: float = -1.5
    zone_center_y: float = -3.5
    zone_length_m: float = 2.0
    zone_width_m: float = 1.0
    low_friction_mu: float = 0.12
    asymmetric_low_mu: float = 0.15
    weak_wheel_side: str = 'left'
    roughness_height_m: float = 0.008
    roughness_spacing_m: float = 0.45
    disturbance_delay_sec: float = 7.0
    disturbance_pulse_count: int = 1
    disturbance_interval_sec: float = 3.0
    impact_duration_sec: float = 0.25
    impact_force_x_n: float = 0.0
    impact_force_y_n: float = 12.0
    wheel_block_duration_sec: float = 0.60
    wheel_block_force_n: float = 3.0
    wheel_block_yaw_torque_nm: float = 0.08
    blocked_wheel: str = 'left'

    def __post_init__(self):
        """Reject invisible disturbances and overly extreme default sweeps."""
        if self.scenario not in SUPPORTED_SCENARIOS:
            raise ValueError(f'unsupported physical scenario: {self.scenario}')
        finite_values = (
            self.zone_center_x,
            self.zone_center_y,
            self.zone_length_m,
            self.zone_width_m,
            self.low_friction_mu,
            self.asymmetric_low_mu,
            self.roughness_height_m,
            self.roughness_spacing_m,
            self.disturbance_delay_sec,
            self.disturbance_interval_sec,
            self.impact_duration_sec,
            self.impact_force_x_n,
            self.impact_force_y_n,
            self.wheel_block_duration_sec,
            self.wheel_block_force_n,
            self.wheel_block_yaw_torque_nm,
        )
        if not all(isfinite(value) for value in finite_values):
            raise ValueError('physical scene parameters must be finite')
        if not 0.05 <= self.low_friction_mu <= 0.80:
            raise ValueError('low_friction_mu must be within [0.05, 0.80]')
        if not 0.05 <= self.asymmetric_low_mu <= 0.80:
            raise ValueError('asymmetric_low_mu must be within [0.05, 0.80]')
        if self.zone_length_m <= 0.5 or self.zone_width_m <= 0.4:
            raise ValueError('disturbance zone is too small for repeatable traversal')
        if self.weak_wheel_side not in {'left', 'right'}:
            raise ValueError('weak_wheel_side must be left or right')
        if not 0.003 <= self.roughness_height_m <= 0.020:
            raise ValueError('roughness_height_m must be within [0.003, 0.020]')
        if not 0.15 <= self.roughness_spacing_m <= 0.80:
            raise ValueError('roughness_spacing_m must be within [0.15, 0.80]')
        if not 0.5 <= self.disturbance_delay_sec <= 30.0:
            raise ValueError('disturbance_delay_sec must be within [0.5, 30]')
        impact_force = hypot(self.impact_force_x_n, self.impact_force_y_n)
        if not 2.0 <= impact_force <= 30.0:
            raise ValueError('impact force magnitude must be within [2, 30] N')
        if not 1 <= self.disturbance_pulse_count <= 5:
            raise ValueError('disturbance_pulse_count must be within [1, 5]')
        if not 0.25 <= self.disturbance_interval_sec <= 10.0:
            raise ValueError('disturbance_interval_sec must be within [0.25, 10.0]')
        if not 0.05 <= self.impact_duration_sec <= 1.0:
            raise ValueError('impact_duration_sec must be within [0.05, 1.0]')
        if not 0.2 <= self.wheel_block_duration_sec <= 2.0:
            raise ValueError('wheel_block_duration_sec must be within [0.2, 2.0]')
        if not 0.5 <= self.wheel_block_force_n <= 8.0:
            raise ValueError('wheel_block_force_n must be within [0.5, 8]')
        if not 0.01 <= self.wheel_block_yaw_torque_nm <= 0.30:
            raise ValueError(
                'wheel_block_yaw_torque_nm must be within [0.01, 0.30]'
            )
        if self.blocked_wheel not in {'left', 'right', 'both'}:
            raise ValueError('blocked_wheel must be left, right, or both')


def build_physical_world(base_world: Path, output_world: Path, config):
    """Add common landmarks and one bounded disturbance configuration."""
    tree = ET.parse(base_world)
    root = tree.getroot()
    world = root.find('world')
    if world is None or world.attrib.get('name') != 'resilient_lab':
        raise ValueError('base world must contain the resilient_lab world')

    _append_lidar_landmarks(world)

    if config.scenario == 'normal':
        pass
    elif config.scenario == 'low_friction':
        _append_surface_patch(
            world,
            name='physical_low_friction_zone',
            center_x=config.zone_center_x,
            center_y=config.zone_center_y,
            length=config.zone_length_m,
            width=config.zone_width_m,
            friction=config.low_friction_mu,
            color='0.12 0.32 0.85 0.72',
        )
    elif config.scenario == 'asymmetric_traction':
        offset = WHEEL_TRACK_OFFSET_M * (
            1.0 if config.weak_wheel_side == 'left' else -1.0
        )
        _append_surface_patch(
            world,
            name=f'physical_{config.weak_wheel_side}_traction_strip',
            center_x=config.zone_center_x,
            center_y=config.zone_center_y + offset,
            length=config.zone_length_m,
            width=0.20,
            friction=config.asymmetric_low_mu,
            color='0.90 0.52 0.08 0.78',
        )
    elif config.scenario == 'rough_surface':
        _append_roughness_bumps(world, config)
    elif config.scenario == 'navigation_gauntlet':
        zone_min_x = config.zone_center_x - 0.5 * config.zone_length_m
        roughness_length = (
            config.zone_length_m * GAUNTLET_ROUGHNESS_LENGTH_FRACTION
        )
        region_gap = min(
            GAUNTLET_MAX_REGION_GAP_M, 0.15 * config.zone_length_m
        )
        friction_length = config.zone_length_m - roughness_length - region_gap
        roughness_center_x = zone_min_x + 0.5 * roughness_length
        friction_center_x = (
            zone_min_x + roughness_length + region_gap
            + 0.5 * friction_length
        )
        _append_surface_patch(
            world,
            name='physical_low_friction_zone',
            center_x=friction_center_x,
            center_y=config.zone_center_y,
            length=friction_length,
            width=config.zone_width_m,
            friction=config.low_friction_mu,
            color='0.12 0.32 0.85 0.52',
        )
        _append_roughness_bumps(
            world,
            config,
            center_x=roughness_center_x,
            length=roughness_length,
            width=config.zone_width_m * GAUNTLET_ROUGHNESS_WIDTH_FRACTION,
            minimum_count=3,
        )
        world.insert(
            5,
            ET.fromstring(
                '<plugin filename="gz-sim-apply-link-wrench-system" '
                'name="gz::sim::systems::ApplyLinkWrench"/>'
            ),
        )
    else:
        world.insert(
            5,
            ET.fromstring(
                '<plugin filename="gz-sim-apply-link-wrench-system" '
                'name="gz::sim::systems::ApplyLinkWrench"/>'
            ),
        )

    output_world.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space='  ')
    tree.write(output_world, encoding='utf-8', xml_declaration=True)


def _append_surface_patch(
    world,
    *,
    name,
    center_x,
    center_y,
    length,
    width,
    friction,
    color,
):
    patch = ET.fromstring(f"""
      <model name="{name}">
        <static>true</static>
        <pose>{center_x} {center_y} 0.001 0 0 0</pose>
        <link name="surface_link">
          <collision name="surface_collision">
            <geometry><box><size>{length} {width} 0.002</size></box></geometry>
            <surface>
              <friction><ode><mu>{friction}</mu><mu2>{friction}</mu2></ode></friction>
            </surface>
          </collision>
          <visual name="surface_visual">
            <geometry><box><size>{length} {width} 0.002</size></box></geometry>
            <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
            <cast_shadows>false</cast_shadows>
          </visual>
        </link>
      </model>
    """)
    world.append(patch)


def _append_lidar_landmarks(world):
    """Add common off-route geometry so scan motion is observable in 2D."""
    landmarks = (
        ('near_cylinder_west', -2.35, -2.62, 'cylinder'),
        ('near_box_south', -1.35, -4.45, 'box'),
        ('near_cylinder_east', -0.25, -2.72, 'cylinder'),
    )
    for name, x_value, y_value, shape in landmarks:
        geometry = (
            '<cylinder><radius>0.20</radius><length>0.75</length></cylinder>'
            if shape == 'cylinder'
            else '<box><size>0.42 0.32 0.75</size></box>'
        )
        model = ET.fromstring(f"""
          <model name="physical_lidar_{name}">
            <static>true</static>
            <pose>{x_value} {y_value} 0.375 0 0 0</pose>
            <link name="landmark_link">
              <collision name="landmark_collision">
                <geometry>{geometry}</geometry>
              </collision>
              <visual name="landmark_visual">
                <geometry>{geometry}</geometry>
                <material>
                  <ambient>0.18 0.70 0.72 1</ambient>
                  <diffuse>0.22 0.82 0.84 1</diffuse>
                </material>
              </visual>
            </link>
          </model>
        """)
        world.append(model)


def _append_roughness_bumps(
    world,
    config,
    *,
    center_x=None,
    length=None,
    width=None,
    minimum_count=5,
):
    center_x = config.zone_center_x if center_x is None else center_x
    length = config.zone_length_m if length is None else length
    width = config.zone_width_m if width is None else width
    bump_count = max(
        minimum_count, ceil(length / config.roughness_spacing_m)
    )
    covered_length = (bump_count - 1) * config.roughness_spacing_m
    first_x = center_x - 0.5 * covered_length
    for index in range(bump_count):
        center_x = first_x + index * config.roughness_spacing_m
        height = config.roughness_height_m
        radius = 0.05
        center_z = height - radius
        bar = ET.fromstring(f"""
          <model name="physical_roughness_bump_{index + 1}">
            <static>true</static>
            <pose>{center_x} {config.zone_center_y} {center_z} 1.57079632679 0 0</pose>
            <link name="bump_link">
              <collision name="bump_collision">
                <geometry>
                  <cylinder><radius>{radius}</radius><length>{width}</length></cylinder>
                </geometry>
                <surface>
                  <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
                </surface>
              </collision>
              <visual name="bump_visual">
                <geometry>
                  <cylinder><radius>{radius}</radius><length>{width}</length></cylinder>
                </geometry>
                <material>
                  <ambient>0.48 0.25 0.08 1</ambient>
                  <diffuse>0.62 0.34 0.12 1</diffuse>
                </material>
              </visual>
            </link>
          </model>
        """)
        world.append(bar)


def _configure_scene(context):
    simulation_share = Path(
        get_package_share_directory('resilient_nav_simulation')
    )
    config = PhysicalSceneConfig(
        scenario=_value(context, 'scenario'),
        zone_center_x=_float_value(context, 'zone_center_x'),
        zone_center_y=_float_value(context, 'zone_center_y'),
        zone_length_m=_float_value(context, 'zone_length_m'),
        zone_width_m=_float_value(context, 'zone_width_m'),
        low_friction_mu=_float_value(context, 'low_friction_mu'),
        asymmetric_low_mu=_float_value(context, 'asymmetric_low_mu'),
        weak_wheel_side=_value(context, 'weak_wheel_side'),
        roughness_height_m=_float_value(context, 'roughness_height_m'),
        roughness_spacing_m=_float_value(context, 'roughness_spacing_m'),
        disturbance_delay_sec=_float_value(
            context, 'disturbance_delay_sec'
        ),
        disturbance_pulse_count=_int_value(
            context, 'disturbance_pulse_count'
        ),
        disturbance_interval_sec=_float_value(
            context, 'disturbance_interval_sec'
        ),
        impact_duration_sec=_float_value(context, 'impact_duration_sec'),
        impact_force_x_n=_float_value(context, 'impact_force_x_n'),
        impact_force_y_n=_float_value(context, 'impact_force_y_n'),
        wheel_block_duration_sec=_float_value(
            context, 'wheel_block_duration_sec'
        ),
        wheel_block_force_n=_float_value(
            context, 'wheel_block_force_n'
        ),
        wheel_block_yaw_torque_nm=_float_value(
            context, 'wheel_block_yaw_torque_nm'
        ),
        blocked_wheel=_value(context, 'blocked_wheel'),
    )
    base_world = Path(_value(context, 'base_world'))
    generated_world = (
        Path(gettempdir())
        / 'resilient_nav_physical_scenes'
        / f'{config.scenario}_{os.getpid()}.sdf'
    )
    build_physical_world(base_world, generated_world, config)

    robot_scene = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(simulation_share / 'launch' / 'phase4_imu_lidar_demo.launch.py')
        ),
        launch_arguments={
            'world': str(generated_world),
            'use_rviz': _value(context, 'use_rviz'),
            'entity_name': _value(context, 'entity_name'),
            'spawn_x': _value(context, 'spawn_x'),
            'spawn_y': _value(context, 'spawn_y'),
            'spawn_z': _value(context, 'spawn_z'),
            'spawn_yaw': _value(context, 'spawn_yaw'),
            'odom_ros_topic': _value(context, 'odom_ros_topic'),
            'start_odom_tf_broadcaster': _value(
                context, 'start_odom_tf_broadcaster'
            ),
        }.items(),
    )
    actions = [robot_scene]
    if config.scenario in {
        'external_impact', 'wheel_block', 'navigation_gauntlet'
    }:
        actions.extend(_dynamic_disturbance_actions(config, context))
    return actions


def _dynamic_disturbance_actions(config, context):
    wrench_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='physical_disturbance_wrench_bridge',
        output='screen',
        arguments=[
            '/world/resilient_lab/wrench/persistent'
            '@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench',
            '/world/resilient_lab/wrench/clear'
            '@ros_gz_interfaces/msg/Entity]gz.msgs.Entity',
        ],
    )
    duration_sec = (
        config.impact_duration_sec
        if config.scenario in {'external_impact', 'navigation_gauntlet'}
        else config.wheel_block_duration_sec
    )
    pulse = Node(
        package='resilient_nav_simulation',
        executable='physical_disturbance_pulse',
        name='physical_disturbance_pulse',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'mode': (
                'external_impact'
                if config.scenario == 'navigation_gauntlet'
                else config.scenario
            ),
            'delay_after_motion_sec': config.disturbance_delay_sec,
            'duration_sec': duration_sec,
            'pulse_count': config.disturbance_pulse_count,
            'pulse_interval_sec': config.disturbance_interval_sec,
            'robot_model_name': _value(context, 'entity_name'),
            'impact_force_x_n': config.impact_force_x_n,
            'impact_force_y_n': config.impact_force_y_n,
            'wheel_block_force_n': config.wheel_block_force_n,
            'wheel_block_yaw_torque_nm': (
                config.wheel_block_yaw_torque_nm
            ),
            'blocked_wheel': config.blocked_wheel,
        }],
    )
    return [wrench_bridge, pulse]


def _value(context, name):
    return LaunchConfiguration(name).perform(context)


def _float_value(context, name):
    return float(_value(context, name))


def _int_value(context, name):
    return int(_value(context, name))


def generate_launch_description():
    """Expose one control and five bounded physical experiment scenarios."""
    simulation_share = Path(
        get_package_share_directory('resilient_nav_simulation')
    )
    arguments = [
        ('scenario', 'low_friction', 'Physical disturbance scenario name.'),
        (
            'base_world',
            str(simulation_share / 'worlds' / 'phase9_slam_world.sdf'),
            'Base SDF; defaults to the Phase 9 mapping world.',
        ),
        ('use_rviz', 'true', 'Start the existing sensor RViz view.'),
        ('entity_name', 'resilient_nav_robot', 'Spawned robot model name.'),
        ('spawn_x', '-4.0', 'Experiment start X position.'),
        ('spawn_y', '-3.5', 'Experiment start Y position.'),
        ('spawn_z', '0.25', 'Experiment start Z position.'),
        ('spawn_yaw', '0.0', 'Experiment start yaw.'),
        ('odom_ros_topic', '/odom', 'Raw Gazebo wheel-odometry topic.'),
        (
            'start_odom_tf_broadcaster',
            'true',
            'Preserve the legacy raw odometry TF owner by default.',
        ),
        ('zone_center_x', '-1.5', 'Static disturbance zone center X.'),
        ('zone_center_y', '-3.5', 'Static disturbance zone center Y.'),
        ('zone_length_m', '2.0', 'Static disturbance zone length.'),
        ('zone_width_m', '1.0', 'Static disturbance zone width.'),
        ('low_friction_mu', '0.12', 'Low-friction zone coefficient.'),
        ('asymmetric_low_mu', '0.15', 'Weak wheel-side friction coefficient.'),
        ('weak_wheel_side', 'left', 'Low-traction strip side: left/right.'),
        ('roughness_height_m', '0.008', 'Exposed height of five rounded bumps.'),
        ('roughness_spacing_m', '0.45', 'Spacing between rounded bumps.'),
        (
            'disturbance_delay_sec',
            '7.0',
            'Delay from the first nonzero command to dynamic disturbance.',
        ),
        ('disturbance_pulse_count', '1', 'Number of separated disturbance pulses.'),
        ('disturbance_interval_sec', '3.0', 'Clear interval between pulses.'),
        ('impact_duration_sec', '0.25', 'External force pulse duration.'),
        ('impact_force_x_n', '0.0', 'World-frame impact force X.'),
        ('impact_force_y_n', '12.0', 'World-frame impact force Y.'),
        ('wheel_block_duration_sec', '0.60', 'Wheel resistance duration.'),
        ('wheel_block_force_n', '3.0', 'Opposing longitudinal force.'),
        (
            'wheel_block_yaw_torque_nm',
            '0.08',
            'Small base yaw torque associated with one blocked wheel.',
        ),
        ('blocked_wheel', 'left', 'Blocked wheel: left/right/both.'),
    ]
    return LaunchDescription([
        *[
            DeclareLaunchArgument(name, default_value=default, description=description)
            for name, default, description in arguments
        ],
        OpaqueFunction(function=_configure_scene),
    ])
