#!/usr/bin/env python3
"""Run a bounded, repeatable square route for the Phase 9 mapping world."""

import argparse
import math
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions

from motion_safety import (
    MotionCommand,
    MotionTestPublisher,
    automatic_stop,
    publish_for_duration,
)


def route_segments(
    linear_speed,
    angular_speed,
    straight_duration,
    turn_angle,
    turn_duration_scale,
    laps,
):
    """Return four straight/left-turn legs per lap, ending at the start pose."""
    turn_duration = abs(turn_angle / angular_speed) * turn_duration_scale
    turn_speed = math.copysign(abs(angular_speed), turn_angle)
    segments = []
    for _ in range(laps):
        for _ in range(4):
            segments.append((
                'straight',
                MotionCommand(linear_x=linear_speed, angular_z=0.0),
                straight_duration,
            ))
            segments.append((
                'turn',
                MotionCommand(linear_x=0.0, angular_z=turn_speed),
                turn_duration,
            ))
    return segments


def loop_closure_route_segments(
    linear_speed,
    angular_speed,
    turn_angle,
    turn_duration_scale,
):
    """Return the collision-cleared Phase 9 automatic-loop route.

    This route requires spawn ``(-4.7, -4.7, pi / 2)``.  Its 18.5 m
    south-west serpentine is the historical chain: three 5.5 m east/west
    legs, 1 m apart.  That deliberately puts a long, consecutive historical
    chain inside a future 3 m loop-search neighbourhood; a single 8.4 m
    straight corridor did not, because this runtime retained only about one
    node per 0.8 m.  The outer diversion returns to the west end of the
    historical middle before the final eastbound overlap.  All waypoints are
    in open space in the static Phase 9 SDF, with at least 0.65 m centreline
    clearance to walls or obstacles (0.35 m robot envelope plus 0.30 m
    margin).
    """
    turn_duration = abs(turn_angle / angular_speed) * turn_duration_scale
    left_turn_speed = math.copysign(abs(angular_speed), turn_angle)
    right_turn_speed = -left_turn_speed
    commands = (
        ('turn_right', 0.0, right_turn_speed, turn_duration),
        ('historical_lower_east', linear_speed, 0.0, 5.5 / linear_speed),
        ('turn_left', 0.0, left_turn_speed, turn_duration),
        ('historical_middle_north', linear_speed, 0.0, 1.0 / linear_speed),
        ('turn_left', 0.0, left_turn_speed, turn_duration),
        ('historical_middle_west', linear_speed, 0.0, 5.5 / linear_speed),
        ('turn_right', 0.0, right_turn_speed, turn_duration),
        ('historical_upper_north', linear_speed, 0.0, 1.0 / linear_speed),
        ('turn_right', 0.0, right_turn_speed, turn_duration),
        ('historical_upper_east', linear_speed, 0.0, 5.5 / linear_speed),
        ('turn_left', 0.0, left_turn_speed, turn_duration),
        ('outer_north', linear_speed, 0.0, 6.9 / linear_speed),
        ('turn_right', 0.0, right_turn_speed, turn_duration),
        ('upper_east', linear_speed, 0.0, 0.4 / linear_speed),
        ('turn_right', 0.0, right_turn_speed, turn_duration),
        ('east_descent', linear_speed, 0.0, 4.1 / linear_speed),
        ('turn_right', 0.0, right_turn_speed, turn_duration),
        ('middle_reentry_west', linear_speed, 0.0, 5.9 / linear_speed),
        ('turn_left', 0.0, left_turn_speed, turn_duration),
        ('historical_reentry_south', linear_speed, 0.0, 2.8 / linear_speed),
        ('turn_left', 0.0, left_turn_speed, turn_duration),
        ('historical_overlap_east', linear_speed, 0.0, 5.5 / linear_speed),
    )
    return [
        (name, MotionCommand(linear_x=linear, angular_z=angular), duration)
        for name, linear, angular, duration in commands
    ]


def publish_for_sim_duration(node, command, duration):
    """Publish against Gazebo time so loop-route metres match its waypoints."""
    start_time = node.get_clock().now()
    while rclpy.ok():
        elapsed = (node.get_clock().now() - start_time).nanoseconds / 1e9
        if elapsed >= duration:
            return
        node.publish_command(command)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(1.0 / 20.0)


def validate_options(options, parser):
    """Reject unsafe or degenerate route inputs before creating a ROS node."""
    numeric_values = {
        'linear speed': options.linear_speed,
        'angular speed': options.angular_speed,
        'straight duration': options.straight_duration,
        'turn angle': options.turn_angle,
        'turn duration scale': options.turn_duration_scale,
    }
    for label, value in numeric_values.items():
        if not math.isfinite(value):
            parser.error(f'{label} must be finite')
    if options.linear_speed <= 0.0:
        parser.error('linear speed must be greater than zero')
    if options.angular_speed <= 0.0:
        parser.error('angular speed must be greater than zero')
    if options.straight_duration <= 0.0:
        parser.error('straight duration must be greater than zero')
    if options.turn_angle == 0.0:
        parser.error('turn angle must be non-zero')
    if options.turn_duration_scale <= 0.0:
        parser.error('turn duration scale must be greater than zero')
    if options.laps < 1:
        parser.error('laps must be at least one')


def parse_options(argv=None):
    """Parse one or more repeatable square route laps."""
    parser = argparse.ArgumentParser(
        description=(
            'Run a bounded four-sided mapping route in phase9_slam_world, '
            'then repeatedly publish zero velocity.'
        )
    )
    parser.add_argument('--linear-speed', type=float, default=0.25)
    parser.add_argument('--angular-speed', type=float, default=0.6)
    parser.add_argument(
        '--straight-duration',
        type=float,
        default=16.0,
        help='Seconds per side; default yields a 4 m side at 0.25 m/s.',
    )
    parser.add_argument(
        '--turn-angle',
        type=float,
        default=math.pi / 2.0,
        help='Radians per corner; positive values turn left.',
    )
    parser.add_argument(
        '--turn-duration-scale',
        type=float,
        default=1.14,
        help=(
            'Gazebo differential-drive turn-duration compensation; the '
            'default is calibrated by the Phase 9 short-route smoke.'
        ),
    )
    parser.add_argument('--laps', type=int, default=1)
    parser.add_argument(
        '--route-shape', choices=('square', 'loop_closure'), default='square',
        help=(
            'square is the mapping baseline; loop_closure requires spawn '
            '(-4.7, -4.7, pi / 2).'
        ),
    )
    parser.add_argument(
        '--loop-stage', choices=('historical', 'diversion', 'full'),
        default='full',
        help=(
            'For loop_closure, stop after the historical chain, after the '
            'diversion reaches the historical midpoint, or after full overlap.'
        ),
    )
    options = parser.parse_args(argv)
    validate_options(options, parser)
    return options


def main(argv=None):
    """Run every route segment with the shared normal and Ctrl-C stop path."""
    options = parse_options(argv)
    if options.route_shape == 'loop_closure':
        segments = loop_closure_route_segments(
            options.linear_speed,
            options.angular_speed,
            options.turn_angle,
            options.turn_duration_scale,
        )
        if options.loop_stage == 'historical':
            segments = segments[:11]
        elif options.loop_stage == 'diversion':
            segments = segments[:20]
    else:
        segments = route_segments(
            options.linear_speed,
            options.angular_speed,
            options.straight_duration,
            options.turn_angle,
            options.turn_duration_scale,
            options.laps,
        )

    rclpy.init(
        args=[],
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = MotionTestPublisher(node_name='phase9_mapping_route')
    if options.route_shape == 'loop_closure':
        node.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True),
        ])
    try:
        with automatic_stop(node):
            node.wait_for_subscriber()
            for index, (name, command, duration) in enumerate(segments, 1):
                node.get_logger().info(
                    f'Segment {index}/{len(segments)} {name}: '
                    f'linear.x={command.linear_x:.3f} m/s, '
                    f'angular.z={command.angular_z:.3f} rad/s, '
                    f'duration={duration:.3f} s'
                )
                if options.route_shape == 'loop_closure':
                    publish_for_sim_duration(node, command, duration)
                else:
                    publish_for_duration(node, command, duration)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl-C received; zero velocity sent.')
    except RuntimeError as error:
        node.get_logger().error(str(error))
        raise SystemExit(1) from error
    else:
        node.get_logger().info('Route complete; zero velocity sent.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
