#!/usr/bin/env python3
"""Run one repeatable route through the physical disturbance zone."""

import argparse
from math import pi
import time

from motion_safety import automatic_stop, MotionCommand, MotionTestPublisher
import rclpy
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions


PUBLISH_PERIOD_SEC = 0.05
SIM_CLOCK_STALL_TIMEOUT_SEC = 3.0


def route_segments(profile='standard'):
    """Return the selected collision-cleared physical experiment route."""
    if profile == 'stress_loops':
        lap_duration = 2.0 * pi / 0.50
        return (
            ('normal_startup', MotionCommand(0.25, 0.0), 8.0),
            ('fault_zone_forward_laps', MotionCommand(0.16, -0.50), 2.0 * lap_duration),
            ('fault_zone_stop', MotionCommand(0.0, 0.0), 0.8),
            ('fault_zone_reverse_laps', MotionCommand(-0.16, 0.50), 2.0 * lap_duration),
        )
    if profile == 'stress_shuttle':
        return (
            ('normal_startup', MotionCommand(0.25, 0.0), 8.0),
            ('fault_zone_forward_1', MotionCommand(0.22, 0.0), 6.0),
            ('fault_zone_reverse_1', MotionCommand(-0.22, 0.0), 6.0),
            ('fault_zone_forward_2', MotionCommand(0.22, 0.0), 6.0),
            ('fault_zone_reverse_2', MotionCommand(-0.22, 0.0), 6.0),
            ('fault_zone_forward_3', MotionCommand(0.22, 0.0), 6.0),
            ('fault_zone_reverse_3', MotionCommand(-0.22, 0.0), 6.0),
        )
    if profile != 'standard':
        raise ValueError(f'unsupported physical route profile: {profile}')
    return (
        ('approach', MotionCommand(0.25, 0.0), 5.2),
        ('zone_acceleration', MotionCommand(0.35, 0.0), 2.0),
        ('zone_left_arc', MotionCommand(0.22, 0.22), 1.0),
        ('zone_right_arc', MotionCommand(0.22, -0.22), 1.0),
        ('zone_stop', MotionCommand(0.0, 0.0), 0.8),
        ('zone_restart_exit', MotionCommand(0.30, 0.0), 3.8),
        ('left_yaw_excitation', MotionCommand(0.18, 0.35), 1.8),
        ('right_yaw_excitation', MotionCommand(0.18, -0.35), 1.8),
    )


def publish_for_sim_duration(node, command, duration_sec):
    """Publish until simulation time advances by the requested duration."""
    start_ns = None
    last_ns = None
    last_progress_wall = time.monotonic()
    duration_ns = int(duration_sec * 1_000_000_000)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)
        now_ns = node.get_clock().now().nanoseconds
        if start_ns is None:
            start_ns = now_ns
            last_ns = now_ns
        if now_ns - start_ns >= duration_ns:
            return
        if now_ns != last_ns:
            last_ns = now_ns
            last_progress_wall = time.monotonic()
        elif time.monotonic() - last_progress_wall > SIM_CLOCK_STALL_TIMEOUT_SEC:
            raise RuntimeError('simulation clock stopped during physical route')
        node.publish_command(command)
        time.sleep(PUBLISH_PERIOD_SEC)


def main(args=None):
    """Publish the fixed route and always finish with repeated zero commands."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--profile',
        choices=('standard', 'stress_loops', 'stress_shuttle'),
        default='standard',
    )
    options = parser.parse_args(args)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = MotionTestPublisher(node_name='physical_disturbance_route')
    node.set_parameters([Parameter('use_sim_time', value=True)])
    try:
        with automatic_stop(node):
            node.wait_for_subscriber()
            segments = route_segments(options.profile)
            for index, (name, command, duration) in enumerate(segments, 1):
                node.get_logger().info(
                    f'Physical route {index}/{len(segments)} {name}: '
                    f'linear.x={command.linear_x:.3f} m/s, '
                    f'angular.z={command.angular_z:.3f} rad/s, '
                    f'duration={duration:.3f} sim-s'
                )
                publish_for_sim_duration(node, command, duration)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl-C received; zero velocity sent.')
    except RuntimeError as error:
        node.get_logger().error(str(error))
        raise SystemExit(1) from error
    else:
        node.get_logger().info('Physical disturbance route complete; zero sent.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
