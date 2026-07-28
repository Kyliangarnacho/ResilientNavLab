#!/usr/bin/env python3
"""Publish a bounded stage 3 velocity command with a safe stop."""

import argparse
import math
import time
from contextlib import contextmanager
from dataclasses import dataclass

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


CMD_VEL_TOPIC = '/cmd_vel'
PUBLISH_RATE_HZ = 20.0
SUBSCRIBER_TIMEOUT_SECONDS = 5.0
STOP_REPETITIONS = 5
STOP_INTERVAL_SECONDS = 0.05


@dataclass(frozen=True)
class MotionCommand:
    """Planar velocity command used by the bounded publisher."""

    linear_x: float
    angular_z: float


def command_for_mode(mode, linear_speed, angular_speed):
    """Select the active velocity components for a motion mode."""
    if mode == 'straight':
        return MotionCommand(linear_x=linear_speed, angular_z=0.0)
    if mode == 'spin':
        return MotionCommand(linear_x=0.0, angular_z=angular_speed)
    if mode == 'arc':
        return MotionCommand(
            linear_x=linear_speed,
            angular_z=angular_speed,
        )
    raise ValueError(f'unsupported motion mode: {mode}')


def validate_options(options, parser):
    """Reject unsafe or meaningless command parameters."""
    numeric_values = {
        'linear speed': options.linear_speed,
        'angular speed': options.angular_speed,
        'duration': options.duration,
    }
    for label, value in numeric_values.items():
        if not math.isfinite(value):
            parser.error(f'{label} must be finite')

    if options.duration <= 0.0:
        parser.error('duration must be greater than zero')
    if options.mode in ('straight', 'arc'):
        if options.linear_speed == 0.0:
            parser.error(f'{options.mode} requires a non-zero linear speed')
    if options.mode in ('spin', 'arc'):
        if options.angular_speed == 0.0:
            parser.error(f'{options.mode} requires a non-zero angular speed')


def parse_options(argv=None):
    """Parse the motion mode, velocities, and bounded duration."""
    parser = argparse.ArgumentParser(
        description=(
            'Publish straight, in-place spin, or arc motion on /cmd_vel, '
            'then automatically publish zero velocity.'
        )
    )
    parser.add_argument(
        'mode',
        choices=('straight', 'spin', 'arc'),
        help='Motion pattern to publish.',
    )
    parser.add_argument(
        '--linear-speed',
        type=float,
        default=0.2,
        metavar='MPS',
        help='Linear X speed in m/s (default: 0.2).',
    )
    parser.add_argument(
        '--angular-speed',
        type=float,
        default=0.6,
        metavar='RADPS',
        help='Angular Z speed in rad/s (default: 0.6).',
    )
    parser.add_argument(
        '--duration',
        type=float,
        default=2.0,
        metavar='SECONDS',
        help='Wall-clock command duration in seconds (default: 2.0).',
    )
    options = parser.parse_args(argv)
    validate_options(options, parser)
    return options


class MotionTestPublisher(Node):
    """Publish the selected Twist and an explicit repeated stop command."""

    def __init__(self):
        """Create the fixed /cmd_vel publisher."""
        super().__init__('motion_test')
        self._publisher = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

    def wait_for_subscriber(self, timeout_sec=SUBSCRIBER_TIMEOUT_SECONDS):
        """Wait briefly for the command bridge before starting motion."""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self._publisher.get_subscription_count() > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(
            f'no subscriber discovered on {CMD_VEL_TOPIC} '
            f'within {timeout_sec:.1f} seconds'
        )

    def publish_command(self, command):
        """Publish one planar velocity sample."""
        message = Twist()
        message.linear.x = command.linear_x
        message.angular.z = command.angular_z
        self._publisher.publish(message)

    def publish_stop(
        self,
        repetitions=STOP_REPETITIONS,
        interval_sec=STOP_INTERVAL_SECONDS,
    ):
        """Repeat zero velocity so the bridge receives a final stop."""
        stop = MotionCommand(linear_x=0.0, angular_z=0.0)
        for index in range(repetitions):
            self.publish_command(stop)
            if index + 1 < repetitions:
                time.sleep(interval_sec)


@contextmanager
def automatic_stop(publisher):
    """Request a stop whenever the protected operation exits."""
    try:
        yield
    finally:
        publisher.publish_stop()


def publish_for_duration(
    publisher,
    command,
    duration,
    publish_rate_hz=PUBLISH_RATE_HZ,
):
    """Publish a command at a fixed wall-clock rate for a bounded time."""
    period = 1.0 / publish_rate_hz
    deadline = time.monotonic() + duration
    next_publish = time.monotonic()

    while time.monotonic() < deadline:
        publisher.publish_command(command)
        next_publish += period
        remaining = deadline - time.monotonic()
        sleep_time = min(next_publish - time.monotonic(), remaining)
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        elif next_publish < time.monotonic():
            next_publish = time.monotonic()


def main(argv=None):
    """Run the bounded publisher and retain control of Ctrl-C cleanup."""
    options = parse_options(argv)
    command = command_for_mode(
        options.mode,
        options.linear_speed,
        options.angular_speed,
    )

    rclpy.init(
        args=[],
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = MotionTestPublisher()

    try:
        with automatic_stop(node):
            node.wait_for_subscriber()
            node.get_logger().info(
                f'Publishing {options.mode}: '
                f'linear.x={command.linear_x:.3f} m/s, '
                f'angular.z={command.angular_z:.3f} rad/s, '
                f'duration={options.duration:.3f} s'
            )
            publish_for_duration(node, command, options.duration)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl-C received; zero velocity sent.')
    except RuntimeError as error:
        node.get_logger().error(str(error))
        raise SystemExit(1) from error
    else:
        node.get_logger().info('Motion complete; zero velocity sent.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
