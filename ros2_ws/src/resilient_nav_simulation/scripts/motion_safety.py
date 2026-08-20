"""Shared bounded `/cmd_vel` publishing primitives for simulation experiments."""

import time
from contextlib import contextmanager
from dataclasses import dataclass

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node


CMD_VEL_TOPIC = '/cmd_vel'
PUBLISH_RATE_HZ = 20.0
SUBSCRIBER_TIMEOUT_SECONDS = 5.0
STOP_REPETITIONS = 5
STOP_INTERVAL_SECONDS = 0.05


@dataclass(frozen=True)
class MotionCommand:
    """Planar velocity command used by bounded simulation publishers."""

    linear_x: float
    angular_z: float


class MotionTestPublisher(Node):
    """Publish bounded Twist commands and an explicit repeated final stop."""

    def __init__(self, node_name='motion_test'):
        """Create the fixed `/cmd_vel` publisher."""
        super().__init__(node_name)
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
