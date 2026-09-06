"""Explicit, single-input authority gate from BRNE raw commands to /cmd_vel."""

from __future__ import annotations

import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

from .control_gate import endpoint_is_self, validated_diff_drive_command


class BrneControlGate(Node):
    """Publish formal velocity only when manually armed and sole topic owner."""

    def __init__(self):
        super().__init__('brne_control_gate')
        self.declare_parameter('armed', False)
        self.declare_parameter('input_topic', '/brne/cmd_vel_raw')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('input_timeout_sec', 0.35)
        self.declare_parameter('check_frequency_hz', 20.0)
        self.declare_parameter('maximum_linear_velocity', 0.30)
        self.declare_parameter('maximum_angular_velocity', 0.80)

        self.armed = bool(self.get_parameter('armed').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.input_timeout_sec = float(
            self.get_parameter('input_timeout_sec').value
        )
        frequency = float(self.get_parameter('check_frequency_hz').value)
        self.maximum_linear_velocity = float(
            self.get_parameter('maximum_linear_velocity').value
        )
        self.maximum_angular_velocity = float(
            self.get_parameter('maximum_angular_velocity').value
        )
        if (
            not self.output_topic
            or min(
                self.input_timeout_sec,
                frequency,
                self.maximum_linear_velocity,
                self.maximum_angular_velocity,
            ) <= 0.0
        ):
            raise ValueError('control gate topics, rates, timeouts, and limits must be valid')

        self._raw_command: Twist | None = None
        self._raw_received_at: float | None = None
        self._publisher = None
        self._ownership_failure = False
        self._last_warning_at = float('-inf')
        self.create_subscription(
            Twist,
            str(self.get_parameter('input_topic').value),
            self._on_raw_command,
            10,
        )
        self.create_timer(1.0 / frequency, self._gate_timer)
        if not self.armed:
            self.get_logger().warning(
                'BRNE control gate is DISARMED and will not create a /cmd_vel publisher'
            )

    def _on_raw_command(self, message: Twist) -> None:
        self._raw_command = message
        self._raw_received_at = time.monotonic()

    def _gate_timer(self) -> None:
        if not self.armed or self._ownership_failure:
            return
        foreign = self._foreign_publishers()
        if foreign:
            self._revoke_ownership(
                'foreign /cmd_vel publisher detected: ' + ', '.join(foreign)
            )
            return
        if self._publisher is None:
            self._publisher = self.create_publisher(Twist, self.output_topic, 10)
            self.get_logger().warning(
                'BRNE control gate ARMED; this node now owns /cmd_vel'
            )
        command = self._validated_fresh_command()
        self._publisher.publish(command if command is not None else Twist())

    def _foreign_publishers(self) -> list[str]:
        endpoints = self.get_publishers_info_by_topic(self.output_topic)
        return sorted({
            f'{endpoint.node_namespace.rstrip("/")}/{endpoint.node_name}'
            for endpoint in endpoints
            if not endpoint_is_self(
                endpoint,
                node_name=self.get_name(),
                node_namespace=self.get_namespace(),
            )
        })

    def _validated_fresh_command(self) -> Twist | None:
        if self._raw_command is None or self._raw_received_at is None:
            return None
        if time.monotonic() - self._raw_received_at > self.input_timeout_sec:
            return None
        raw = self._raw_command
        values = (
            raw.linear.x,
            raw.linear.y,
            raw.linear.z,
            raw.angular.x,
            raw.angular.y,
            raw.angular.z,
        )
        validated = validated_diff_drive_command(
            values,
            maximum_linear_velocity=self.maximum_linear_velocity,
            maximum_angular_velocity=self.maximum_angular_velocity,
        )
        if validated is None:
            self._warn_throttled('rejected non-finite, unsupported, or out-of-bounds raw Twist')
            return None
        linear_velocity, angular_velocity = validated
        command = Twist()
        command.linear.x = linear_velocity
        command.angular.z = angular_velocity
        return command

    def _revoke_ownership(self, reason: str) -> None:
        if self._publisher is not None:
            self._publisher.publish(Twist())
            self.destroy_publisher(self._publisher)
            self._publisher = None
        self._ownership_failure = True
        self.get_logger().error(f'BRNE control ownership revoked: {reason}')

    def _warn_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_warning_at >= 2.0:
            self.get_logger().warning(message)
            self._last_warning_at = now

    def publish_shutdown_stop(self) -> None:
        if self._publisher is not None:
            self._publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = BrneControlGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_shutdown_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
