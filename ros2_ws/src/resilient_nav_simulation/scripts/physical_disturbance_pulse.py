#!/usr/bin/env python3
"""Apply one simulation-time-bounded physical wrench through ros_gz_bridge."""

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Wrench
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity, EntityWrench


PERSISTENT_TOPIC = '/world/resilient_lab/wrench/persistent'
CLEAR_TOPIC = '/world/resilient_lab/wrench/clear'


class PhysicalDisturbancePulse(Node):
    """Publish an external push or wheel-resistance pulse in simulation time."""

    def __init__(self):
        super().__init__('physical_disturbance_pulse')
        self._declare_parameters()
        self._mode = str(self.get_parameter('mode').value)
        self._delay_after_motion_sec = float(
            self.get_parameter('delay_after_motion_sec').value
        )
        self._duration_sec = float(self.get_parameter('duration_sec').value)
        self._pulse_count = int(self.get_parameter('pulse_count').value)
        self._pulse_interval_sec = float(
            self.get_parameter('pulse_interval_sec').value
        )
        self._start_time_sec = None
        self._end_time_sec = None
        self._robot_model_name = str(
            self.get_parameter('robot_model_name').value
        )
        self._impact_force_x_n = float(
            self.get_parameter('impact_force_x_n').value
        )
        self._impact_force_y_n = float(
            self.get_parameter('impact_force_y_n').value
        )
        self._wheel_block_force_n = float(
            self.get_parameter('wheel_block_force_n').value
        )
        self._wheel_block_yaw_torque_nm = float(
            self.get_parameter('wheel_block_yaw_torque_nm').value
        )
        self._blocked_wheel = str(self.get_parameter('blocked_wheel').value)
        if self._mode not in {'external_impact', 'wheel_block'}:
            raise ValueError('mode must be external_impact or wheel_block')
        if self._delay_after_motion_sec < 0.0 or self._duration_sec <= 0.0:
            raise ValueError('disturbance duration must be positive')
        if self._pulse_count < 1 or self._pulse_interval_sec < 0.0:
            raise ValueError('disturbance pulse schedule is invalid')
        if self._blocked_wheel not in {'left', 'right', 'both'}:
            raise ValueError('blocked_wheel must be left, right, or both')

        self._wrench_publisher = self.create_publisher(
            EntityWrench, PERSISTENT_TOPIC, 10
        )
        self._clear_publisher = self.create_publisher(Entity, CLEAR_TOPIC, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_command, 10)
        self._active = False
        self._clear_cycles = 0
        self._pulse_index = 0
        self.create_timer(0.05, self._on_timer)

    def _declare_parameters(self):
        defaults = {
            'mode': 'external_impact',
            'delay_after_motion_sec': 7.0,
            'duration_sec': 0.25,
            'pulse_count': 1,
            'pulse_interval_sec': 3.0,
            'robot_model_name': 'resilient_nav_robot',
            'impact_force_x_n': 0.0,
            'impact_force_y_n': 12.0,
            'wheel_block_force_n': 3.0,
            'wheel_block_yaw_torque_nm': 0.08,
            'blocked_wheel': 'left',
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

    def _on_command(self, message):
        if self._start_time_sec is not None:
            return
        magnitude = abs(message.linear.x) + abs(message.angular.z)
        if magnitude <= 1.0e-4:
            return
        now_sec = self.get_clock().now().nanoseconds / 1_000_000_000.0
        self._start_time_sec = now_sec + self._delay_after_motion_sec
        self._end_time_sec = self._start_time_sec + self._duration_sec
        self.get_logger().info(
            f'physical disturbance {self._mode} armed for '
            f'{self._pulse_count} pulse(s), first at {self._start_time_sec:.3f}s'
        )

    def _on_timer(self):
        if self._start_time_sec is None or self._end_time_sec is None:
            return
        now_sec = self.get_clock().now().nanoseconds / 1_000_000_000.0
        if now_sec < self._start_time_sec:
            return
        if now_sec < self._end_time_sec:
            for message in self._active_messages():
                self._wrench_publisher.publish(message)
            if not self._active:
                self.get_logger().info(
                    f'physical disturbance {self._mode} pulse '
                    f'{self._pulse_index + 1}/{self._pulse_count} active '
                    f'at {now_sec:.3f}s'
                )
            self._active = True
            return
        if not self._active:
            return
        for entity in self._target_entities():
            self._clear_publisher.publish(entity)
        self._clear_cycles += 1
        if self._clear_cycles >= 3:
            self.get_logger().info(
                f'physical disturbance {self._mode} pulse '
                f'{self._pulse_index + 1}/{self._pulse_count} cleared '
                f'at {now_sec:.3f}s'
            )
            self._pulse_index += 1
            if self._pulse_index >= self._pulse_count:
                rclpy.shutdown()
                return
            self._start_time_sec = now_sec + self._pulse_interval_sec
            self._end_time_sec = self._start_time_sec + self._duration_sec
            self._active = False
            self._clear_cycles = 0

    def _active_messages(self):
        if self._mode == 'external_impact':
            wrench = Wrench()
            wrench.force.x = self._impact_force_x_n
            wrench.force.y = self._impact_force_y_n
            return [self._entity_wrench(wrench)]

        wrench = Wrench()
        wrench.force.x = -abs(self._wheel_block_force_n)
        if self._blocked_wheel == 'left':
            wrench.torque.z = abs(self._wheel_block_yaw_torque_nm)
        elif self._blocked_wheel == 'right':
            wrench.torque.z = -abs(self._wheel_block_yaw_torque_nm)
        return [self._entity_wrench(wrench)]

    def _target_entities(self):
        return [self._entity()]

    def _entity_wrench(self, wrench):
        message = EntityWrench()
        message.header.stamp = self.get_clock().now().to_msg()
        message.entity = self._entity()
        message.wrench = wrench
        return message

    def _entity(self):
        entity = Entity()
        entity.name = self._robot_model_name
        entity.type = Entity.MODEL
        return entity


def main(args=None):
    """Run the bounded disturbance publisher."""
    rclpy.init(args=args)
    node = PhysicalDisturbancePulse()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
