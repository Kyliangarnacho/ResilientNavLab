"""Drive the fixed Gazebo pedestrian only after a real Nav2 Path exists."""

from __future__ import annotations

import math
import time

from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool
from std_msgs.msg import Float64

from .crossing_scenario import CrossingScenario


READY_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class BrneCrossingPedestrianDriver(Node):
    """Issue bounded velocity only for the pedestrian's crossing joint."""

    def __init__(self):
        super().__init__('brne_crossing_pedestrian_driver')
        self.declare_parameter(
            'command_topic', '/brne/pedestrian/joint_velocity'
        )
        self.declare_parameter('odometry_topic', '/brne/pedestrian/odometry')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('ready_topic', '/brne/ready')
        self.declare_parameter('speed', 0.25)
        self.declare_parameter('motion_axis', 'y')
        self.declare_parameter('target_world_y', -2.50)
        self.declare_parameter('world_y_direction', 1)
        self.declare_parameter('start_delay_sec', 0.1)
        self.declare_parameter('maximum_duration_sec', 10.0)
        self.declare_parameter('input_timeout_sec', 0.5)
        self.declare_parameter('publish_frequency_hz', 20.0)

        self.motion_axis = str(self.get_parameter('motion_axis').value)
        if self.motion_axis not in ('x', 'y'):
            raise ValueError('pedestrian motion_axis must be x or y')
        self.scenario = CrossingScenario(
            speed=float(self.get_parameter('speed').value),
            target_world_y=float(self.get_parameter('target_world_y').value),
            world_y_direction=int(self.get_parameter('world_y_direction').value),
            start_delay_sec=float(self.get_parameter('start_delay_sec').value),
            maximum_duration_sec=float(
                self.get_parameter('maximum_duration_sec').value
            ),
        )
        self.input_timeout_sec = float(
            self.get_parameter('input_timeout_sec').value
        )
        frequency = float(self.get_parameter('publish_frequency_hz').value)
        if self.input_timeout_sec <= 0.0 or frequency <= 0.0:
            raise ValueError('crossing driver timeout and frequency must be positive')
        self.publisher = self.create_publisher(
            Float64, str(self.get_parameter('command_topic').value), 10
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odometry_topic').value),
            self._on_odometry,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter('plan_topic').value),
            self._on_plan,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('ready_topic').value),
            self._on_ready,
            READY_QOS,
        )
        self._world_y: float | None = None
        self._odom_received_at: float | None = None
        self._plan_received_at: float | None = None
        self._brne_ready = False
        self._reported_start = False
        self._reported_stop = False
        self.create_timer(1.0 / frequency, self._timer)

    def _on_odometry(self, message: Odometry) -> None:
        world_position = (
            message.pose.pose.position.x
            if self.motion_axis == 'x'
            else message.pose.pose.position.y
        )
        if (
            message.header.frame_id != 'gazebo_world'
            or not math.isfinite(world_position)
        ):
            self._world_y = None
            self._odom_received_at = None
            return
        # CrossingScenario remains a scalar state machine; target_world_y is
        # retained as its backwards-compatible parameter name.
        self._world_y = world_position
        self._odom_received_at = time.monotonic()

    def _on_plan(self, message: Path) -> None:
        if message.header.frame_id == 'map' and message.poses:
            self._plan_received_at = time.monotonic()

    def _on_ready(self, message: Bool) -> None:
        self._brne_ready = bool(message.data)

    def _timer(self) -> None:
        now_wall = time.monotonic()
        inputs_fresh = (
            self._brne_ready
            and
            self._odom_received_at is not None
            and self._plan_received_at is not None
            and now_wall - self._odom_received_at <= self.input_timeout_sec
            and now_wall - self._plan_received_at <= self.input_timeout_sec * 3.0
        )
        now_sec = self.get_clock().now().nanoseconds / 1e9
        speed = self.scenario.command(
            now_sec=now_sec,
            plan_ready=inputs_fresh,
            world_y=self._world_y,
        )
        self.publisher.publish(Float64(data=speed))
        if speed > 0.0 and not self._reported_start:
            self.get_logger().info('fixed pedestrian crossing started')
            self._reported_start = True
        if (
            self._reported_start and speed == 0.0
            and (self.scenario.complete or self.scenario.failed)
            and not self._reported_stop
        ):
            outcome = 'complete' if self.scenario.complete else 'timed out'
            self.get_logger().info(f'fixed pedestrian crossing {outcome}; command stopped')
            self._reported_stop = True

    def publish_shutdown_stop(self) -> None:
        self.publisher.publish(Float64())


def main(args=None):
    rclpy.init(args=args)
    node = BrneCrossingPedestrianDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_shutdown_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
