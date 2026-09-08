"""Move four Gazebo crowd-test pedestrians along bounded tracks."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node

from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


DEFAULT_ENTITY_NAMES = [
    'amcl_test_pedestrian_1',
    'amcl_test_pedestrian_2',
    'amcl_test_pedestrian_3',
    'amcl_test_pedestrian_4',
]
DEFAULT_BASE_X = [-2.0, -1.2, -0.4, 0.5]
DEFAULT_BASE_Y = [-4.7, -2.3, -4.5, -2.5]
DEFAULT_YAW = [
    math.pi / 2.0,
    -math.pi / 2.0,
    math.pi / 2.0,
    -math.pi / 2.0,
]


def oscillating_offset(
    elapsed_sec: float,
    speed: float,
    travel_distance: float,
) -> float:
    """Return a continuous triangle-wave offset between zero and distance."""
    values = (elapsed_sec, speed, travel_distance)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('oscillator inputs must be finite')
    if elapsed_sec < 0.0 or speed <= 0.0 or travel_distance <= 0.0:
        raise ValueError('oscillator time and magnitudes must be positive')
    leg_duration = travel_distance / speed
    phase = elapsed_sec % (2.0 * leg_duration)
    if phase <= leg_duration:
        return speed * phase
    return travel_distance - speed * (phase - leg_duration)


def oscillating_command(
    elapsed_sec: float,
    speed: float,
    leg_duration_sec: float,
) -> float:
    """Return the direction used by the continuous triangle wave."""
    values = (elapsed_sec, speed, leg_duration_sec)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('oscillator inputs must be finite')
    if elapsed_sec < 0.0 or speed <= 0.0 or leg_duration_sec <= 0.0:
        raise ValueError('oscillator time and magnitudes must be positive')
    return speed if int(elapsed_sec / leg_duration_sec) % 2 == 0 else -speed


class AmclCrowdOscillator(Node):
    """Set four pedestrian poses through Gazebo's official pose service."""

    def __init__(self) -> None:
        """Configure the tracks and start the simulation-time oscillator."""
        super().__init__('amcl_crowd_oscillator')
        self.declare_parameter(
            'service_name', '/world/resilient_lab/set_pose'
        )
        self.declare_parameter('entity_names', DEFAULT_ENTITY_NAMES)
        self.declare_parameter('base_x', DEFAULT_BASE_X)
        self.declare_parameter('base_y', DEFAULT_BASE_Y)
        self.declare_parameter('yaw', DEFAULT_YAW)
        self.declare_parameter('height', 0.60)
        self.declare_parameter('speed', 0.25)
        self.declare_parameter('travel_distance', 2.0)
        self.declare_parameter('start_delay_sec', 7.0)
        self.declare_parameter('update_frequency_hz', 10.0)

        self.entity_names = list(
            self.get_parameter('entity_names').value
        )
        self.base_x = list(self.get_parameter('base_x').value)
        self.base_y = list(self.get_parameter('base_y').value)
        self.yaw = list(self.get_parameter('yaw').value)
        self.height = float(self.get_parameter('height').value)
        self.speed = float(self.get_parameter('speed').value)
        self.travel_distance = float(
            self.get_parameter('travel_distance').value
        )
        self.start_delay_sec = float(
            self.get_parameter('start_delay_sec').value
        )
        frequency = float(
            self.get_parameter('update_frequency_hz').value
        )
        count = len(self.entity_names)
        if (
            count == 0
            or len(set(self.entity_names)) != count
            or any(not name for name in self.entity_names)
            or any(len(values) != count for values in (
                self.base_x, self.base_y, self.yaw
            ))
            or self.height <= 0.0
            or self.speed <= 0.0
            or self.travel_distance <= 0.0
            or self.start_delay_sec < 0.0
            or frequency <= 0.0
        ):
            raise ValueError('invalid AMCL crowd oscillator parameters')

        service_name = str(self.get_parameter('service_name').value)
        self.client = self.create_client(SetEntityPose, service_name)
        self.epoch_sec: float | None = None
        self.pending = {}
        self.successful_entities = set()
        self.service_warning_emitted = False
        self.motion_verified = False
        self.last_direction: float | None = None
        self.create_timer(1.0 / frequency, self._timer)

    def _timer(self) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec <= 0.0:
            return
        if self.epoch_sec is None:
            self.epoch_sec = now_sec
        elapsed_sec = now_sec - self.epoch_sec
        if elapsed_sec < self.start_delay_sec:
            return
        if not self.client.service_is_ready():
            if not self.service_warning_emitted:
                self.get_logger().warning(
                    'waiting for Gazebo SetEntityPose service'
                )
                self.service_warning_emitted = True
            return

        motion_time = elapsed_sec - self.start_delay_sec
        offset = oscillating_offset(
            motion_time, self.speed, self.travel_distance
        )
        leg_duration = self.travel_distance / self.speed
        direction = oscillating_command(
            motion_time, self.speed, leg_duration
        )
        if direction != self.last_direction:
            label = 'forward' if direction > 0.0 else 'reverse'
            self.get_logger().info(
                f'crowd pedestrians moving {label} at '
                f'{self.speed:.2f} m/s'
            )
            self.last_direction = direction

        self._collect_responses()
        for index, name in enumerate(self.entity_names):
            if name in self.pending:
                continue
            request = SetEntityPose.Request()
            request.entity.name = name
            request.entity.type = Entity.MODEL
            request.pose.position.x = (
                self.base_x[index] + offset * math.cos(self.yaw[index])
            )
            request.pose.position.y = (
                self.base_y[index] + offset * math.sin(self.yaw[index])
            )
            request.pose.position.z = self.height
            request.pose.orientation.z = math.sin(self.yaw[index] / 2.0)
            request.pose.orientation.w = math.cos(self.yaw[index] / 2.0)
            self.pending[name] = self.client.call_async(request)

    def _collect_responses(self) -> None:
        for name, future in list(self.pending.items()):
            if not future.done():
                continue
            del self.pending[name]
            try:
                response = future.result()
            except Exception as error:
                self.get_logger().error(
                    f'failed to move {name}: {error}'
                )
                continue
            if response is None or not response.success:
                self.get_logger().error(f'Gazebo rejected pose for {name}')
                continue
            self.successful_entities.add(name)

        if (
            not self.motion_verified
            and len(self.successful_entities) == len(self.entity_names)
        ):
            self.get_logger().info(
                'Gazebo confirmed motion commands for all four pedestrians'
            )
            self.motion_verified = True


def main(args=None) -> None:
    """Run the crowd oscillator node."""
    rclpy.init(args=args)
    node = AmclCrowdOscillator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
