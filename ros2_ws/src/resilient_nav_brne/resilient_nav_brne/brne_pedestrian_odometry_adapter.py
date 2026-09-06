"""Convert one or more Gazebo body-frame odometry streams into PedestrianArray."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from resilient_nav_interfaces.msg import Pedestrian, PedestrianArray

from .pedestrian_state import state_in_odom


@dataclass(frozen=True)
class _InputSource:
    topic: str
    child_frame: str
    pedestrian_id: int


def _yaw_from_quaternion(quaternion) -> float | None:
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not all(math.isfinite(value) for value in values):
        return None
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-9:
        return None
    x_value, y_value, z_value, w_value = (value / norm for value in values)
    return math.atan2(
        2.0 * (w_value * z_value + x_value * y_value),
        1.0 - 2.0 * (y_value * y_value + z_value * z_value),
    )


def _csv_values(value) -> list[str]:
    """Split an explicitly scalar launch parameter without accepting blanks."""
    text = str(value).strip()
    return [] if not text else [item.strip() for item in text.split(',')]


class BrnePedestrianOdometryAdapter(Node):
    """Relay finite body-frame motion as one ID-stable, odom-frame array."""

    def __init__(self):
        super().__init__('brne_pedestrian_odometry_adapter')
        # Scalar settings preserve Scene 1.  Parallel arrays are Scene 2's
        # minimal aggregation extension, avoiding competing singleton writers.
        self.declare_parameter('input_topic', '/brne/pedestrian/odometry')
        self.declare_parameter('input_child_frame', 'brne_pedestrian')
        self.declare_parameter('pedestrian_id', 1)
        # Empty Python lists infer BYTE_ARRAY in Jazzy rclpy before launch can
        # override them.  Keep Scene 2's source lists explicitly scalar.
        self.declare_parameter('input_topics_csv', '')
        self.declare_parameter('input_child_frames_csv', '')
        self.declare_parameter('pedestrian_ids_csv', '')
        self.declare_parameter('output_topic', '/brne/pedestrians')
        self.declare_parameter('input_frame', 'gazebo_world')
        self.declare_parameter('output_frame', 'odom')
        self.declare_parameter('odom_origin_world_x', -3.5)
        self.declare_parameter('odom_origin_world_y', -3.5)
        self.declare_parameter('odom_origin_world_yaw', 0.0)
        self.declare_parameter('maximum_stamp_age_sec', 0.25)

        self.input_frame = str(self.get_parameter('input_frame').value)
        self.output_frame = str(self.get_parameter('output_frame').value)
        self.origin = (
            float(self.get_parameter('odom_origin_world_x').value),
            float(self.get_parameter('odom_origin_world_y').value),
            float(self.get_parameter('odom_origin_world_yaw').value),
        )
        self.maximum_stamp_age_sec = float(self.get_parameter('maximum_stamp_age_sec').value)
        if (
            not self.input_frame or not self.output_frame
            or self.maximum_stamp_age_sec <= 0.0
            or not all(math.isfinite(value) for value in self.origin)
        ):
            raise ValueError('pedestrian adapter parameters are invalid')

        input_topics = _csv_values(self.get_parameter('input_topics_csv').value)
        input_child_frames = _csv_values(
            self.get_parameter('input_child_frames_csv').value
        )
        pedestrian_ids_text = _csv_values(
            self.get_parameter('pedestrian_ids_csv').value
        )
        if not input_topics and not input_child_frames and not pedestrian_ids_text:
            input_topics = [str(self.get_parameter('input_topic').value)]
            input_child_frames = [str(self.get_parameter('input_child_frame').value)]
            pedestrian_ids = [int(self.get_parameter('pedestrian_id').value)]
        elif not (input_topics and input_child_frames and pedestrian_ids_text):
            raise ValueError('pedestrian source CSV parameters must be specified together')
        else:
            try:
                pedestrian_ids = [int(value) for value in pedestrian_ids_text]
            except ValueError as error:
                raise ValueError('pedestrian_ids_csv must contain integers') from error
        if not (
            input_topics and len(input_topics) == len(input_child_frames) == len(pedestrian_ids)
        ):
            raise ValueError('pedestrian source arrays must be non-empty and equal length')
        self.sources = tuple(
            _InputSource(str(topic), str(child_frame), int(pedestrian_id))
            for topic, child_frame, pedestrian_id in zip(
                input_topics, input_child_frames, pedestrian_ids
            )
        )
        if (
            any(not source.topic or not source.child_frame or source.pedestrian_id < 0
                for source in self.sources)
            or len({source.pedestrian_id for source in self.sources}) != len(self.sources)
        ):
            raise ValueError('pedestrian sources require non-negative unique IDs')

        self.publisher = self.create_publisher(
            PedestrianArray, str(self.get_parameter('output_topic').value), 10
        )
        self._latest: dict[int, tuple[Pedestrian, float]] = {}
        for source in self.sources:
            self.create_subscription(
                Odometry, source.topic,
                lambda message, input_source=source: self._on_odometry(message, input_source),
                qos_profile_sensor_data,
            )

    def _on_odometry(self, message: Odometry, source: _InputSource) -> None:
        if not self._valid_header(message, source):
            return
        position = message.pose.pose.position
        twist = message.twist.twist.linear
        yaw = _yaw_from_quaternion(message.pose.pose.orientation)
        values = (position.x, position.y, position.z, twist.x, twist.y)
        if yaw is None or not all(math.isfinite(value) for value in values):
            return
        state = state_in_odom(
            world_x=position.x, world_y=position.y, world_yaw=yaw,
            twist_x=twist.x, twist_y=twist.y,
            odom_origin_world_x=self.origin[0],
            odom_origin_world_y=self.origin[1],
            odom_origin_world_yaw=self.origin[2],
        )
        if state is None:
            return
        pedestrian = Pedestrian()
        pedestrian.header.stamp = message.header.stamp
        pedestrian.header.frame_id = self.output_frame
        pedestrian.id = source.pedestrian_id
        pedestrian.pose.position.x = state.x
        pedestrian.pose.position.y = state.y
        pedestrian.pose.orientation.z = math.sin(state.yaw / 2.0)
        pedestrian.pose.orientation.w = math.cos(state.yaw / 2.0)
        pedestrian.velocity.linear.x = state.velocity_x
        pedestrian.velocity.linear.y = state.velocity_y
        self._latest[source.pedestrian_id] = (pedestrian, time.monotonic())

        output = PedestrianArray()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.output_frame
        now_wall = time.monotonic()
        output.pedestrians.extend(
            pedestrian
            for pedestrian_id, (pedestrian, received_at) in sorted(self._latest.items())
            if now_wall - received_at <= self.maximum_stamp_age_sec
        )
        self.publisher.publish(output)

    def _valid_header(self, message: Odometry, source: _InputSource) -> bool:
        if (
            message.header.frame_id != self.input_frame
            or message.child_frame_id != source.child_frame
        ):
            return False
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        if stamp_ns <= 0:
            return False
        age_sec = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        return -0.1 <= age_sec <= self.maximum_stamp_age_sec


def main(args=None):
    rclpy.init(args=args)
    node = BrnePedestrianOdometryAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
