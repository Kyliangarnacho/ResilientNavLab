"""Relay validated Phase 10 navigation data into the isolated BRNE inputs."""

from __future__ import annotations

import copy
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from .waypoint_adapter import (
    select_local_waypoint,
    transform_points_se2,
    yaw_from_quaternion,
)


PLAN_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class BrneShadowInputAdapter(Node):
    """Forward only fresh odom plus a TF-converted local waypoint to BRNE."""

    def __init__(self):
        """Create a read-only bridge from Phase 10 data to /brne inputs."""
        super().__init__('brne_shadow_input_adapter')
        self.declare_parameter('source_odom_topic', '/odometry/filtered')
        self.declare_parameter('source_plan_topic', '/plan')
        self.declare_parameter('brne_odom_topic', '/brne/odom')
        self.declare_parameter('brne_goal_topic', '/brne/goal_pose')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_frequency_hz', 10.0)
        self.declare_parameter('odom_max_age_sec', 0.25)
        self.declare_parameter('odom_max_receipt_age_sec', 0.5)
        self.declare_parameter('plan_max_age_sec', 1.5)
        self.declare_parameter('plan_max_receipt_age_sec', 3.0)
        self.declare_parameter('tf_timeout_sec', 0.05)
        self.declare_parameter('local_goal_distance', 0.8)
        self.declare_parameter('maximum_path_nearest_distance', 0.75)

        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.odom_max_age_sec = float(self.get_parameter('odom_max_age_sec').value)
        self.odom_max_receipt_age_sec = float(
            self.get_parameter('odom_max_receipt_age_sec').value
        )
        self.plan_max_age_sec = float(self.get_parameter('plan_max_age_sec').value)
        self.plan_max_receipt_age_sec = float(
            self.get_parameter('plan_max_receipt_age_sec').value
        )
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)
        self.local_goal_distance = float(
            self.get_parameter('local_goal_distance').value
        )
        self.maximum_path_nearest_distance = float(
            self.get_parameter('maximum_path_nearest_distance').value
        )
        frequency = float(self.get_parameter('publish_frequency_hz').value)
        if min(
            self.odom_max_age_sec, self.odom_max_receipt_age_sec,
            self.plan_max_age_sec, self.plan_max_receipt_age_sec,
            self.tf_timeout_sec, self.local_goal_distance,
            self.maximum_path_nearest_distance, frequency,
        ) <= 0.0:
            raise ValueError('all timeout, distance, and frequency parameters must be positive')

        self.odom_publisher = self.create_publisher(
            Odometry, str(self.get_parameter('brne_odom_topic').value), 10
        )
        self.goal_publisher = self.create_publisher(
            PoseStamped, str(self.get_parameter('brne_goal_topic').value), 10
        )
        self.create_subscription(
            Odometry, str(self.get_parameter('source_odom_topic').value),
            self._odom_callback, qos_profile_sensor_data,
        )
        self.create_subscription(
            Path, str(self.get_parameter('source_plan_topic').value),
            self._plan_callback, PLAN_QOS,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._odom = None
        self._odom_received_at = None
        self._plan = None
        self._plan_received_at = None
        self.forward_count = 0
        self.last_waypoint_index = None
        self.last_nearest_distance = None
        self.last_failure = 'waiting for odometry and Path'
        self.create_timer(1.0 / frequency, self._forward_timer)

    def _odom_callback(self, message):
        """Retain only a finite, correctly-framed healthy-EKF odometry sample."""
        if not self._valid_odom(message):
            self._odom = None
            self._odom_received_at = None
            self.last_failure = 'invalid odometry'
            return
        self._odom = copy.deepcopy(message)
        self._odom_received_at = time.monotonic()

    def _plan_callback(self, message):
        """Retain only a finite map-frame Nav2 Path with a valid source stamp."""
        if not self._valid_path(message):
            self._plan = None
            self._plan_received_at = None
            self.last_failure = 'invalid Path'
            return
        self._plan = copy.deepcopy(message)
        self._plan_received_at = time.monotonic()

    def _forward_timer(self):
        """Publish both isolated inputs together, or stop forwarding entirely."""
        if not self._sources_fresh():
            return
        transform = self._map_to_odom_transform()
        if transform is None:
            return
        transformed_path = transform_points_se2(
            [(pose.pose.position.x, pose.pose.position.y) for pose in self._plan.poses],
            transform.translation.x,
            transform.translation.y,
            yaw_from_quaternion(transform.rotation),
        )
        robot_xy = (
            self._odom.pose.pose.position.x,
            self._odom.pose.pose.position.y,
        )
        selected = select_local_waypoint(
            transformed_path, robot_xy, self.local_goal_distance,
            self.maximum_path_nearest_distance,
        )
        if selected is None:
            self.last_failure = 'Path is not close enough to current odometry'
            return
        index, waypoint, nearest_distance = selected
        goal = PoseStamped()
        goal.header.frame_id = self.odom_frame
        # Anchor the derived goal to the odometry sample used to pick it.
        goal.header.stamp = self._odom.header.stamp
        goal.pose.position.x = float(waypoint[0])
        goal.pose.position.y = float(waypoint[1])
        heading = math.atan2(
            waypoint[1] - robot_xy[1], waypoint[0] - robot_xy[0]
        )
        if not math.isfinite(heading):
            self.last_failure = 'derived waypoint heading is non-finite'
            return
        goal.pose.orientation.z = math.sin(heading / 2.0)
        goal.pose.orientation.w = math.cos(heading / 2.0)
        self.odom_publisher.publish(self._odom)
        self.goal_publisher.publish(goal)
        self.forward_count += 1
        self.last_waypoint_index = index
        self.last_nearest_distance = nearest_distance
        self.last_failure = None

    def _valid_odom(self, message):
        """Enforce the frozen Phase 10 EKF frame and numeric contract."""
        if (
            message.header.frame_id != self.odom_frame
            or message.child_frame_id != self.base_frame
            or self._stamp_nanoseconds(message.header.stamp) <= 0
        ):
            return False
        position = message.pose.pose.position
        values = (position.x, position.y, position.z)
        return all(math.isfinite(value) for value in values) and (
            yaw_from_quaternion(message.pose.pose.orientation) is not None
        )

    def _valid_path(self, message):
        """Accept only the current Phase 10 map-frame Path contract."""
        if (
            message.header.frame_id != self.map_frame
            or self._stamp_nanoseconds(message.header.stamp) <= 0
            or not message.poses
        ):
            return False
        return all(
            pose.header.frame_id == self.map_frame
            and math.isfinite(pose.pose.position.x)
            and math.isfinite(pose.pose.position.y)
            for pose in message.poses
        )

    def _sources_fresh(self):
        """Require both ROS stamps and local receipt ages before forwarding."""
        if self._odom is None or self._plan is None:
            self.last_failure = 'waiting for odometry and Path'
            return False
        if (
            not self._message_fresh(
                self._odom.header.stamp, self._odom_received_at,
                self.odom_max_age_sec, self.odom_max_receipt_age_sec,
            )
            or not self._message_fresh(
                self._plan.header.stamp, self._plan_received_at,
                self.plan_max_age_sec, self.plan_max_receipt_age_sec,
            )
        ):
            self.last_failure = 'stale odometry or Path'
            return False
        return True

    def _message_fresh(self, stamp, received_at, maximum_ros_age, maximum_receipt_age):
        """Reject zero, future, stale, or locally stopped source messages."""
        if received_at is None:
            return False
        stamp_ns = self._stamp_nanoseconds(stamp)
        now_ns = self.get_clock().now().nanoseconds
        age_sec = (now_ns - stamp_ns) / 1e9
        return (
            stamp_ns > 0
            and -0.1 <= age_sec <= maximum_ros_age
            and time.monotonic() - received_at <= maximum_receipt_age
        )

    def _map_to_odom_transform(self):
        """Read map-to-odom TF at the selected odometry sample's exact stamp."""
        if self._odom is None:
            self.last_failure = 'odometry unavailable for timestamped TF lookup'
            return None
        try:
            stamped = self.tf_buffer.lookup_transform(
                self.odom_frame, self.map_frame,
                Time.from_msg(self._odom.header.stamp),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except Exception as error:  # tf2 exceptions vary across Jazzy builds.
            self.last_failure = f'map-to-odom TF unavailable: {type(error).__name__}'
            return None
        transform = stamped.transform
        values = (
            transform.translation.x, transform.translation.y,
            transform.translation.z,
        )
        if not all(math.isfinite(value) for value in values):
            self.last_failure = 'map-to-odom TF has non-finite translation'
            return None
        if yaw_from_quaternion(transform.rotation) is None:
            self.last_failure = 'map-to-odom TF has invalid rotation'
            return None
        return transform

    @staticmethod
    def _stamp_nanoseconds(stamp):
        """Normalize a ROS time message without treating an empty stamp as valid."""
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def main(args=None):
    """Run only the adapter; it neither starts Nav2 nor controls the robot."""
    rclpy.init(args=args)
    node = BrneShadowInputAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
