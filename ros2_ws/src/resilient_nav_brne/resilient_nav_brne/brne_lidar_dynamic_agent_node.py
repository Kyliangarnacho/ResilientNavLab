"""Build BRNE dynamic-agent input only from timestamped robot LiDAR scans."""

from __future__ import annotations

import math
import time

from geometry_msgs.msg import Point
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from resilient_nav_interfaces.msg import Pedestrian, PedestrianArray
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .lidar_dynamic_tracker import (
    agents_within_range,
    clusters_from_scan,
    LidarDynamicTracker,
    LidarTrackerConfig,
    transform_clusters,
)


class BrneLidarDynamicAgentNode(Node):
    """Convert clustered scan residual motion into the existing BRNE input."""

    def __init__(self) -> None:
        super().__init__('brne_lidar_dynamic_agent_node')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('scan_frame', 'lidar_link')
        self.declare_parameter('output_frame', 'odom')
        self.declare_parameter('output_topic', '/brne/pedestrians')
        self.declare_parameter('marker_topic', '/brne/sensor_dynamic_agents')
        self.declare_parameter('maximum_scan_age_sec', 0.25)
        self.declare_parameter('transform_timeout_sec', 0.05)
        self.declare_parameter('cluster_gap', 0.12)
        self.declare_parameter('minimum_cluster_points', 3)
        self.declare_parameter('maximum_usable_range', 6.0)
        self.declare_parameter('maximum_dynamic_agent_range', 4.0)
        self.declare_parameter('maximum_track_diameter', 0.60)
        self.declare_parameter('association_distance', 0.35)
        self.declare_parameter('minimum_common_matches', 3)
        self.declare_parameter('history_size', 6)
        self.declare_parameter('minimum_confirmations', 6)
        self.declare_parameter('common_motion_inlier_distance', 0.02)
        self.declare_parameter('dynamic_minimum_speed', 0.08)
        self.declare_parameter('dynamic_minimum_displacement', 0.04)
        self.declare_parameter('direction_consistency', 0.75)
        self.declare_parameter('minimum_direction_step', 0.005)
        self.declare_parameter('velocity_ema_alpha', 0.25)
        self.declare_parameter('velocity_ema_stability_window', 3)
        self.declare_parameter('velocity_ema_max_direction_change_rad', 0.35)
        self.declare_parameter('velocity_ema_outlier_direction_change_rad', 0.70)
        self.declare_parameter('stationary_maximum_speed', 0.04)
        self.declare_parameter('stationary_confirmation_frames', 8)

        self.scan_frame = str(self.get_parameter('scan_frame').value)
        self.output_frame = str(self.get_parameter('output_frame').value)
        self.maximum_scan_age_sec = float(
            self.get_parameter('maximum_scan_age_sec').value
        )
        self.transform_timeout_sec = float(
            self.get_parameter('transform_timeout_sec').value
        )
        self.maximum_dynamic_agent_range = float(
            self.get_parameter('maximum_dynamic_agent_range').value
        )
        if (
            not self.scan_frame
            or not self.output_frame
            or self.maximum_scan_age_sec <= 0.0
            or self.transform_timeout_sec <= 0.0
            or self.maximum_dynamic_agent_range <= 0.0
        ):
            raise ValueError('LiDAR dynamic-agent node parameters are invalid')
        self.tracker = LidarDynamicTracker(LidarTrackerConfig(
            cluster_gap=float(self.get_parameter('cluster_gap').value),
            minimum_cluster_points=int(
                self.get_parameter('minimum_cluster_points').value
            ),
            maximum_usable_range=float(
                self.get_parameter('maximum_usable_range').value
            ),
            maximum_track_diameter=float(
                self.get_parameter('maximum_track_diameter').value
            ),
            association_distance=float(
                self.get_parameter('association_distance').value
            ),
            minimum_common_matches=int(
                self.get_parameter('minimum_common_matches').value
            ),
            history_size=int(self.get_parameter('history_size').value),
            minimum_confirmations=int(
                self.get_parameter('minimum_confirmations').value
            ),
            common_motion_inlier_distance=float(
                self.get_parameter('common_motion_inlier_distance').value
            ),
            dynamic_minimum_speed=float(
                self.get_parameter('dynamic_minimum_speed').value
            ),
            dynamic_minimum_displacement=float(
                self.get_parameter('dynamic_minimum_displacement').value
            ),
            direction_consistency=float(
                self.get_parameter('direction_consistency').value
            ),
            minimum_direction_step=float(
                self.get_parameter('minimum_direction_step').value
            ),
            velocity_ema_alpha=float(
                self.get_parameter('velocity_ema_alpha').value
            ),
            velocity_ema_stability_window=int(
                self.get_parameter('velocity_ema_stability_window').value
            ),
            velocity_ema_max_direction_change_rad=float(
                self.get_parameter(
                    'velocity_ema_max_direction_change_rad'
                ).value
            ),
            velocity_ema_outlier_direction_change_rad=float(
                self.get_parameter(
                    'velocity_ema_outlier_direction_change_rad'
                ).value
            ),
            stationary_maximum_speed=float(
                self.get_parameter('stationary_maximum_speed').value
            ),
            stationary_confirmation_frames=int(
                self.get_parameter('stationary_confirmation_frames').value
            ),
        ))
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            PedestrianArray, str(self.get_parameter('output_topic').value), 10
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, str(self.get_parameter('marker_topic').value), 10
        )
        self._pending_scan: LaserScan | None = None
        self._last_log_at = float('-inf')
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / 30.0, self._process_latest_scan)

    def _on_scan(self, message: LaserScan) -> None:
        # Keep one scan until its own timestamped TF arrives. Replacing it on
        # every callback can starve exact-time lookup when TF trails LiDAR.
        if message.header.frame_id == self.scan_frame and self._pending_scan is None:
            self._pending_scan = message

    def _process_latest_scan(self) -> None:
        scan = self._pending_scan
        if scan is None:
            return
        stamp_ns = (
            int(scan.header.stamp.sec) * 1_000_000_000
            + int(scan.header.stamp.nanosec)
        )
        if stamp_ns <= 0:
            self._pending_scan = None
            self.tracker.reset()
            return
        age_sec = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if age_sec > self.maximum_scan_age_sec or age_sec < -0.1:
            self._pending_scan = None
            self.tracker.reset()
            return
        try:
            stamped_transform = self.tf_buffer.lookup_transform(
                self.output_frame,
                scan.header.frame_id,
                Time.from_msg(scan.header.stamp),
                timeout=Duration(seconds=self.transform_timeout_sec),
            )
        except Exception:
            return
        self._pending_scan = None
        quaternion = stamped_transform.transform.rotation
        yaw = _yaw_from_quaternion(quaternion)
        if yaw is None:
            self.tracker.reset()
            return
        scan_clusters = clusters_from_scan(
            scan.ranges,
            angle_min=float(scan.angle_min),
            angle_increment=float(scan.angle_increment),
            range_min=float(scan.range_min),
            range_max=float(scan.range_max),
            config=self.tracker.config,
        )
        translation = stamped_transform.transform.translation
        odom_clusters = transform_clusters(
            scan_clusters,
            translation=(translation.x, translation.y),
            yaw=yaw,
        )
        dynamic_candidates, diagnostics = self.tracker.update(
            odom_clusters,
            stamp_ns / 1e9,
        )
        agents = agents_within_range(
            dynamic_candidates,
            observer_position=(translation.x, translation.y),
            maximum_range=self.maximum_dynamic_agent_range,
        )
        output = PedestrianArray()
        output.header = scan.header
        output.header.frame_id = self.output_frame
        for agent in agents:
            pedestrian = Pedestrian()
            pedestrian.header = output.header
            pedestrian.id = agent.track_id
            pedestrian.pose.position.x = agent.position[0]
            pedestrian.pose.position.y = agent.position[1]
            heading = math.atan2(agent.velocity[1], agent.velocity[0])
            pedestrian.pose.orientation.z = math.sin(heading / 2.0)
            pedestrian.pose.orientation.w = math.cos(heading / 2.0)
            pedestrian.velocity.linear.x = agent.velocity[0]
            pedestrian.velocity.linear.y = agent.velocity[1]
            output.pedestrians.append(pedestrian)
        self.publisher.publish(output)
        self.marker_publisher.publish(_agent_markers(output))

        now_wall = time.monotonic()
        if now_wall - self._last_log_at >= 1.0:
            self.get_logger().info(
                'LiDAR agent tracking: '
                f'clusters={diagnostics.cluster_count}, '
                f'matches={diagnostics.matched_cluster_count}, '
                f'common_drift={diagnostics.common_drift}, '
                f'common_rotation_rad={diagnostics.common_rotation_rad:.5f}, '
                f'tracks={diagnostics.track_count}, '
                f'dynamic_candidates={diagnostics.dynamic_agent_count}, '
                f'dynamic_agents={len(agents)}, '
                'states='
                f'{[(agent.track_id, agent.position, agent.velocity) for agent in agents]}'
            )
            self._last_log_at = now_wall


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


def _agent_markers(message: PedestrianArray) -> MarkerArray:
    markers = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    markers.markers.append(clear)
    for pedestrian in message.pedestrians:
        body = Marker()
        body.header = message.header
        body.ns = 'brne_sensor_dynamic_agents'
        body.id = int(pedestrian.id) * 2
        body.type = Marker.SPHERE
        body.action = Marker.ADD
        body.pose = pedestrian.pose
        body.pose.position.z = 0.30
        body.scale.x = 0.36
        body.scale.y = 0.36
        body.scale.z = 0.60
        body.color.r = 1.0
        body.color.g = 0.15
        body.color.b = 0.85
        body.color.a = 0.85
        markers.markers.append(body)

        velocity = Marker()
        velocity.header = message.header
        velocity.ns = 'brne_sensor_dynamic_velocity'
        velocity.id = int(pedestrian.id) * 2 + 1
        velocity.type = Marker.ARROW
        velocity.action = Marker.ADD
        start = Point(
            x=pedestrian.pose.position.x,
            y=pedestrian.pose.position.y,
            z=0.35,
        )
        end = Point(
            x=start.x + pedestrian.velocity.linear.x,
            y=start.y + pedestrian.velocity.linear.y,
            z=start.z,
        )
        velocity.points.extend((start, end))
        velocity.scale.x = 0.035
        velocity.scale.y = 0.07
        velocity.scale.z = 0.09
        velocity.color.r = 1.0
        velocity.color.g = 0.85
        velocity.color.b = 0.0
        velocity.color.a = 0.95
        markers.markers.append(velocity)
    return markers


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BrneLidarDynamicAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
