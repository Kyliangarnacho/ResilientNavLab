"""Read-only runtime summary for the healthy Phase 9 mapping baseline."""

from collections import deque
import json
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from resilient_nav_slam.probe_metrics import (
    freshness_seconds,
    occupancy_ratios,
    quaternion_yaw,
    scan_rate_hz,
    stamp_nanoseconds,
)


class SlamProbe(Node):
    """Subscribe and report only; this node has no publishers or control path."""

    def __init__(self):
        super().__init__(
            'slam_probe',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True),
            ],
        )
        self.declare_parameter('report_period_sec', 2.0)
        self.declare_parameter('scan_rate_window', 32)

        report_period_sec = float(
            self.get_parameter('report_period_sec').value,
        )
        scan_rate_window = int(self.get_parameter('scan_rate_window').value)
        if report_period_sec <= 0.0:
            raise ValueError('report_period_sec must be positive')
        if scan_rate_window < 2:
            raise ValueError('scan_rate_window must be at least 2')

        self._scan_receipt_times = deque(maxlen=scan_rate_window)
        self._map_summary = None
        self._scan_frame = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            LaserScan,
            '/scan',
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self._on_map,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_timer(report_period_sec, self._report)

    def _on_scan(self, message: LaserScan) -> None:
        """Record receive timing and the declared scan frame, never scan payload."""
        self._scan_receipt_times.append(time.monotonic())
        self._scan_frame = message.header.frame_id

    def _on_map(self, message: OccupancyGrid) -> None:
        """Store map metadata and aggregate cell classes, never republish the map."""
        self._map_summary = {
            'resolution': message.info.resolution,
            'width': message.info.width,
            'height': message.info.height,
            'ratios': occupancy_ratios(message.data),
        }

    def _transform_status(self, target_frame: str, source_frame: str) -> dict:
        """Query the latest transform and return connectivity plus age."""
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return {'connected': False, 'freshness_sec': None}

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return {
            'connected': True,
            'freshness_sec': freshness_seconds(
                self.get_clock().now().nanoseconds,
                stamp_nanoseconds(transform.header.stamp),
            ),
            'translation': {
                'x': translation.x,
                'y': translation.y,
                'z': translation.z,
            },
            'yaw_rad': quaternion_yaw(
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            ),
        }

    def _report(self) -> None:
        """Log the structured health-neutral mapping observation snapshot."""
        map_to_odom = self._transform_status('map', 'odom')
        odom_to_base = self._transform_status('odom', 'base_footprint')
        base_to_lidar = self._transform_status('base_footprint', 'lidar_link')
        links = {
            'map_to_odom': map_to_odom,
            'odom_to_base_footprint': odom_to_base,
            'base_footprint_to_lidar_link': base_to_lidar,
        }
        report = {
            'scan_rate_hz': scan_rate_hz(list(self._scan_receipt_times)),
            'scan_frame': self._scan_frame,
            'map_received': self._map_summary is not None,
            'map': self._map_summary,
            'map_to_odom': map_to_odom,
            'odom_to_base_footprint': odom_to_base,
            'map_to_odom_freshness_sec': map_to_odom['freshness_sec'],
            'odom_to_base_footprint_freshness_sec': odom_to_base[
                'freshness_sec'
            ],
            'tf_connectivity': {
                name: status['connected'] for name, status in links.items()
            },
            'tf_chain_connected': all(
                status['connected'] for status in links.values()
            ),
        }
        self.get_logger().info(json.dumps(report, sort_keys=True))


def main(args=None) -> None:
    """Run the probe until ROS shuts it down."""
    rclpy.init(args=args)
    node = SlamProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
