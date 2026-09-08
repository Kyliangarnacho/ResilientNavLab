"""ROS adapter publishing scan-matched LiDAR translation velocity."""

from math import hypot, isfinite

from geometry_msgs.msg import TwistWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from resilient_nav_interfaces.msg import LidarOdometryStatus
from sensor_msgs.msg import LaserScan

from .scan_matching import (
    PlanarScanMatcher,
    ScanMatcherConfig,
    laser_scan_points,
    scan_observability,
)


LINEAR_X_COVARIANCE_INDEX = 0


class LidarOdometry(Node):
    """Estimate forward velocity from adjacent scans without wheel odometry."""

    def __init__(self) -> None:
        """Configure the matcher and its private measurement topic."""
        super().__init__('lidar_odometry')
        self._declare_parameters()
        self._matcher = PlanarScanMatcher(ScanMatcherConfig(
            min_points=self._integer_parameter('min_points'),
            max_points=self._integer_parameter('max_points'),
            max_iterations=self._integer_parameter('max_iterations'),
            max_correspondence_distance_m=self._float_parameter(
                'max_correspondence_distance_m'
            ),
            robust_mad_scale=self._float_parameter('robust_mad_scale'),
            robust_residual_floor_m=self._float_parameter(
                'robust_residual_floor_m'
            ),
            trim_fraction=self._float_parameter('trim_fraction'),
            min_inlier_ratio=self._float_parameter('min_inlier_ratio'),
            max_rmse_m=self._float_parameter('max_rmse_m'),
            max_translation_m=self._float_parameter('max_translation_m'),
            max_rotation_rad=self._float_parameter('max_rotation_rad'),
            normal_neighbor_max_distance_m=self._float_parameter(
                'normal_neighbor_max_distance_m'
            ),
            normal_max_curvature_ratio=self._float_parameter(
                'normal_max_curvature_ratio'
            ),
            min_point_to_line_observability=self._float_parameter(
                'min_point_to_line_observability'
            ),
            point_to_line_damping=self._float_parameter(
                'point_to_line_damping'
            ),
        ))
        self._minimum_linear_variance = self._float_parameter(
            'minimum_linear_variance'
        )
        self._maximum_scan_interval_sec = self._float_parameter(
            'maximum_scan_interval_sec'
        )
        self._maximum_linear_speed_mps = self._float_parameter(
            'maximum_linear_speed_mps'
        )
        if (
            not isfinite(self._minimum_linear_variance)
            or self._minimum_linear_variance <= 0.0
            or not isfinite(self._maximum_scan_interval_sec)
            or self._maximum_scan_interval_sec <= 0.0
            or not isfinite(self._maximum_linear_speed_mps)
            or self._maximum_linear_speed_mps <= 0.0
        ):
            raise ValueError(
                'LiDAR odometry timing, speed limit, or covariance is invalid'
            )
        self._previous_points = None
        self._previous_stamp_sec = None
        self._previous_frame = None
        self._publisher = self.create_publisher(
            TwistWithCovarianceStamped,
            self._string_parameter('output_topic'),
            qos_profile_sensor_data,
        )
        self._status_publisher = self.create_publisher(
            LidarOdometryStatus,
            self._string_parameter('status_topic'),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            self._string_parameter('scan_topic'),
            self._on_scan,
            qos_profile_sensor_data,
        )

    def _declare_parameters(self) -> None:
        defaults = {
            'scan_topic': '/faulted/scan',
            'output_topic': '/lidar/odometry/twist',
            'status_topic': '/lidar/odometry/status',
            'min_points': 40,
            'max_points': 180,
            'max_iterations': 12,
            'max_correspondence_distance_m': 0.35,
            'robust_mad_scale': 3.5,
            'robust_residual_floor_m': 0.02,
            'trim_fraction': 0.80,
            'min_inlier_ratio': 0.45,
            'max_rmse_m': 0.08,
            'max_translation_m': 0.20,
            'max_rotation_rad': 0.25,
            'normal_neighbor_max_distance_m': 0.50,
            'normal_max_curvature_ratio': 0.20,
            'min_point_to_line_observability': 0.05,
            'point_to_line_damping': 1.0e-6,
            'minimum_linear_variance': 0.0025,
            'maximum_scan_interval_sec': 0.50,
            'maximum_linear_speed_mps': 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _on_scan(self, message: LaserScan) -> None:
        points = laser_scan_points(
            message.ranges,
            angle_min=float(message.angle_min),
            angle_increment=float(message.angle_increment),
            range_min=float(message.range_min),
            range_max=float(message.range_max),
            max_points=self._matcher.config.max_points,
        )
        stamp_sec = _stamp_sec(message)
        frame = message.header.frame_id
        if points is None or stamp_sec is None or not frame:
            self._reset_reference()
            return
        if (
            self._previous_points is None
            or self._previous_stamp_sec is None
            or self._previous_frame != frame
        ):
            self._set_reference(points, stamp_sec, frame)
            return

        interval_sec = stamp_sec - self._previous_stamp_sec
        if (
            interval_sec <= 0.0
            or interval_sec > self._maximum_scan_interval_sec
        ):
            self._set_reference(points, stamp_sec, frame)
            return
        match = self._matcher.match(self._previous_points, points)
        self._set_reference(points, stamp_sec, frame)
        observability = scan_observability(points)
        if match is None:
            self._publish_status(message, observability, None)
            return

        forward_displacement, lateral_displacement = (
            match.translation_current_frame
        )
        velocity = _bounded_planar_velocity(
            forward_displacement,
            lateral_displacement,
            interval_sec,
            self._maximum_linear_speed_mps,
        )
        if velocity is None:
            self._publish_status(message, observability, None)
            return
        forward_velocity, lateral_velocity = velocity
        output = TwistWithCovarianceStamped()
        output.header = message.header
        output.twist.twist.linear.x = forward_velocity
        output.twist.twist.linear.y = lateral_velocity
        output.twist.twist.angular.z = match.rotation_rad / interval_sec
        output.twist.covariance[LINEAR_X_COVARIANCE_INDEX] = max(
            self._minimum_linear_variance,
            (match.rmse_m / interval_sec) ** 2,
        )
        self._publish_status(message, observability, output, match)
        self._publisher.publish(output)

    def _publish_status(
        self,
        scan: LaserScan,
        observability: float,
        velocity: TwistWithCovarianceStamped | None,
        match=None,
    ) -> None:
        """Publish the GT-free ICP gate evidence for the Fusion Supervisor."""
        status = LidarOdometryStatus()
        status.header = scan.header
        status.valid = velocity is not None and match is not None
        status.icp_observability = float(observability)
        if status.valid:
            status.velocity = velocity.twist
            status.icp_rmse_m = float(match.rmse_m)
            status.icp_inlier_ratio = float(match.inlier_ratio)
        self._status_publisher.publish(status)

    def _set_reference(self, points, stamp_sec: float, frame: str) -> None:
        self._previous_points = points
        self._previous_stamp_sec = stamp_sec
        self._previous_frame = frame

    def _reset_reference(self) -> None:
        self._previous_points = None
        self._previous_stamp_sec = None
        self._previous_frame = None

    def _string_parameter(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _integer_parameter(self, name: str) -> int:
        return int(self.get_parameter(name).value)


def _stamp_sec(message: LaserScan) -> float | None:
    stamp = message.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        return None
    value = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
    return value if isfinite(value) else None


def _bounded_planar_velocity(
    forward_displacement: float,
    lateral_displacement: float,
    interval_sec: float,
    maximum_speed_mps: float,
) -> tuple[float, float] | None:
    """Reject finite-looking ICP matches that imply impossible robot speed."""
    values = (
        forward_displacement,
        lateral_displacement,
        interval_sec,
        maximum_speed_mps,
    )
    if not all(isfinite(value) for value in values):
        return None
    if interval_sec <= 0.0 or maximum_speed_mps <= 0.0:
        return None
    forward_velocity = forward_displacement / interval_sec
    lateral_velocity = lateral_displacement / interval_sec
    if hypot(forward_velocity, lateral_velocity) > maximum_speed_mps:
        return None
    return forward_velocity, lateral_velocity


def main(args=None) -> None:
    """Run the LiDAR odometry adapter."""
    rclpy.init(args=args)
    node = LidarOdometry()
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
