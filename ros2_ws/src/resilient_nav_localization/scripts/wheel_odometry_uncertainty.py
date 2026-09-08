#!/usr/bin/env python3
"""Model small encoder yaw error and covariance on bridged wheel odometry."""

from copy import deepcopy
import math
import random

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


LINEAR_X_COVARIANCE_INDEX = 0
YAW_COVARIANCE_INDEX = 35


class WheelYawMeasurementModel:
    """Apply deterministic yaw quantization and fixed-seed white noise."""

    def __init__(
        self,
        *,
        quantization_step_rad: float,
        noise_stddev_rad: float,
        random_seed: int,
    ) -> None:
        """Validate configuration and initialize the deterministic PRNG."""
        if (
            not math.isfinite(quantization_step_rad)
            or quantization_step_rad <= 0.0
            or not math.isfinite(noise_stddev_rad)
            or noise_stddev_rad < 0.0
            or isinstance(random_seed, bool)
            or not isinstance(random_seed, int)
        ):
            raise ValueError(
                'wheel yaw measurement model parameters are invalid'
            )
        self.quantization_step_rad = quantization_step_rad
        self.noise_stddev_rad = noise_stddev_rad
        self._random = random.Random(random_seed)

    def apply(self, message: Odometry) -> Odometry | None:
        """Return a copy with only its planar pose yaw measurement changed."""
        yaw = _yaw_from_quaternion(message.pose.pose.orientation)
        if yaw is None:
            return None
        quantized_yaw = (
            round(yaw / self.quantization_step_rad)
            * self.quantization_step_rad
        )
        measured_yaw = _normalize_angle(
            quantized_yaw
            + self._random.gauss(0.0, self.noise_stddev_rad)
        )
        output = deepcopy(message)
        output.pose.pose.orientation.x = 0.0
        output.pose.pose.orientation.y = 0.0
        output.pose.pose.orientation.z = math.sin(measured_yaw / 2.0)
        output.pose.pose.orientation.w = math.cos(measured_yaw / 2.0)
        return output


def attach_nominal_covariance(
    message: Odometry,
    *,
    linear_x_variance: float,
    yaw_variance: float,
    yaw_rate_variance: float,
) -> Odometry | None:
    """Fill zero variances without overwriting existing sensor values."""
    requested = (linear_x_variance, yaw_variance, yaw_rate_variance)
    if not all(math.isfinite(value) and value > 0.0 for value in requested):
        return None
    source = (
        float(message.twist.covariance[LINEAR_X_COVARIANCE_INDEX]),
        float(message.pose.covariance[YAW_COVARIANCE_INDEX]),
        float(message.twist.covariance[YAW_COVARIANCE_INDEX]),
    )
    if not all(math.isfinite(value) and value >= 0.0 for value in source):
        return None

    output = deepcopy(message)
    if source[0] == 0.0:
        output.twist.covariance[
            LINEAR_X_COVARIANCE_INDEX
        ] = linear_x_variance
    if source[1] == 0.0:
        output.pose.covariance[YAW_COVARIANCE_INDEX] = yaw_variance
    if source[2] == 0.0:
        output.twist.covariance[YAW_COVARIANCE_INDEX] = yaw_rate_variance
    return output


class WheelOdometryUncertainty(Node):
    """Publish wheel odometry with modest yaw error and usable covariance."""

    def __init__(self) -> None:
        """Configure the shared wheel-odometry measurement boundary."""
        super().__init__('wheel_odometry_uncertainty')
        self.declare_parameter('input_topic', '/wheel/odometry/raw')
        self.declare_parameter('output_topic', '/wheel/odometry')
        self.declare_parameter('linear_x_variance', 0.0004)
        self.declare_parameter('yaw_variance', 0.0004)
        self.declare_parameter('yaw_rate_variance', 0.0004)
        self.declare_parameter('yaw_quantization_step_rad', 0.002)
        self.declare_parameter('yaw_measurement_noise_stddev_rad', 0.0005)
        self.declare_parameter('random_seed', 240907)
        self._linear_x_variance = float(
            self.get_parameter('linear_x_variance').value
        )
        self._yaw_variance = float(self.get_parameter('yaw_variance').value)
        self._yaw_rate_variance = float(
            self.get_parameter('yaw_rate_variance').value
        )
        if not all(
            math.isfinite(value) and value > 0.0
            for value in (
                self._linear_x_variance,
                self._yaw_variance,
                self._yaw_rate_variance,
            )
        ):
            raise ValueError(
                'wheel odometry variances must be finite and positive'
            )
        self._yaw_model = WheelYawMeasurementModel(
            quantization_step_rad=float(
                self.get_parameter('yaw_quantization_step_rad').value
            ),
            noise_stddev_rad=float(
                self.get_parameter(
                    'yaw_measurement_noise_stddev_rad'
                ).value
            ),
            random_seed=int(self.get_parameter('random_seed').value),
        )

        self._publisher = self.create_publisher(
            Odometry,
            str(self.get_parameter('output_topic').value),
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('input_topic').value),
            self._on_odometry,
            qos_profile_sensor_data,
        )

    def _on_odometry(self, message: Odometry) -> None:
        modeled = self._yaw_model.apply(message)
        output = None if modeled is None else attach_nominal_covariance(
            modeled,
            linear_x_variance=self._linear_x_variance,
            yaw_variance=self._yaw_variance,
            yaw_rate_variance=self._yaw_rate_variance,
        )
        if output is not None:
            self._publisher.publish(output)
        else:
            self.get_logger().warning(
                'suppressed wheel odometry: invalid yaw or covariance'
            )


def _yaw_from_quaternion(quaternion) -> float | None:
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not all(math.isfinite(value) for value in values):
        return None
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-9:
        return None
    x_value, y_value, z_value, w_value = (
        value / norm for value in values
    )
    return math.atan2(
        2.0 * (w_value * z_value + x_value * y_value),
        1.0 - 2.0 * (y_value * y_value + z_value * z_value),
    )


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None) -> None:
    """Run the uncertainty boundary node."""
    rclpy.init(args=args)
    node = WheelOdometryUncertainty()
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
