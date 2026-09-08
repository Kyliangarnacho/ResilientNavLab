"""Tests for nominal covariance attached to bridged wheel odometry."""

import importlib.util
import math
from pathlib import Path

from nav_msgs.msg import Odometry
import pytest


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / 'scripts'
    / 'wheel_odometry_uncertainty.py'
)
SPEC = importlib.util.spec_from_file_location(
    'wheel_odometry_uncertainty', MODULE_PATH
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def odometry_at_yaw(yaw):
    message = Odometry()
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return message


def yaw_from_odometry(message):
    orientation = message.pose.pose.orientation
    return math.atan2(
        2.0 * orientation.w * orientation.z,
        1.0 - 2.0 * orientation.z * orientation.z,
    )


def test_yaw_is_quantized_without_mutating_the_source():
    source = odometry_at_yaw(0.1234)
    model = MODULE.WheelYawMeasurementModel(
        quantization_step_rad=0.01,
        noise_stddev_rad=0.0,
        random_seed=7,
    )

    output = model.apply(source)

    assert output is not None
    assert yaw_from_odometry(output) == pytest.approx(0.12)
    assert yaw_from_odometry(source) == pytest.approx(0.1234)


def test_fixed_seed_produces_repeatable_tiny_measurement_noise():
    """Two model instances with one seed must emit the same sequence."""
    first = MODULE.WheelYawMeasurementModel(
        quantization_step_rad=0.001,
        noise_stddev_rad=0.0002,
        random_seed=240907,
    )
    second = MODULE.WheelYawMeasurementModel(
        quantization_step_rad=0.001,
        noise_stddev_rad=0.0002,
        random_seed=240907,
    )
    source = odometry_at_yaw(0.1234)

    first_sequence = [yaw_from_odometry(first.apply(source)) for _ in range(4)]
    second_sequence = [yaw_from_odometry(second.apply(source)) for _ in range(4)]

    assert first_sequence == pytest.approx(second_sequence)
    assert len(set(first_sequence)) == 4
    assert max(abs(value - 0.123) for value in first_sequence) < 0.001


def test_invalid_yaw_measurement_model_input_fails_closed():
    with pytest.raises(ValueError):
        MODULE.WheelYawMeasurementModel(
            quantization_step_rad=0.0,
            noise_stddev_rad=0.0002,
            random_seed=1,
        )

    model = MODULE.WheelYawMeasurementModel(
        quantization_step_rad=0.001,
        noise_stddev_rad=0.0002,
        random_seed=1,
    )
    invalid = Odometry()
    invalid.pose.pose.orientation.w = 0.0
    assert model.apply(invalid) is None


def test_zero_bridge_covariances_receive_nominal_variances():
    source = Odometry()
    source.pose.pose.orientation.w = 1.0

    output = MODULE.attach_nominal_covariance(
        source,
        linear_x_variance=0.0004,
        yaw_variance=0.0009,
        yaw_rate_variance=0.0016,
    )

    assert output is not None
    assert output.twist.covariance[0] == 0.0004
    assert output.pose.covariance[35] == 0.0009
    assert output.twist.covariance[35] == 0.0016
    assert source.twist.covariance[0] == 0.0
    assert source.pose.covariance[35] == 0.0


def test_existing_covariances_are_preserved_and_bad_values_fail_closed():
    source = Odometry()
    source.twist.covariance[0] = 0.1
    source.pose.covariance[35] = 0.2
    source.twist.covariance[35] = 0.3

    output = MODULE.attach_nominal_covariance(
        source,
        linear_x_variance=0.0004,
        yaw_variance=0.0004,
        yaw_rate_variance=0.0004,
    )

    assert output is not None
    assert output.twist.covariance[0] == 0.1
    assert output.pose.covariance[35] == 0.2
    assert output.twist.covariance[35] == 0.3

    source.pose.covariance[35] = float('nan')
    assert MODULE.attach_nominal_covariance(
        source,
        linear_x_variance=0.0004,
        yaw_variance=0.0004,
        yaw_rate_variance=0.0004,
    ) is None
