from copy import deepcopy
import math

from resilient_nav_fault_injection.imu_fault_models import (
    get_fault_state,
    is_active_window,
    seconds_to_stamp,
    stamp_to_seconds,
    validate_time_window,
)
from resilient_nav_interfaces.msg import FaultStatus


FREEZE_MODEL = 'freeze'
BIAS_MODEL = 'bias'


def validate_model(model):
    """Validate a wheel odometry fault model name."""
    if model not in {FREEZE_MODEL, BIAS_MODEL}:
        raise ValueError('model must be freeze or bias')


def validate_linear_bias(linear_bias_mps):
    """Keep the wheel-speed bias finite and experimentally bounded."""
    if not math.isfinite(linear_bias_mps) or abs(linear_bias_mps) > 1.0:
        raise ValueError('linear_bias_mps must be finite and within [-1, 1]')


class WheelOdometryFreezeModel:
    """Freeze wheel odometry pose and twist inside an active time window."""

    def __init__(self):
        self._snapshot = None

    def apply(
        self,
        odometry_msg,
        *,
        model,
        enabled,
        start_time_sec,
        end_time_sec,
        linear_bias_mps=0.0,
    ):
        """Return a copied odometry message with the selected wheel fault."""
        validate_model(model)
        validate_time_window(start_time_sec, end_time_sec)
        validate_linear_bias(linear_bias_mps)

        msg_time_sec = stamp_to_seconds(odometry_msg.header.stamp)
        if not is_active_window(
            enabled,
            msg_time_sec,
            start_time_sec,
            end_time_sec,
        ):
            self._snapshot = None
            return deepcopy(odometry_msg)

        if model == BIAS_MODEL:
            self._snapshot = None
            faulted_msg = deepcopy(odometry_msg)
            faulted_msg.twist.twist.linear.x += linear_bias_mps
            return faulted_msg

        if self._snapshot is None:
            self._snapshot = (
                deepcopy(odometry_msg.pose),
                deepcopy(odometry_msg.twist),
            )

        faulted_msg = deepcopy(odometry_msg)
        faulted_msg.pose = deepcopy(self._snapshot[0])
        faulted_msg.twist = deepcopy(self._snapshot[1])
        return faulted_msg


def make_wheel_fault_status(
    odometry_msg,
    *,
    model,
    enabled,
    start_time_sec,
    end_time_sec,
    scenario_id,
    scenario_seed,
    event_id,
    source_topic,
    faulted_topic,
    linear_bias_mps=0.0,
):
    """Build a FaultStatus message for the current wheel odometry stamp."""
    validate_model(model)
    validate_time_window(start_time_sec, end_time_sec)
    validate_linear_bias(linear_bias_mps)

    msg_time_sec = stamp_to_seconds(odometry_msg.header.stamp)
    status = FaultStatus()
    status.header.stamp = odometry_msg.header.stamp
    status.header.frame_id = ''
    status.scenario_id = scenario_id
    status.scenario_seed = scenario_seed
    status.event_id = event_id
    status.source_topic = source_topic
    status.faulted_topic = faulted_topic
    status.sensor = 'wheel_odometry'
    status.model = model
    status.start_time = seconds_to_stamp(start_time_sec)
    status.end_time = seconds_to_stamp(end_time_sec)
    status.state = get_fault_state(
        enabled,
        msg_time_sec,
        start_time_sec,
        end_time_sec,
    )
    if model == BIAS_MODEL:
        status.severity = abs(linear_bias_mps)
        status.affected_fields = ['twist.twist.linear.x']
        status.parameters_yaml = (
            f'model: {model}\n'
            f'linear_bias_mps: {linear_bias_mps}\n'
            f'enabled: {str(enabled).lower()}\n'
        )
    else:
        status.severity = 1.0
        status.affected_fields = ['pose', 'twist']
        status.parameters_yaml = (
            f'model: {model}\n'
            f'enabled: {str(enabled).lower()}\n'
        )
    return status
