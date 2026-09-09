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


SECTOR_BLINDNESS_MODEL = 'sector_blindness'
DROPOUT_MODEL = 'dropout'


def validate_model(model):
    """Validate a LaserScan fault model name."""
    if model not in {SECTOR_BLINDNESS_MODEL, DROPOUT_MODEL}:
        raise ValueError('model must be sector_blindness or dropout')


def validate_sector_width(sector_width_rad):
    """Validate a sector width in radians."""
    if sector_width_rad <= 0.0 or sector_width_rad > 2.0 * math.pi:
        raise ValueError('sector_width_rad must be in (0.0, 2*pi]')


def shortest_angular_distance(angle_a, angle_b):
    """Return the normalized shortest angular distance from angle_b to angle_a."""
    return math.atan2(
        math.sin(angle_a - angle_b),
        math.cos(angle_a - angle_b),
    )


def is_angle_in_sector(angle_rad, sector_center_rad, sector_width_rad):
    """Return whether an angle lies within a circular sector."""
    validate_sector_width(sector_width_rad)
    if sector_width_rad >= 2.0 * math.pi:
        return True

    return (
        abs(shortest_angular_distance(angle_rad, sector_center_rad))
        <= sector_width_rad * 0.5
    )


def apply_scan_fault(
    scan_msg,
    *,
    model,
    enabled,
    sector_center_rad,
    sector_width_rad,
    start_time_sec,
    end_time_sec,
):
    """Return a copied LaserScan message with sector blindness applied."""
    validate_model(model)
    validate_sector_width(sector_width_rad)
    validate_time_window(start_time_sec, end_time_sec)

    msg_time_sec = stamp_to_seconds(scan_msg.header.stamp)
    if not is_active_window(
        enabled,
        msg_time_sec,
        start_time_sec,
        end_time_sec,
    ):
        return deepcopy(scan_msg)

    if model == DROPOUT_MODEL:
        return None

    faulted_msg = deepcopy(scan_msg)

    intensities_match_ranges = len(faulted_msg.intensities) == len(
        faulted_msg.ranges
    )
    for index, _range in enumerate(faulted_msg.ranges):
        beam_angle = scan_msg.angle_min + index * scan_msg.angle_increment
        if is_angle_in_sector(
            beam_angle,
            sector_center_rad,
            sector_width_rad,
        ):
            faulted_msg.ranges[index] = float('nan')
            if intensities_match_ranges:
                faulted_msg.intensities[index] = 0.0

    return faulted_msg


def make_scan_fault_status(
    scan_msg,
    *,
    model,
    enabled,
    sector_center_rad,
    sector_width_rad,
    start_time_sec,
    end_time_sec,
    scenario_id,
    scenario_seed,
    event_id,
    source_topic,
    faulted_topic,
):
    """Build a FaultStatus message for the current LaserScan stamp."""
    validate_model(model)
    validate_sector_width(sector_width_rad)
    validate_time_window(start_time_sec, end_time_sec)

    msg_time_sec = stamp_to_seconds(scan_msg.header.stamp)
    status = FaultStatus()
    status.header.stamp = scan_msg.header.stamp
    status.header.frame_id = ''
    status.scenario_id = scenario_id
    status.scenario_seed = scenario_seed
    status.event_id = event_id
    status.source_topic = source_topic
    status.faulted_topic = faulted_topic
    status.sensor = 'lidar'
    status.model = model
    status.start_time = seconds_to_stamp(start_time_sec)
    status.end_time = seconds_to_stamp(end_time_sec)
    status.state = get_fault_state(
        enabled,
        msg_time_sec,
        start_time_sec,
        end_time_sec,
    )
    if model == DROPOUT_MODEL:
        status.severity = 1.0
        status.affected_fields = ['message']
        status.parameters_yaml = f'model: {model}\n'
    else:
        status.severity = sector_width_rad
        status.affected_fields = ['ranges']
        status.parameters_yaml = (
            f'model: {model}\n'
            f'center: {sector_center_rad}\n'
            f'width: {sector_width_rad}\n'
            'invalid_value: nan\n'
        )
    return status
