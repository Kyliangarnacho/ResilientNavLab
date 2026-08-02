from resilient_nav_fault_injection.imu_fault_models import (
    apply_imu_fault,
    BIAS_MODEL,
    FaultStatusChangeTracker,
    get_fault_state,
    make_imu_fault_status,
    seconds_to_stamp,
    stamp_to_seconds,
    validate_time_window,
)


__all__ = [
    'FaultStatusChangeTracker',
    'get_imu_z_gyro_bias_state',
    'inject_imu_z_gyro_bias',
    'make_imu_z_gyro_bias_status',
    'seconds_to_stamp',
    'stamp_to_seconds',
    'validate_time_window',
]


def get_imu_z_gyro_bias_state(enabled, msg_time_sec, start_time_sec, end_time_sec):
    """Return the FaultStatus state for an IMU z gyro bias fault."""
    return get_fault_state(enabled, msg_time_sec, start_time_sec, end_time_sec)


def make_imu_z_gyro_bias_status(
    imu_msg,
    *,
    enabled,
    bias_rad_s,
    start_time_sec,
    end_time_sec,
    scenario_id,
    scenario_seed,
    event_id,
    source_topic,
    faulted_topic,
):
    """Build a FaultStatus message for the current input IMU timestamp."""
    return make_imu_fault_status(
        imu_msg,
        model=BIAS_MODEL,
        enabled=enabled,
        bias_rad_s=bias_rad_s,
        noise_sigma_rad_s=0.0,
        start_time_sec=start_time_sec,
        end_time_sec=end_time_sec,
        scenario_id=scenario_id,
        scenario_seed=scenario_seed,
        event_id=event_id,
        source_topic=source_topic,
        faulted_topic=faulted_topic,
    )


def inject_imu_z_gyro_bias(
    imu_msg,
    *,
    enabled,
    bias_rad_s,
    start_time_sec,
    end_time_sec,
):
    """Return a copied IMU message with an optional z angular velocity bias."""
    return apply_imu_fault(
        imu_msg,
        model=BIAS_MODEL,
        enabled=enabled,
        bias_rad_s=bias_rad_s,
        noise_sigma_rad_s=0.0,
        start_time_sec=start_time_sec,
        end_time_sec=end_time_sec,
    )
