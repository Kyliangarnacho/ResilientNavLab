from copy import deepcopy
import math

from builtin_interfaces.msg import Time
from resilient_nav_interfaces.msg import FaultStatus


BIAS_MODEL = 'bias'
DROPOUT_MODEL = 'dropout'
FIXED_DELAY_MODEL = 'fixed_delay'
GAUSSIAN_NOISE_MODEL = 'gaussian_noise'


def stamp_to_seconds(stamp):
    """Convert a ROS builtin_interfaces/Time stamp to floating seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def validate_time_window(start_time_sec, end_time_sec):
    """Validate an injection time window expressed in simulation seconds."""
    if start_time_sec < 0.0:
        raise ValueError('start_time_sec must be non-negative')
    if end_time_sec <= start_time_sec:
        raise ValueError('end_time_sec must be greater than start_time_sec')


def validate_noise_sigma(noise_sigma_rad_s):
    """Validate a Gaussian noise standard deviation in rad/s."""
    if noise_sigma_rad_s < 0.0:
        raise ValueError('noise_sigma_rad_s must be non-negative')


def validate_dropout_probability(dropout_probability):
    """Validate a message dropout probability."""
    if dropout_probability < 0.0 or dropout_probability > 1.0:
        raise ValueError('dropout_probability must be in [0.0, 1.0]')


def validate_delay_sec(delay_sec):
    """Validate a fixed message delivery delay in seconds."""
    if delay_sec < 0.0:
        raise ValueError('delay_sec must be non-negative')


def validate_model(model):
    """Validate an IMU fault model name."""
    if model not in (
        BIAS_MODEL,
        DROPOUT_MODEL,
        FIXED_DELAY_MODEL,
        GAUSSIAN_NOISE_MODEL,
    ):
        raise ValueError(
            'model must be one of: bias, dropout, fixed_delay, gaussian_noise'
        )


def seconds_to_stamp(time_sec):
    """Convert non-negative floating seconds to a ROS Time message."""
    if time_sec < 0.0:
        raise ValueError('time_sec must be non-negative')

    whole_seconds = math.floor(time_sec)
    nanoseconds = int(round((time_sec - whole_seconds) * 1e9))
    if nanoseconds == 1000000000:
        whole_seconds += 1
        nanoseconds = 0

    stamp = Time()
    stamp.sec = whole_seconds
    stamp.nanosec = nanoseconds
    return stamp


def is_active_window(enabled, msg_time_sec, start_time_sec, end_time_sec):
    """Return whether a fault is enabled and active for a message time."""
    validate_time_window(start_time_sec, end_time_sec)
    return enabled and start_time_sec <= msg_time_sec < end_time_sec


def get_fault_state(enabled, msg_time_sec, start_time_sec, end_time_sec):
    """Return the FaultStatus state for a configured time window."""
    validate_time_window(start_time_sec, end_time_sec)

    if not enabled:
        return FaultStatus.CANCELLED
    if msg_time_sec < start_time_sec:
        return FaultStatus.SCHEDULED
    if msg_time_sec < end_time_sec:
        return FaultStatus.ACTIVE
    return FaultStatus.ENDED


def apply_imu_fault(
    imu_msg,
    *,
    model,
    enabled,
    bias_rad_s,
    noise_sigma_rad_s,
    dropout_probability=0.30,
    delay_sec=0.50,
    start_time_sec,
    end_time_sec,
    noise_rng=None,
):
    """Return a copied faulted IMU message, or None when dropped."""
    validate_model(model)
    validate_time_window(start_time_sec, end_time_sec)
    validate_noise_sigma(noise_sigma_rad_s)
    validate_dropout_probability(dropout_probability)
    validate_delay_sec(delay_sec)

    faulted_msg = deepcopy(imu_msg)
    msg_time_sec = stamp_to_seconds(imu_msg.header.stamp)

    if not is_active_window(
        enabled,
        msg_time_sec,
        start_time_sec,
        end_time_sec,
    ):
        return faulted_msg

    if model == BIAS_MODEL:
        faulted_msg.angular_velocity.z += bias_rad_s
    elif model == DROPOUT_MODEL:
        if noise_rng is None:
            raise ValueError('noise_rng is required for dropout')
        if noise_rng.random() < dropout_probability:
            return None
    elif model == GAUSSIAN_NOISE_MODEL:
        if noise_rng is None:
            raise ValueError('noise_rng is required for gaussian_noise')
        faulted_msg.angular_velocity.z += noise_rng.gauss(
            0.0,
            noise_sigma_rad_s,
        )

    return faulted_msg


def make_imu_fault_status(
    imu_msg,
    *,
    model,
    enabled,
    bias_rad_s,
    noise_sigma_rad_s,
    dropout_probability=0.30,
    delay_sec=0.50,
    start_time_sec,
    end_time_sec,
    scenario_id,
    scenario_seed,
    event_id,
    source_topic,
    faulted_topic,
):
    """Build a FaultStatus message for the current input IMU timestamp."""
    validate_model(model)
    validate_noise_sigma(noise_sigma_rad_s)
    validate_dropout_probability(dropout_probability)
    validate_delay_sec(delay_sec)

    msg_time_sec = stamp_to_seconds(imu_msg.header.stamp)
    status = FaultStatus()
    status.header.stamp = imu_msg.header.stamp
    status.header.frame_id = ''
    status.scenario_id = scenario_id
    status.scenario_seed = scenario_seed
    status.event_id = event_id
    status.source_topic = source_topic
    status.faulted_topic = faulted_topic
    status.sensor = 'imu'
    status.model = _status_model_name(model)
    status.start_time = seconds_to_stamp(start_time_sec)
    status.end_time = seconds_to_stamp(end_time_sec)
    status.state = get_fault_state(
        enabled,
        msg_time_sec,
        start_time_sec,
        end_time_sec,
    )
    status.severity = _severity(
        model,
        bias_rad_s,
        noise_sigma_rad_s,
        dropout_probability,
        delay_sec,
    )
    status.affected_fields = _affected_fields(model)
    status.parameters_yaml = _parameters_yaml(
        model,
        enabled,
        bias_rad_s,
        noise_sigma_rad_s,
        dropout_probability,
        delay_sec,
        scenario_seed,
    )
    return status


def _status_model_name(model):
    if model == BIAS_MODEL:
        return 'z_gyro_bias'
    if model == DROPOUT_MODEL:
        return DROPOUT_MODEL
    if model == FIXED_DELAY_MODEL:
        return FIXED_DELAY_MODEL
    return GAUSSIAN_NOISE_MODEL


def _severity(
    model,
    bias_rad_s,
    noise_sigma_rad_s,
    dropout_probability,
    delay_sec,
):
    if model == BIAS_MODEL:
        return abs(bias_rad_s)
    if model == DROPOUT_MODEL:
        return dropout_probability
    if model == FIXED_DELAY_MODEL:
        return delay_sec
    return noise_sigma_rad_s


def _affected_fields(model):
    if model == DROPOUT_MODEL:
        return ['message_delivery']
    if model == FIXED_DELAY_MODEL:
        return ['message_delivery_time']
    return ['angular_velocity.z']


def _parameters_yaml(
    model,
    enabled,
    bias_rad_s,
    noise_sigma_rad_s,
    dropout_probability,
    delay_sec,
    scenario_seed,
):
    if model == BIAS_MODEL:
        return (
            f'enabled: {str(enabled).lower()}\n'
            f'bias_rad_s: {bias_rad_s}\n'
        )

    if model == DROPOUT_MODEL:
        return (
            f'model: {model}\n'
            f'dropout_probability: {dropout_probability}\n'
            f'seed: {scenario_seed}\n'
        )

    if model == FIXED_DELAY_MODEL:
        return (
            f'model: {model}\n'
            f'delay_sec: {delay_sec}\n'
        )

    return (
        f'model: {model}\n'
        f'sigma: {noise_sigma_rad_s}\n'
        f'seed: {scenario_seed}\n'
    )


class ImuFixedDelayQueue:
    """Queue IMU messages for ordered fixed-delay release."""

    def __init__(self, delay_sec=0.50):
        validate_delay_sec(delay_sec)
        self._delay_sec = delay_sec
        self._pending = []

    @property
    def delay_sec(self):
        return self._delay_sec

    def handle_message(
        self,
        imu_msg,
        *,
        enabled,
        current_ros_time_sec,
        start_time_sec,
        end_time_sec,
    ):
        """Queue active-window messages or return an immediate passthrough."""
        validate_time_window(start_time_sec, end_time_sec)
        msg_time_sec = stamp_to_seconds(imu_msg.header.stamp)
        if not is_active_window(
            enabled,
            msg_time_sec,
            start_time_sec,
            end_time_sec,
        ):
            return deepcopy(imu_msg)

        release_time_sec = current_ros_time_sec + self._delay_sec
        self._pending.append((release_time_sec, deepcopy(imu_msg)))
        return None

    def pop_ready(self, current_ros_time_sec):
        """Return due messages in original input order."""
        ready = []
        while self._pending and self._pending[0][0] <= current_ros_time_sec:
            ready.append(self._pending.pop(0)[1])
        return ready

    def __len__(self):
        return len(self._pending)


class FaultStatusChangeTracker:
    """Track and return statuses only for the first sample or state changes."""

    def __init__(self):
        self._last_state = None

    def first_or_changed(self, status_msg):
        """Return a status only when it is the first or the state changed."""
        if self._last_state == status_msg.state:
            return None
        self._last_state = status_msg.state
        return status_msg
