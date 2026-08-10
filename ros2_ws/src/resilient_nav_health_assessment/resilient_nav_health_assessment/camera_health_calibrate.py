"""Collect descriptive camera feature baselines from a ROS Image stream."""

import csv
from datetime import datetime, timezone
import json
from math import isfinite
import os
from pathlib import Path
import shlex
import subprocess
import time

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from resilient_nav_health_assessment.camera_health_features import (
    compute_camera_health_features,
)
from resilient_nav_interfaces.msg import FaultStatus
from sensor_msgs.msg import Image


DEFAULT_IMAGE_TOPIC = '/camera/c920/image_raw'
DEFAULT_OUTPUT_DIRECTORY = '/tmp/phase7_2_calibration/'
DEFAULT_VIDEO_DEVICE = '/dev/video0'
DEFAULT_DURATION_SEC = 60.0
DEFAULT_SCENARIO_LABEL = 'unspecified'
DEFAULT_SESSION_ID = ''
DEFAULT_FAULT_STATUS_TOPIC = '/fault_injection/status'

TRUTH_FIELDS = (
    'event_id',
    'truth_model',
    'truth_state',
    'severity',
)

VISUAL_METRICS = (
    'mean_gray',
    'gray_std',
    'p05',
    'p95',
    'dark_ratio',
    'bright_ratio',
    'laplacian_variance',
    'edge_density',
    'entropy',
    'frame_diff_mean',
)

SAMPLE_FIELDS = (
    'scenario_label',
    'session_id',
    *TRUTH_FIELDS,
    'header_stamp_sec',
    'header_stamp_nanosec',
    'receive_time_sec',
    'interarrival_sec',
    *VISUAL_METRICS,
)

ENCODING_CHANNEL_ORDER = {
    'mono8': 'rgb',
    '8uc1': 'rgb',
    'rgb8': 'rgb',
    'rgba8': 'rgb',
    'bgr8': 'bgr',
    'bgra8': 'bgr',
}


def _utc_now_text(epoch_sec=None):
    """Return an ISO-8601 UTC timestamp for metadata and snapshots."""
    if epoch_sec is None:
        epoch_sec = time.time()
    return datetime.fromtimestamp(epoch_sec, timezone.utc).isoformat()


def _default_session_id(epoch_sec):
    """Return a sortable UTC session identifier with microsecond precision."""
    return datetime.fromtimestamp(epoch_sec, timezone.utc).strftime(
        '%Y%m%dT%H%M%S.%fZ'
    )


def _validated_scenario_label(scenario_label):
    """Return a non-empty scenario label suitable for CSV and JSON metadata."""
    normalized = str(scenario_label).strip()
    if not normalized:
        raise ValueError('scenario_label must not be empty')
    return normalized


def _validated_session_id(session_id, started_receive_time_sec):
    """Return an explicit or generated safe single-directory session ID."""
    normalized = str(session_id).strip()
    if not normalized:
        normalized = _default_session_id(started_receive_time_sec)
    if normalized in {'.', '..'} or Path(normalized).name != normalized:
        raise ValueError(
            'session_id must be one directory name without path separators'
        )
    if any(character in normalized for character in ('/', '\\', '\x00')):
        raise ValueError(
            'session_id must be one directory name without path separators'
        )
    return normalized


def image_message_to_numpy(message, bridge):
    """Convert a supported ROS Image while retaining its channel order."""
    encoding = message.encoding.lower()
    if encoding not in ENCODING_CHANNEL_ORDER:
        supported = ', '.join(sorted(ENCODING_CHANNEL_ORDER))
        raise ValueError(
            f'unsupported Image encoding {message.encoding!r}; '
            f'supported encodings: {supported}'
        )
    try:
        image = bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
    except CvBridgeError as error:
        raise ValueError(
            f'cv_bridge failed to convert encoding {message.encoding!r}: {error}'
        ) from error
    return np.asarray(image), ENCODING_CHANNEL_ORDER[encoding]


def capture_v4l2_controls(
    video_device=DEFAULT_VIDEO_DEVICE,
    *,
    runner=subprocess.run,
    captured_at_epoch_sec=None,
):
    """Read V4L2 controls without issuing any control write operation."""
    command = [
        'v4l2-ctl',
        '--device',
        video_device,
        '--list-ctrls-menus',
    ]
    lines = [
        f'captured_at_utc: {_utc_now_text(captured_at_epoch_sec)}',
        f'command: {shlex.join(command)}',
        'read_only: true',
    ]
    try:
        result = runner(
            command,
            capture_output=True,
            text=True,
            timeout=10.0,
            check=False,
        )
    except (OSError, subprocess.SubprocessError) as error:
        lines.extend([
            'returncode: unavailable',
            'stdout:',
            '',
            'stderr:',
            str(error),
        ])
        return '\n'.join(lines) + '\n'

    lines.extend([
        f'returncode: {result.returncode}',
        'stdout:',
        result.stdout.rstrip(),
        'stderr:',
        result.stderr.rstrip(),
    ])
    return '\n'.join(lines) + '\n'


def _descriptive_statistics(values, percentile_names):
    """Return finite population statistics or nulls for an empty series."""
    finite_values = [float(value) for value in values if isfinite(value)]
    result = {'count': len(finite_values), 'mean': None}
    result.update({name: None for name in percentile_names})
    if not finite_values:
        return result

    array = np.asarray(finite_values, dtype=np.float64)
    result['mean'] = float(np.mean(array))
    percentiles = {
        'p05': 5,
        'p50': 50,
        'p95': 95,
        'p99': 99,
    }
    for name in percentile_names:
        result[name] = float(np.percentile(array, percentiles[name]))
    return result


def _visual_statistics(values):
    """Return the required descriptive distribution for one visual metric."""
    result = _descriptive_statistics(values, ('p05', 'p50', 'p95'))
    finite_values = [float(value) for value in values if isfinite(value)]
    result['std'] = None
    if finite_values:
        result['std'] = float(np.std(finite_values))
    return result


def _atomic_write_text(path, content):
    """Replace one text output only after its complete content is written."""
    temporary_path = path.with_name(path.name + '.tmp')
    with temporary_path.open('w', encoding='utf-8', newline='') as stream:
        stream.write(content)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary_path, path)


FAULT_STATE_NAMES = {
    FaultStatus.SCHEDULED: 'SCHEDULED',
    FaultStatus.ACTIVE: 'ACTIVE',
    FaultStatus.ENDED: 'ENDED',
    FaultStatus.CANCELLED: 'CANCELLED',
}


def _seconds_from_stamp(stamp):
    """Convert a ROS Time message to floating-point seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class FaultTruthRecorder:
    """Attach camera truth labels and persist read-only transition controls."""

    def __init__(self, session, *, initial_controls, controls_reader):
        self._session = session
        self._initial_controls = initial_controls
        self._controls_reader = controls_reader
        self._current = {
            'event_id': '',
            'truth_model': '',
            'truth_state': '',
            'severity': None,
        }
        self._last_transition = None

    @property
    def current_annotation(self):
        """Return the truth label applied to the next Image sample."""
        return dict(self._current)

    def record_status(self, message, *, received_time_sec=None):
        """Record one changed camera FaultStatus and controls snapshot."""
        if message.sensor.strip().lower() not in {'camera', 'c920'}:
            return False
        state_name = FAULT_STATE_NAMES.get(message.state)
        if state_name is None:
            return False
        signature = (str(message.event_id), int(message.state))
        if signature == self._last_transition:
            return False
        if received_time_sec is None:
            received_time_sec = time.time()
        self._last_transition = signature
        self._current = {
            'event_id': str(message.event_id),
            'truth_model': str(message.model),
            'truth_state': state_name,
            'severity': float(message.severity),
        }
        self._session.record_truth_transition({
            **self._current,
            'received_time_sec': float(received_time_sec),
            'status_stamp_sec': _seconds_from_stamp(message.header.stamp),
            'start_time_sec': _seconds_from_stamp(message.start_time),
            'end_time_sec': _seconds_from_stamp(message.end_time),
        })
        if state_name in {'SCHEDULED', 'ACTIVE'}:
            self._write_controls_once(
                'before_controls.txt', self._initial_controls
            )
        if state_name == 'ACTIVE':
            self._write_controls_once(
                'fault_controls.txt', self._controls_reader()
            )
        elif state_name == 'ENDED':
            self._write_controls_once(
                'after_controls.txt', self._controls_reader()
            )
        return True

    def _write_controls_once(self, name, content):
        path = self._session.session_directory / name
        if not path.exists():
            _atomic_write_text(path, content)


def _csv_text(samples):
    """Serialize feature samples with stable columns and empty null cells."""
    from io import StringIO

    stream = StringIO(newline='')
    writer = csv.DictWriter(stream, fieldnames=SAMPLE_FIELDS)
    writer.writeheader()
    for sample in samples:
        writer.writerow({
            field: '' if sample[field] is None else sample[field]
            for field in SAMPLE_FIELDS
        })
    return stream.getvalue()


class CameraHealthCalibrationSession:
    """Accumulate frames and persist a descriptive camera session."""

    def __init__(
        self,
        *,
        source_topic=DEFAULT_IMAGE_TOPIC,
        output_directory=DEFAULT_OUTPUT_DIRECTORY,
        video_device=DEFAULT_VIDEO_DEVICE,
        duration_sec=DEFAULT_DURATION_SEC,
        scenario_label=DEFAULT_SCENARIO_LABEL,
        session_id=DEFAULT_SESSION_ID,
        record_fault_truth=False,
        started_receive_time_sec=None,
        started_monotonic_sec=None,
    ):
        if not isfinite(duration_sec) or duration_sec <= 0.0:
            raise ValueError('duration_sec must be finite and greater than zero')
        self.source_topic = source_topic
        self.video_device = video_device
        self.duration_sec = float(duration_sec)
        self.record_fault_truth = bool(record_fault_truth)
        self.started_receive_time_sec = (
            time.time()
            if started_receive_time_sec is None
            else float(started_receive_time_sec)
        )
        self.started_monotonic_sec = (
            time.monotonic()
            if started_monotonic_sec is None
            else float(started_monotonic_sec)
        )
        self.scenario_label = _validated_scenario_label(scenario_label)
        self.session_id = _validated_session_id(
            session_id, self.started_receive_time_sec
        )
        self.baseline_root_directory = Path(output_directory).expanduser()
        self.session_directory = (
            self.baseline_root_directory / self.session_id
        )
        self.baseline_root_directory.mkdir(parents=True, exist_ok=True)
        try:
            self.session_directory.mkdir()
        except FileExistsError as error:
            raise FileExistsError(
                f'baseline session already exists: {self.session_directory}'
            ) from error
        self.samples = []
        self.conversion_error_count = 0
        self.conversion_errors = []
        self.truth_transitions = []
        self._sample_monotonic_times = []
        self._previous_image = None
        self._finalized = False
        self._summary = None
        self._output_paths = None

    @property
    def finalized(self):
        """Report whether outputs have already been safely persisted."""
        return self._finalized

    def duration_elapsed(self, now_monotonic_sec=None):
        """Report whether the configured wall-clock collection time elapsed."""
        if now_monotonic_sec is None:
            now_monotonic_sec = time.monotonic()
        return (
            float(now_monotonic_sec) - self.started_monotonic_sec
            >= self.duration_sec
        )

    def record_conversion_error(self, error):
        """Count a rejected Image without terminating baseline collection."""
        self.conversion_error_count += 1
        self.conversion_errors.append(str(error))
        self.conversion_errors = self.conversion_errors[-10:]

    def record_truth_transition(self, transition):
        """Append one changed truth state for later margin alignment."""
        if not self.record_fault_truth:
            raise RuntimeError('fault truth recording is disabled')
        self.truth_transitions.append(dict(transition))

    def add_frame(
        self,
        image,
        *,
        channel_order,
        header_stamp_sec,
        header_stamp_nanosec,
        receive_time_sec=None,
        receive_monotonic_sec=None,
        truth_annotation=None,
    ):
        """Extract existing features and append one timestamped sample."""
        if self._finalized:
            raise RuntimeError('cannot add a frame after session finalization')
        if receive_time_sec is None:
            receive_time_sec = time.time()
        if receive_monotonic_sec is None:
            receive_monotonic_sec = time.monotonic()
        receive_monotonic_sec = float(receive_monotonic_sec)

        interarrival_sec = None
        if self._sample_monotonic_times:
            interarrival_sec = (
                receive_monotonic_sec - self._sample_monotonic_times[-1]
            )
            if interarrival_sec < 0.0:
                raise ValueError('receive_monotonic_sec must not move backwards')

        features = compute_camera_health_features(
            image,
            self._previous_image,
            channel_order=channel_order,
        )
        sample = {
            'scenario_label': self.scenario_label,
            'session_id': self.session_id,
            **{
                field: (
                    truth_annotation.get(field)
                    if truth_annotation is not None else None
                )
                for field in TRUTH_FIELDS
            },
            'header_stamp_sec': int(header_stamp_sec),
            'header_stamp_nanosec': int(header_stamp_nanosec),
            'receive_time_sec': float(receive_time_sec),
            'interarrival_sec': interarrival_sec,
        }
        sample.update({name: features[name] for name in VISUAL_METRICS})
        self.samples.append(sample)
        self._sample_monotonic_times.append(receive_monotonic_sec)
        self._previous_image = np.array(image, copy=True)
        return sample

    def build_summary(
        self,
        *,
        stop_reason,
        finished_receive_time_sec=None,
        finished_monotonic_sec=None,
    ):
        """Build timing, visual, and optional truth session metadata."""
        if finished_receive_time_sec is None:
            finished_receive_time_sec = time.time()
        if finished_monotonic_sec is None:
            finished_monotonic_sec = time.monotonic()

        interarrivals = [
            sample['interarrival_sec']
            for sample in self.samples
            if sample['interarrival_sec'] is not None
        ]
        interarrival_summary = _descriptive_statistics(
            interarrivals, ('p50', 'p95', 'p99')
        )
        interarrival_summary['max_gap'] = (
            max(interarrivals) if interarrivals else None
        )

        observed_fps = None
        sample_span_sec = None
        if len(self._sample_monotonic_times) >= 2:
            sample_span_sec = (
                self._sample_monotonic_times[-1]
                - self._sample_monotonic_times[0]
            )
            if sample_span_sec > 0.0:
                observed_fps = (
                    (len(self._sample_monotonic_times) - 1)
                    / sample_span_sec
                )

        visual_summary = {
            metric: _visual_statistics([
                sample[metric]
                for sample in self.samples
                if sample[metric] is not None
            ])
            for metric in VISUAL_METRICS
        }
        return {
            'schema_version': 2,
            'description_only': True,
            'record_fault_truth': self.record_fault_truth,
            'fault_truth': {
                'recorded': self.record_fault_truth,
                'transitions': list(self.truth_transitions),
            },
            'scenario_label': self.scenario_label,
            'session_id': self.session_id,
            'source_topic': self.source_topic,
            'video_device': self.video_device,
            'baseline_root_directory': str(self.baseline_root_directory),
            'output_directory': str(self.session_directory),
            'duration_sec_requested': self.duration_sec,
            'stop_reason': stop_reason,
            'started_at_utc': _utc_now_text(self.started_receive_time_sec),
            'finished_at_utc': _utc_now_text(finished_receive_time_sec),
            'session_elapsed_sec': float(
                finished_monotonic_sec - self.started_monotonic_sec
            ),
            'sample_count': len(self.samples),
            'conversion_error_count': self.conversion_error_count,
            'recent_conversion_errors': list(self.conversion_errors),
            'observed_fps': observed_fps,
            'sample_span_sec': sample_span_sec,
            'interarrival_sec': interarrival_summary,
            'visual_metrics': visual_summary,
        }

    def finalize(
        self,
        *,
        stop_reason,
        controls_snapshot,
        finished_receive_time_sec=None,
        finished_monotonic_sec=None,
    ):
        """Write all three outputs once and return their paths."""
        if self._finalized:
            return dict(self._output_paths)

        summary = self.build_summary(
            stop_reason=stop_reason,
            finished_receive_time_sec=finished_receive_time_sec,
            finished_monotonic_sec=finished_monotonic_sec,
        )
        paths = {
            'feature_samples_csv': (
                self.session_directory / 'feature_samples.csv'
            ),
            'baseline_summary_json': (
                self.session_directory / 'baseline_summary.json'
            ),
            'controls_snapshot_txt': (
                self.session_directory / 'controls_snapshot.txt'
            ),
        }
        _atomic_write_text(paths['feature_samples_csv'], _csv_text(self.samples))
        _atomic_write_text(
            paths['baseline_summary_json'],
            json.dumps(
                summary,
                indent=2,
                sort_keys=True,
                allow_nan=False,
            ) + '\n',
        )
        _atomic_write_text(paths['controls_snapshot_txt'], controls_snapshot)
        self._summary = summary
        self._output_paths = paths
        self._finalized = True
        return dict(paths)


class CameraHealthCalibrateNode(Node):
    """Subscribe to camera images and persist a finite baseline run."""

    def __init__(self):
        super().__init__('camera_health_calibrate')
        self.declare_parameter('image_topic', DEFAULT_IMAGE_TOPIC)
        self.declare_parameter('output_directory', DEFAULT_OUTPUT_DIRECTORY)
        self.declare_parameter('video_device', DEFAULT_VIDEO_DEVICE)
        self.declare_parameter('duration_sec', DEFAULT_DURATION_SEC)
        self.declare_parameter('scenario_label', DEFAULT_SCENARIO_LABEL)
        self.declare_parameter('session_id', DEFAULT_SESSION_ID)
        self.declare_parameter('record_fault_truth', False)
        self.declare_parameter(
            'fault_status_topic', DEFAULT_FAULT_STATUS_TOPIC
        )

        image_topic = str(self.get_parameter('image_topic').value)
        output_directory = str(self.get_parameter('output_directory').value)
        video_device = str(self.get_parameter('video_device').value)
        duration_sec = float(self.get_parameter('duration_sec').value)
        scenario_label = str(self.get_parameter('scenario_label').value)
        session_id = str(self.get_parameter('session_id').value)
        record_fault_truth = bool(
            self.get_parameter('record_fault_truth').value
        )
        fault_status_topic = str(
            self.get_parameter('fault_status_topic').value
        )
        self._session = CameraHealthCalibrationSession(
            source_topic=image_topic,
            output_directory=output_directory,
            video_device=video_device,
            duration_sec=duration_sec,
            scenario_label=scenario_label,
            session_id=session_id,
            record_fault_truth=record_fault_truth,
        )
        self._bridge = CvBridge()
        self._controls_snapshot = capture_v4l2_controls(video_device)
        self._truth_recorder = None
        self._truth_subscription = None
        if record_fault_truth:
            self._truth_recorder = FaultTruthRecorder(
                self._session,
                initial_controls=self._controls_snapshot,
                controls_reader=(
                    lambda: capture_v4l2_controls(video_device)
                ),
            )
            truth_qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._truth_subscription = self.create_subscription(
                FaultStatus,
                fault_status_topic,
                self._on_fault_status,
                truth_qos,
            )
        self._subscription = self.create_subscription(
            Image,
            image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self._duration_timer = self.create_timer(0.1, self._on_duration_timer)
        self.get_logger().info(
            f'Collecting descriptive camera baseline from {image_topic} for '
            f'{duration_sec:.3f} s as scenario {scenario_label!r}, session '
            f'{self._session.session_id!r}, into '
            f'{self._session.session_directory}'
        )

    @property
    def session(self):
        """Expose the session for integration diagnostics and tests."""
        return self._session

    def _on_image(self, message):
        """Convert and record one Image while keeping conversion failures safe."""
        try:
            image, channel_order = image_message_to_numpy(message, self._bridge)
            self._session.add_frame(
                image,
                channel_order=channel_order,
                header_stamp_sec=message.header.stamp.sec,
                header_stamp_nanosec=message.header.stamp.nanosec,
                receive_time_sec=time.time(),
                receive_monotonic_sec=time.monotonic(),
                truth_annotation=(
                    self._truth_recorder.current_annotation
                    if self._truth_recorder is not None else None
                ),
            )
        except (TypeError, ValueError) as error:
            self._session.record_conversion_error(error)
            self.get_logger().warning(f'Image sample rejected: {error}')

    def _on_fault_status(self, message):
        """Record optional camera truth without affecting image features."""
        self._truth_recorder.record_status(
            message, received_time_sec=time.time()
        )

    def _on_duration_timer(self):
        """Finalize and stop the process after the configured duration."""
        if not self._session.duration_elapsed():
            return
        self.finalize('duration_elapsed')
        self._duration_timer.cancel()
        self.context.try_shutdown()

    def finalize(self, stop_reason):
        """Persist outputs idempotently and report their directory."""
        was_finalized = self._session.finalized
        try:
            paths = self._session.finalize(
                stop_reason=stop_reason,
                controls_snapshot=self._controls_snapshot,
            )
        except OSError as error:
            if self.context.ok():
                self.get_logger().error(
                    f'Failed to write baseline outputs: {error}'
                )
            raise
        if not was_finalized and self.context.ok():
            self.get_logger().info(
                f'Camera baseline persisted with {len(self._session.samples)} '
                f'samples at {paths["baseline_summary_json"].parent}'
            )
        return paths


def main(args=None):
    """Run until duration expiry or Ctrl+C, always attempting final output."""
    rclpy.init(args=args)
    node = None
    stop_reason = 'executor_stopped'
    try:
        node = CameraHealthCalibrateNode()
        rclpy.spin(node)
        if node.session.finalized:
            stop_reason = 'duration_elapsed'
    except KeyboardInterrupt:
        stop_reason = 'keyboard_interrupt'
    except ExternalShutdownException:
        stop_reason = 'external_shutdown'
    finally:
        if node is not None:
            node.finalize(stop_reason)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
