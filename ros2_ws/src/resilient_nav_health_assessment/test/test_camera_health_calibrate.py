"""Unit tests for descriptive C920 baseline collection and persistence."""

import csv
import json
from types import SimpleNamespace

from cv_bridge import CvBridgeError
import numpy as np
import pytest
import resilient_nav_health_assessment.camera_health_calibrate as calibrate
from resilient_nav_health_assessment.camera_health_calibrate import (
    CameraHealthCalibrationSession,
    capture_v4l2_controls,
    FaultTruthRecorder,
    image_message_to_numpy,
    SAMPLE_FIELDS,
    TRUTH_FIELDS,
    VISUAL_METRICS,
)
from resilient_nav_interfaces.msg import FaultStatus
from sensor_msgs.msg import Image


class FakeBridge:
    """Return a configured array or conversion exception for Image tests."""

    def __init__(self, image=None, error=None):
        self.image = image
        self.error = error
        self.desired_encodings = []

    def imgmsg_to_cv2(self, message, desired_encoding):
        self.desired_encodings.append(desired_encoding)
        if self.error is not None:
            raise self.error
        return self.image


def _session(tmp_path, **overrides):
    parameters = {
        'source_topic': '/camera/c920/image_raw',
        'output_directory': tmp_path,
        'video_device': '/dev/video0',
        'duration_sec': 1.0,
        'scenario_label': 'stationary_indoor',
        'session_id': 'session-test',
        'started_receive_time_sec': 1000.0,
        'started_monotonic_sec': 10.0,
    }
    parameters.update(overrides)
    return CameraHealthCalibrationSession(**parameters)


def _add_frame(
    session, image, index, monotonic_sec, truth_annotation=None
):
    return session.add_frame(
        image,
        channel_order='rgb',
        header_stamp_sec=100 + index,
        header_stamp_nanosec=200 + index,
        receive_time_sec=1000.0 + monotonic_sec,
        receive_monotonic_sec=monotonic_sec,
        truth_annotation=truth_annotation,
    )


def _fault_status(state, *, sensor='camera'):
    message = FaultStatus()
    message.header.stamp.sec = 100 + state
    message.event_id = 'camera-fault-1'
    message.sensor = sensor
    message.model = 'underexposure'
    message.state = state
    message.severity = 0.75
    message.start_time.sec = 102
    message.end_time.sec = 110
    return message


def test_timing_and_visual_statistics_are_descriptive(tmp_path):
    session = _session(tmp_path)
    _add_frame(session, np.zeros((8, 8), dtype=np.uint8), 0, 10.0)
    _add_frame(session, np.full((8, 8), 128, dtype=np.uint8), 1, 10.1)
    _add_frame(session, np.full((8, 8), 255, dtype=np.uint8), 2, 10.3)

    summary = session.build_summary(
        stop_reason='test_complete',
        finished_receive_time_sec=1001.0,
        finished_monotonic_sec=11.0,
    )

    timing = summary['interarrival_sec']
    assert summary['description_only'] is True
    assert summary['scenario_label'] == 'stationary_indoor'
    assert summary['session_id'] == 'session-test'
    assert summary['sample_count'] == 3
    assert summary['observed_fps'] == pytest.approx(2.0 / 0.3)
    assert timing['count'] == 2
    assert timing['mean'] == pytest.approx(0.15)
    assert timing['p50'] == pytest.approx(0.15)
    assert timing['p95'] == pytest.approx(0.195)
    assert timing['p99'] == pytest.approx(0.199)
    assert timing['max_gap'] == pytest.approx(0.2)
    assert summary['visual_metrics']['mean_gray']['count'] == 3
    assert summary['visual_metrics']['mean_gray']['mean'] == pytest.approx(
        127.6666667
    )
    assert summary['visual_metrics']['frame_diff_mean']['count'] == 2


def test_csv_json_and_controls_outputs_have_stable_content(tmp_path):
    session = _session(tmp_path)
    frame = np.full((8, 8), 64, dtype=np.uint8)
    _add_frame(session, frame, 0, 10.0)
    _add_frame(session, frame.copy(), 1, 10.2)

    paths = session.finalize(
        stop_reason='duration_elapsed',
        controls_snapshot='read_only controls\n',
        finished_receive_time_sec=1001.0,
        finished_monotonic_sec=11.0,
    )

    assert set(paths) == {
        'feature_samples_csv',
        'baseline_summary_json',
        'controls_snapshot_txt',
    }
    with paths['feature_samples_csv'].open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    assert tuple(rows[0]) == SAMPLE_FIELDS
    assert len(rows) == 2
    assert rows[0]['scenario_label'] == 'stationary_indoor'
    assert rows[0]['session_id'] == 'session-test'
    assert all(rows[0][field] == '' for field in TRUTH_FIELDS)
    assert rows[0]['header_stamp_sec'] == '100'
    assert rows[0]['interarrival_sec'] == ''
    assert rows[0]['frame_diff_mean'] == ''
    assert float(rows[1]['interarrival_sec']) == pytest.approx(0.2)
    assert float(rows[1]['frame_diff_mean']) == 0.0

    summary_text = paths['baseline_summary_json'].read_text(encoding='utf-8')
    summary = json.loads(summary_text)
    assert summary['stop_reason'] == 'duration_elapsed'
    assert summary['scenario_label'] == 'stationary_indoor'
    assert summary['session_id'] == 'session-test'
    assert summary['baseline_root_directory'] == str(tmp_path)
    assert summary['output_directory'] == str(tmp_path / 'session-test')
    assert set(summary['visual_metrics']) == set(VISUAL_METRICS)
    assert 'HEALTHY' not in summary_text
    assert 'FAULT' not in summary_text
    assert paths['controls_snapshot_txt'].read_text() == 'read_only controls\n'
    assert paths['baseline_summary_json'].parent == tmp_path / 'session-test'


def test_empty_session_writes_header_and_null_descriptive_statistics(tmp_path):
    session = _session(tmp_path)

    paths = session.finalize(
        stop_reason='duration_elapsed',
        controls_snapshot='no device data\n',
        finished_receive_time_sec=1001.0,
        finished_monotonic_sec=11.0,
    )

    with paths['feature_samples_csv'].open(newline='', encoding='utf-8') as stream:
        assert list(csv.DictReader(stream)) == []
    summary = json.loads(paths['baseline_summary_json'].read_text())
    assert summary['sample_count'] == 0
    assert summary['observed_fps'] is None
    assert summary['interarrival_sec']['count'] == 0
    assert summary['interarrival_sec']['max_gap'] is None
    for metric in VISUAL_METRICS:
        assert summary['visual_metrics'][metric]['count'] == 0
        assert summary['visual_metrics'][metric]['mean'] is None
        assert summary['visual_metrics'][metric]['std'] is None


def test_unsupported_image_encoding_is_rejected_before_cv_bridge():
    message = Image()
    message.encoding = '32FC1'
    bridge = FakeBridge(image=np.zeros((2, 2), dtype=np.float32))

    with pytest.raises(ValueError, match='unsupported Image encoding'):
        image_message_to_numpy(message, bridge)

    assert bridge.desired_encodings == []


def test_cv_bridge_error_is_reported_as_a_clear_value_error():
    message = Image()
    message.encoding = 'rgb8'
    bridge = FakeBridge(error=CvBridgeError('bad step'))

    with pytest.raises(ValueError, match='cv_bridge failed.*bad step'):
        image_message_to_numpy(message, bridge)


@pytest.mark.parametrize(
    'encoding,expected_order',
    [('mono8', 'rgb'), ('rgb8', 'rgb'), ('bgr8', 'bgr'), ('bgra8', 'bgr')],
)
def test_supported_image_encoding_preserves_channel_order(
    encoding, expected_order,
):
    message = Image()
    message.encoding = encoding
    image = np.zeros((2, 3), dtype=np.uint8)
    bridge = FakeBridge(image=image)

    converted, channel_order = image_message_to_numpy(message, bridge)

    assert converted is image
    assert channel_order == expected_order
    assert bridge.desired_encodings == ['passthrough']


def test_session_strictly_reuses_existing_feature_function(tmp_path, monkeypatch):
    calls = []

    def fake_features(image, previous_image, *, channel_order):
        calls.append((image, previous_image, channel_order))
        return {
            metric: None if metric == 'frame_diff_mean' else 1.0
            for metric in VISUAL_METRICS
        }

    monkeypatch.setattr(calibrate, 'compute_camera_health_features', fake_features)
    session = _session(tmp_path)
    image = np.zeros((4, 4), dtype=np.uint8)

    sample = _add_frame(session, image, 0, 10.0)

    assert len(calls) == 1
    assert calls[0][0] is image
    assert calls[0][1] is None
    assert calls[0][2] == 'rgb'
    assert sample['mean_gray'] == 1.0


def test_truth_annotation_does_not_enter_feature_function(tmp_path, monkeypatch):
    calls = []

    def fake_features(image, previous_image, *, channel_order):
        calls.append((image, previous_image, channel_order))
        return {
            metric: None if metric == 'frame_diff_mean' else 2.0
            for metric in VISUAL_METRICS
        }

    monkeypatch.setattr(calibrate, 'compute_camera_health_features', fake_features)
    session = _session(tmp_path, record_fault_truth=True)
    annotation = {
        'event_id': 'event-1',
        'truth_model': 'blur',
        'truth_state': 'ACTIVE',
        'severity': 0.5,
    }

    sample = _add_frame(
        session,
        np.zeros((4, 4), dtype=np.uint8),
        0,
        10.0,
        annotation,
    )

    assert len(calls) == 1
    assert calls[0][2] == 'rgb'
    assert sample['truth_model'] == 'blur'
    assert sample['truth_state'] == 'ACTIVE'
    assert sample['mean_gray'] == 2.0


def test_fault_truth_transitions_label_samples_and_snapshot_controls(tmp_path):
    session = _session(tmp_path, record_fault_truth=True)
    control_reads = iter(['fault controls\n', 'after controls\n'])
    recorder = FaultTruthRecorder(
        session,
        initial_controls='before controls\n',
        controls_reader=lambda: next(control_reads),
    )

    assert recorder.record_status(
        _fault_status(FaultStatus.SCHEDULED), received_time_sec=1001.0
    )
    scheduled = _add_frame(
        session,
        np.zeros((4, 4), dtype=np.uint8),
        0,
        10.0,
        recorder.current_annotation,
    )
    assert recorder.record_status(
        _fault_status(FaultStatus.ACTIVE), received_time_sec=1002.0
    )
    active = _add_frame(
        session,
        np.ones((4, 4), dtype=np.uint8),
        1,
        10.1,
        recorder.current_annotation,
    )
    assert recorder.record_status(
        _fault_status(FaultStatus.ACTIVE), received_time_sec=1002.1
    ) is False
    assert recorder.record_status(
        _fault_status(FaultStatus.ENDED), received_time_sec=1010.0
    )
    ended = _add_frame(
        session,
        np.full((4, 4), 2, dtype=np.uint8),
        2,
        10.2,
        recorder.current_annotation,
    )

    assert scheduled['truth_state'] == 'SCHEDULED'
    assert active['truth_state'] == 'ACTIVE'
    assert active['event_id'] == 'camera-fault-1'
    assert active['severity'] == pytest.approx(0.75)
    assert ended['truth_state'] == 'ENDED'
    assert len(session.truth_transitions) == 3
    assert (session.session_directory / 'before_controls.txt').read_text() == (
        'before controls\n'
    )
    assert (session.session_directory / 'fault_controls.txt').read_text() == (
        'fault controls\n'
    )
    assert (session.session_directory / 'after_controls.txt').read_text() == (
        'after controls\n'
    )

    paths = session.finalize(
        stop_reason='test', controls_snapshot='startup controls\n'
    )
    summary = json.loads(paths['baseline_summary_json'].read_text())
    assert summary['record_fault_truth'] is True
    assert len(summary['fault_truth']['transitions']) == 3


def test_fault_truth_recorder_ignores_non_camera_status(tmp_path):
    session = _session(tmp_path, record_fault_truth=True)
    recorder = FaultTruthRecorder(
        session,
        initial_controls='before\n',
        controls_reader=lambda: 'unexpected\n',
    )

    assert recorder.record_status(
        _fault_status(FaultStatus.ACTIVE, sensor='imu'),
        received_time_sec=1002.0,
    ) is False
    assert recorder.current_annotation['truth_state'] == ''
    assert session.truth_transitions == []
    assert not (session.session_directory / 'before_controls.txt').exists()


def test_v4l2_snapshot_uses_only_the_read_command():
    calls = []

    def fake_runner(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(returncode=0, stdout='brightness: 128\n', stderr='')

    snapshot = capture_v4l2_controls(
        '/dev/video9',
        runner=fake_runner,
        captured_at_epoch_sec=0.0,
    )

    assert calls[0][0] == [
        'v4l2-ctl',
        '--device',
        '/dev/video9',
        '--list-ctrls-menus',
    ]
    assert all('--set' not in argument for argument in calls[0][0])
    assert calls[0][1]['check'] is False
    assert 'read_only: true' in snapshot
    assert 'brightness: 128' in snapshot


def test_v4l2_snapshot_records_command_failure_without_aborting():
    def failing_runner(command, **kwargs):
        raise FileNotFoundError('v4l2-ctl missing')

    snapshot = capture_v4l2_controls('/dev/video0', runner=failing_runner)

    assert 'returncode: unavailable' in snapshot
    assert 'v4l2-ctl missing' in snapshot


def test_finalize_is_idempotent_and_rejects_late_frames(tmp_path):
    session = _session(tmp_path)
    _add_frame(session, np.zeros((4, 4), dtype=np.uint8), 0, 10.0)
    first_paths = session.finalize(
        stop_reason='keyboard_interrupt',
        controls_snapshot='first snapshot\n',
        finished_receive_time_sec=1000.5,
        finished_monotonic_sec=10.5,
    )
    first_json = first_paths['baseline_summary_json'].read_text()

    second_paths = session.finalize(
        stop_reason='external_shutdown',
        controls_snapshot='second snapshot\n',
        finished_receive_time_sec=1001.0,
        finished_monotonic_sec=11.0,
    )

    assert session.finalized is True
    assert second_paths == first_paths
    assert first_paths['baseline_summary_json'].read_text() == first_json
    assert json.loads(first_json)['stop_reason'] == 'keyboard_interrupt'
    assert first_paths['controls_snapshot_txt'].read_text() == 'first snapshot\n'
    with pytest.raises(RuntimeError, match='after session finalization'):
        _add_frame(session, np.zeros((4, 4), dtype=np.uint8), 1, 10.6)


def test_duration_boundary_and_invalid_duration_are_explicit(tmp_path):
    session = _session(tmp_path, duration_sec=2.0)

    assert session.duration_elapsed(11.999) is False
    assert session.duration_elapsed(12.0) is True
    with pytest.raises(ValueError, match='greater than zero'):
        _session(tmp_path, duration_sec=0.0)


def test_backwards_receive_time_is_rejected(tmp_path):
    session = _session(tmp_path)
    _add_frame(session, np.zeros((4, 4), dtype=np.uint8), 0, 10.2)

    with pytest.raises(ValueError, match='must not move backwards'):
        _add_frame(session, np.zeros((4, 4), dtype=np.uint8), 1, 10.1)


def test_independent_session_directories_never_overwrite(tmp_path):
    first = _session(tmp_path, session_id='session-a')
    second = _session(tmp_path, session_id='session-b')

    first_paths = first.finalize(
        stop_reason='test', controls_snapshot='first\n'
    )
    second_paths = second.finalize(
        stop_reason='test', controls_snapshot='second\n'
    )

    assert first_paths['baseline_summary_json'].parent == tmp_path / 'session-a'
    assert second_paths['baseline_summary_json'].parent == tmp_path / 'session-b'
    assert first_paths['controls_snapshot_txt'].read_text() == 'first\n'
    assert second_paths['controls_snapshot_txt'].read_text() == 'second\n'
    with pytest.raises(FileExistsError, match='already exists'):
        _session(tmp_path, session_id='session-a')


def test_empty_session_id_is_generated_and_path_ids_are_rejected(tmp_path):
    generated = _session(tmp_path, session_id='')

    assert generated.session_id == '19700101T001640.000000Z'
    assert generated.session_directory == tmp_path / generated.session_id
    with pytest.raises(ValueError, match='one directory name'):
        _session(tmp_path, session_id='../escape')
    with pytest.raises(ValueError, match='scenario_label must not be empty'):
        _session(tmp_path, scenario_label='  ', session_id='unused')
