"""Unit tests for camera stale/freeze state and descriptive metrics."""

import numpy as np
import pytest
import resilient_nav_health_assessment.camera_health_monitor as monitor_module
from resilient_nav_health_assessment.camera_health_monitor import (
    CameraHealthEvaluator,
)
from resilient_nav_interfaces.msg import SensorHealth


def _evaluator(**overrides):
    parameters = {
        'window_duration_sec': 3.0,
        'min_samples': 3,
        'stale_timeout_sec': 1.0,
        'freeze_duration_sec': 1.0,
        'fault_confirmation_sec': 0.5,
        'recovery_confirmation_sec': 0.8,
    }
    parameters.update(overrides)
    evaluator = CameraHealthEvaluator(**parameters)
    evaluator.start(0.0)
    return evaluator


def _image(value, changing_pixel=None):
    image = np.full((8, 8), value, dtype=np.uint8)
    if changing_pixel is not None:
        image[0, 0] = changing_pixel
    return image


def _textured_image(offset=0):
    rows, columns = np.indices((8, 8))
    return ((rows * 17 + columns * 31 + offset) % 180 + 30).astype(
        np.uint8
    )


def _add(evaluator, received_sec, image, stamp_sec=None):
    if stamp_sec is None:
        stamp_sec = 100.0 + received_sec
    evaluator.add_image(
        image,
        channel_order='rgb',
        received_sec=received_sec,
        stamp_sec=stamp_sec,
    )


def _metrics(decision):
    return dict(zip(decision.metric_names, decision.metric_values))


def _feature_result(fingerprint, **overrides):
    result = {
        'mean_gray': 60.0,
        'gray_std': 25.0,
        'p05': 6.0,
        'p95': 85.0,
        'dark_ratio': 0.19,
        'bright_ratio': 0.0,
        'laplacian_variance': 280.0,
        'edge_density': 0.02,
        'entropy': 5.7,
        'frame_diff_mean': 1.0,
        'frame_fingerprint': fingerprint,
    }
    result.update(overrides)
    return result


def _install_feature_sequence(monkeypatch, feature_rows):
    rows = iter(feature_rows)

    def compute_features(*args, **kwargs):
        return next(rows)

    monkeypatch.setattr(
        monitor_module, 'compute_camera_health_features', compute_features
    )


def _seed_changing_stream(evaluator):
    _add(evaluator, 0.0, _textured_image(0))
    _add(evaluator, 0.1, _textured_image(1))
    _add(evaluator, 0.2, _textured_image(2))


def test_normal_message_stream_is_healthy_with_all_required_metrics():
    evaluator = _evaluator()
    _seed_changing_stream(evaluator)

    decision = evaluator.evaluate(0.2)
    metrics = _metrics(decision)

    assert decision.state == SensorHealth.HEALTHY
    assert decision.detected_fault == 'none'
    assert decision.health_score == pytest.approx(1.0)
    assert decision.confidence == pytest.approx(1.0)
    for name in [
        'message_age_sec',
        'latest_interarrival_sec',
        'rolling_observed_fps',
        'max_recent_gap_sec',
        'header_stamp_progress',
        'header_stamp_progress_ratio',
        'fingerprint_changed',
        'fingerprint_identical_duration_sec',
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
    ]:
        assert name in metrics
    assert metrics['rolling_observed_fps'] == pytest.approx(10.0)
    assert metrics['max_recent_gap_sec'] == pytest.approx(0.1)


def test_stale_requires_time_confirmation_before_fault():
    evaluator = _evaluator()
    _seed_changing_stream(evaluator)

    pending = evaluator.evaluate(1.21)
    still_pending = evaluator.evaluate(1.60)
    fault = evaluator.evaluate(1.72)

    assert pending.state == SensorHealth.DEGRADED
    assert pending.detected_fault == 'stale'
    assert pending.reasons == ['stale_confirmation_pending']
    assert still_pending.state == SensorHealth.DEGRADED
    assert fault.state == SensorHealth.FAULT
    assert fault.detected_fault == 'stale'
    assert fault.health_score == 0.0
    assert fault.confidence == 1.0


def test_stamp_progress_with_exact_fingerprint_repetition_detects_freeze():
    evaluator = _evaluator()
    frame = _image(80)
    pending = None
    for index in range(9):
        received_sec = index * 0.2
        _add(evaluator, received_sec, frame.copy())
        pending = evaluator.evaluate(received_sec)

    assert pending.state == SensorHealth.FAULT
    assert pending.detected_fault == 'freeze'
    metrics = _metrics(pending)
    assert metrics['message_age_sec'] == 0.0
    assert metrics['header_stamp_progress'] == 1.0
    assert metrics['fingerprint_stamp_progress_continuous'] == 1.0
    assert metrics['fingerprint_identical_duration_sec'] >= 1.0
    assert pending.health_score == pytest.approx(0.1)


def test_stale_and_freeze_are_distinct_when_repeated_stream_stops():
    evaluator = _evaluator()
    frame = _image(80)
    for index in range(6):
        _add(evaluator, index * 0.2, frame.copy())
    freeze_pending = evaluator.evaluate(1.0)
    stale_pending = evaluator.evaluate(2.01)
    stale_fault = evaluator.evaluate(2.52)

    assert freeze_pending.detected_fault == 'freeze'
    assert freeze_pending.state == SensorHealth.DEGRADED
    assert stale_pending.detected_fault == 'stale'
    assert stale_pending.state == SensorHealth.DEGRADED
    assert stale_fault.detected_fault == 'stale'
    assert stale_fault.state == SensorHealth.FAULT


def test_equal_fingerprint_without_stamp_progress_is_not_freeze():
    evaluator = _evaluator()
    frame = _image(80)
    for index in range(16):
        received_sec = index * 0.2
        _add(evaluator, received_sec, frame.copy(), stamp_sec=100.0)

    decision = evaluator.evaluate(3.0)
    metrics = _metrics(decision)

    assert decision.state == SensorHealth.HEALTHY
    assert decision.detected_fault == 'none'
    assert metrics['fingerprint_identical_duration_sec'] >= 1.0
    assert metrics['fingerprint_stamp_progress_continuous'] == 0.0
    assert metrics['header_stamp_progress'] == 0.0


def test_static_scene_with_small_pixel_changes_is_not_freeze():
    evaluator = _evaluator()
    decisions = []
    for index in range(25):
        received_sec = index * 0.125
        _add(
            evaluator,
            received_sec,
            _image(80, changing_pixel=80 + index),
        )
        decisions.append(evaluator.evaluate(received_sec))

    assert all(
        decision.state != SensorHealth.FAULT for decision in decisions
    )
    assert decisions[-1].state == SensorHealth.HEALTHY
    assert _metrics(decisions[-1])['fingerprint_changed'] == 1.0


def test_fault_recovery_requires_continuous_stable_duration():
    evaluator = _evaluator()
    frozen = _image(80)
    for index in range(9):
        received_sec = index * 0.2
        _add(evaluator, received_sec, frozen.copy())
        fault = evaluator.evaluate(received_sec)
    assert fault.state == SensorHealth.FAULT
    assert fault.detected_fault == 'freeze'

    _add(evaluator, 1.8, _image(81))
    recovery_started = evaluator.evaluate(1.8)
    _add(evaluator, 2.2, _image(82))
    recovery_pending = evaluator.evaluate(2.2)
    _add(evaluator, 2.61, _image(83))
    recovered = evaluator.evaluate(2.61)

    assert recovery_started.state == SensorHealth.FAULT
    assert recovery_started.reasons == ['freeze_recovery_pending']
    assert recovery_pending.state == SensorHealth.FAULT
    assert recovery_pending.health_score == pytest.approx(0.4)
    assert recovered.state == SensorHealth.HEALTHY
    assert recovered.detected_fault == 'none'


def test_freeze_recovery_timer_resets_when_content_stops_progressing():
    evaluator = _evaluator()
    frozen = _image(80)
    for index in range(9):
        received_sec = index * 0.2
        _add(evaluator, received_sec, frozen.copy())
        fault = evaluator.evaluate(received_sec)
    assert fault.state == SensorHealth.FAULT

    _add(evaluator, 1.8, _image(81))
    assert evaluator.evaluate(1.8).state == SensorHealth.FAULT
    _add(evaluator, 2.2, _image(81))
    interrupted = evaluator.evaluate(2.2)
    _add(evaluator, 2.3, _image(82))
    restarted = evaluator.evaluate(2.3)
    _add(evaluator, 3.11, _image(83))
    recovered = evaluator.evaluate(3.11)

    assert interrupted.reasons == ['freeze_recovery_pending']
    assert restarted.state == SensorHealth.FAULT
    assert recovered.state == SensorHealth.HEALTHY


def test_observed_8_to_15_hz_variation_and_baseline_gap_do_not_fault():
    evaluator = _evaluator()
    received_sec = 0.0
    decisions = []
    intervals = [1.0 / 15.0, 1.0 / 8.0, 0.38, 0.09, 0.12] * 4
    for index, interval in enumerate(intervals):
        received_sec += interval
        _add(evaluator, received_sec, _image(40 + index))
        decisions.append(evaluator.evaluate(received_sec))

    assert all(
        decision.state != SensorHealth.FAULT for decision in decisions
    )
    assert decisions[-1].state == SensorHealth.HEALTHY
    metrics = _metrics(decisions[-1])
    assert metrics['max_recent_gap_sec'] == pytest.approx(0.38)
    assert metrics['rolling_observed_fps'] > 0.0


def test_blank_stream_without_informative_reference_is_not_low_information():
    evaluator = _evaluator(freeze_duration_sec=10.0)
    for index in range(8):
        received_sec = index * 0.15
        _add(evaluator, received_sec, _image(80))

    decision = evaluator.evaluate(1.05)
    metrics = _metrics(decision)

    assert decision.state == SensorHealth.HEALTHY
    assert decision.detected_fault == 'none'
    assert decision.health_score == 1.0
    assert decision.confidence == 1.0
    assert metrics['exposure_dark_candidate'] == 0.0
    assert metrics['exposure_bright_candidate'] == 0.0
    assert metrics['blur_candidate'] == 0.0
    assert metrics['low_information_candidate'] == 0.0
    assert metrics['low_information_reference_ready'] == 0.0


def test_underexposure_dev_features_confirm_fault_and_recover(monkeypatch):
    evaluator = _evaluator(freeze_duration_sec=0.4)
    underexposed = [
        _feature_result(
            'dev-black-frame',
            mean_gray=4.035,
            p95=5.506,
            dark_ratio=0.946,
        )
        for index in range(8)
    ]
    recovered = [
        _feature_result(
            f'recovered-{index}',
            mean_gray=61.016,
            p95=85.815,
            dark_ratio=0.188,
        )
        for index in range(3)
    ]
    _install_feature_sequence(monkeypatch, underexposed + recovered)

    decisions = []
    for index in range(8):
        received_sec = index * 0.2
        _add(evaluator, received_sec, _image(index))
        decisions.append(evaluator.evaluate(received_sec))

    assert decisions[0].state == SensorHealth.DEGRADED
    assert decisions[0].detected_fault == 'underexposed'
    assert decisions[2].state == SensorHealth.DEGRADED
    assert decisions[-1].state == SensorHealth.FAULT
    assert decisions[-1].detected_fault == 'underexposed'
    fault_metrics = _metrics(decisions[-1])
    assert fault_metrics['exposure_dark_candidate'] == 1.0
    assert fault_metrics['fingerprint_identical_duration_sec'] >= 0.4

    recovery_decisions = []
    for received_sec in (1.6, 2.0, 2.5):
        _add(evaluator, received_sec, _image(int(received_sec * 10)))
        recovery_decisions.append(evaluator.evaluate(received_sec))

    assert recovery_decisions[0].state == SensorHealth.FAULT
    assert recovery_decisions[0].reasons == [
        'underexposed_recovery_pending'
    ]
    assert recovery_decisions[1].state == SensorHealth.FAULT
    assert recovery_decisions[-1].state == SensorHealth.HEALTHY
    assert recovery_decisions[-1].detected_fault == 'none'


def test_overexposure_dev_features_confirm_fault_and_recover(monkeypatch):
    evaluator = _evaluator(freeze_duration_sec=0.4)
    overexposed = [
        _feature_result(
            'dev-bright-frame',
            mean_gray=183.974,
            p05=175.070,
            p95=188.091,
            dark_ratio=0.034,
            bright_ratio=0.0,
        )
        for index in range(8)
    ]
    recovered = [
        _feature_result(
            f'over-recovered-{index}',
            mean_gray=62.165,
            p05=9.479,
            p95=87.002,
            dark_ratio=0.202,
            bright_ratio=0.0,
        )
        for index in range(3)
    ]
    _install_feature_sequence(monkeypatch, overexposed + recovered)

    decisions = []
    for index in range(8):
        received_sec = index * 0.2
        _add(evaluator, received_sec, _image(index))
        decisions.append(evaluator.evaluate(received_sec))

    assert decisions[0].state == SensorHealth.DEGRADED
    assert decisions[0].detected_fault == 'overexposed'
    assert decisions[2].state == SensorHealth.DEGRADED
    assert decisions[-1].state == SensorHealth.FAULT
    assert decisions[-1].detected_fault == 'overexposed'
    fault_metrics = _metrics(decisions[-1])
    assert fault_metrics['exposure_bright_candidate'] == 1.0
    assert fault_metrics['bright_ratio_observation_candidate'] == 0.0
    assert fault_metrics['fingerprint_identical_duration_sec'] >= 0.4

    recovery_decisions = []
    for received_sec in (1.6, 2.0, 2.5):
        _add(evaluator, received_sec, _image(int(received_sec * 10)))
        recovery_decisions.append(evaluator.evaluate(received_sec))

    assert recovery_decisions[0].state == SensorHealth.FAULT
    assert recovery_decisions[0].reasons == [
        'overexposed_recovery_pending'
    ]
    assert recovery_decisions[1].state == SensorHealth.FAULT
    assert recovery_decisions[-1].state == SensorHealth.HEALTHY
    assert recovery_decisions[-1].detected_fault == 'none'


def test_healthy_bright_baseline_does_not_confirm_overexposure(monkeypatch):
    evaluator = _evaluator(freeze_duration_sec=10.0)
    healthy_bright = [
        _feature_result(
            f'healthy-bright-{index}',
            mean_gray=163.255,
            p05=64.620,
            p95=247.956,
            bright_ratio=0.557,
        )
        for index in range(12)
    ]
    _install_feature_sequence(monkeypatch, healthy_bright)

    decisions = []
    for index in range(12):
        received_sec = index * 0.125
        _add(evaluator, received_sec, _image(index))
        decisions.append(evaluator.evaluate(received_sec))

    assert all(decision.state != SensorHealth.FAULT for decision in decisions)
    assert decisions[-1].state == SensorHealth.HEALTHY
    assert decisions[-1].detected_fault == 'none'
    metrics = _metrics(decisions[-1])
    assert metrics['exposure_bright_candidate'] == 0.0
    assert metrics['overexposure_candidate_duration_sec'] == 0.0


def test_blur_dev_features_confirm_fault_and_recover(monkeypatch):
    evaluator = _evaluator(freeze_duration_sec=0.4)
    healthy_pre = [
        _feature_result(
            f'blur-pre-{index}',
            laplacian_variance=309.680,
            edge_density=0.028,
            entropy=5.798,
            gray_std=26.752,
        )
        for index in range(3)
    ]
    blurred = [
        _feature_result(
            'dev-blurred-frame',
            laplacian_variance=6.853,
            edge_density=0.0,
            entropy=5.776,
            gray_std=24.377,
        )
        for index in range(8)
    ]
    recovered = [
        _feature_result(
            f'blur-recovered-{index}',
            laplacian_variance=263.716,
            edge_density=0.024,
            entropy=5.755,
            gray_std=26.0,
        )
        for index in range(3)
    ]
    _install_feature_sequence(
        monkeypatch, healthy_pre + blurred + recovered
    )

    pre_decisions = []
    for index in range(3):
        received_sec = index * 0.2
        _add(evaluator, received_sec, _image(index))
        pre_decisions.append(evaluator.evaluate(received_sec))
    assert pre_decisions[0].state == SensorHealth.UNKNOWN
    assert pre_decisions[1].state == SensorHealth.UNKNOWN
    assert pre_decisions[2].state == SensorHealth.HEALTHY

    decisions = []
    for index in range(8):
        received_sec = 0.6 + index * 0.2
        _add(evaluator, received_sec, _image(10 + index))
        decisions.append(evaluator.evaluate(received_sec))

    assert decisions[0].state == SensorHealth.DEGRADED
    assert decisions[0].detected_fault == 'blurred'
    assert decisions[2].state == SensorHealth.DEGRADED
    assert decisions[-1].state == SensorHealth.FAULT
    assert decisions[-1].detected_fault == 'blurred'
    fault_metrics = _metrics(decisions[-1])
    assert fault_metrics['blur_candidate'] == 1.0
    assert fault_metrics['blur_reference_ready'] == 1.0
    assert fault_metrics['blur_candidate_duration_sec'] >= 0.6
    assert fault_metrics['fingerprint_identical_duration_sec'] >= 0.4

    recovery_decisions = []
    for received_sec in (2.2, 2.7, 3.21):
        _add(evaluator, received_sec, _image(int(received_sec * 10)))
        recovery_decisions.append(evaluator.evaluate(received_sec))

    assert recovery_decisions[0].state == SensorHealth.FAULT
    assert recovery_decisions[0].reasons == ['blurred_recovery_pending']
    assert recovery_decisions[1].state == SensorHealth.FAULT
    assert recovery_decisions[-1].state == SensorHealth.HEALTHY
    assert recovery_decisions[-1].detected_fault == 'none'


def test_occlusion_dev_features_confirm_low_information_and_recover(
    monkeypatch,
):
    evaluator = _evaluator(freeze_duration_sec=0.4)
    healthy_pre = [
        _feature_result(
            f'occlusion-pre-{index}',
            edge_density=0.028,
            entropy=5.533,
            gray_std=27.915,
            dark_ratio=0.190,
        )
        for index in range(3)
    ]
    low_information = [
        _feature_result(
            'occlusion-active-frame',
            laplacian_variance=2.0,
            edge_density=0.0,
            entropy=4.697,
            gray_std=58.869,
            dark_ratio=0.444,
        )
        for index in range(8)
    ]
    recovered = [
        _feature_result(
            f'occlusion-recovered-{index}',
            edge_density=0.021,
            entropy=5.271,
            gray_std=33.564,
            dark_ratio=0.114,
        )
        for index in range(3)
    ]
    _install_feature_sequence(
        monkeypatch, healthy_pre + low_information + recovered
    )

    for index in range(3):
        received_sec = index * 0.2
        _add(evaluator, received_sec, _image(index))
        healthy = evaluator.evaluate(received_sec)
    assert healthy.state == SensorHealth.HEALTHY

    decisions = []
    for index in range(8):
        received_sec = 0.6 + index * 0.2
        _add(evaluator, received_sec, _image(10 + index))
        decisions.append(evaluator.evaluate(received_sec))

    assert decisions[0].state == SensorHealth.DEGRADED
    assert decisions[0].detected_fault == 'low_information'
    assert decisions[2].state == SensorHealth.DEGRADED
    assert decisions[-1].state == SensorHealth.FAULT
    assert decisions[-1].detected_fault == 'low_information'
    fault_metrics = _metrics(decisions[-1])
    assert fault_metrics['low_information_candidate'] == 1.0
    assert fault_metrics['low_information_reference_ready'] == 1.0
    assert fault_metrics['low_information_candidate_duration_sec'] >= 0.6

    recovery_decisions = []
    for received_sec in (2.2, 2.7, 3.21):
        _add(evaluator, received_sec, _image(int(received_sec * 10)))
        recovery_decisions.append(evaluator.evaluate(received_sec))

    assert recovery_decisions[0].state == SensorHealth.FAULT
    assert recovery_decisions[0].reasons == [
        'low_information_recovery_pending'
    ]
    assert recovery_decisions[1].state == SensorHealth.FAULT
    assert recovery_decisions[-1].state == SensorHealth.HEALTHY
    assert recovery_decisions[-1].detected_fault == 'none'


def test_low_information_fault_stays_latched_after_reference_expiry(
    monkeypatch,
):
    evaluator = _evaluator(freeze_duration_sec=10.0)
    healthy_pre = [
        _feature_result(
            f'long-occlusion-pre-{index}',
            edge_density=0.028,
            entropy=5.533,
            gray_std=27.915,
            dark_ratio=0.190,
        )
        for index in range(3)
    ]
    sustained_occlusion = [
        _feature_result(
            f'long-occlusion-active-{index}',
            laplacian_variance=2.0,
            edge_density=0.0,
            entropy=4.697,
            gray_std=58.869,
            dark_ratio=0.444,
        )
        for index in range(151)
    ]
    _install_feature_sequence(monkeypatch, healthy_pre + sustained_occlusion)

    for index in range(3):
        received_sec = index * 0.2
        _add(evaluator, received_sec, _image(index))
        evaluator.evaluate(received_sec)

    decisions = []
    for index in range(151):
        received_sec = 0.6 + index * 0.2
        _add(evaluator, received_sec, _image(10 + index))
        decisions.append(evaluator.evaluate(received_sec))

    after_reference_expiry = [
        decision
        for index, decision in enumerate(decisions)
        if 0.6 + index * 0.2 > 3.4
    ]
    assert after_reference_expiry
    assert all(
        decision.state == SensorHealth.FAULT
        and decision.detected_fault == 'low_information'
        for decision in after_reference_expiry
    )
    metrics = _metrics(decisions[-1])
    assert metrics['low_information_reference_ready'] == 1.0
    assert metrics['low_information_candidate_duration_sec'] >= 2.0


def test_stationary_bright_low_texture_never_becomes_blurred(monkeypatch):
    evaluator = _evaluator(freeze_duration_sec=10.0)
    low_texture = [
        _feature_result(
            f'low-texture-{index}',
            laplacian_variance=(169.552 if index == 0 else 3.386),
            edge_density=(0.00753 if index == 0 else 0.0),
            entropy=(5.988 if index == 0 else 5.335),
            gray_std=(70.509 if index == 0 else 23.179),
        )
        for index in range(14)
    ]
    _install_feature_sequence(monkeypatch, low_texture)

    decisions = []
    for index in range(14):
        received_sec = index * 0.125
        _add(evaluator, received_sec, _image(index))
        decisions.append(evaluator.evaluate(received_sec))

    assert all(decision.state != SensorHealth.FAULT for decision in decisions)
    assert decisions[-1].state == SensorHealth.HEALTHY
    metrics = _metrics(decisions[-1])
    assert metrics['blur_reference_ready'] == 0.0
    assert metrics['blur_candidate'] == 0.0
    assert metrics['low_information_reference_ready'] == 0.0
    assert metrics['low_information_candidate'] == 0.0


def test_other_healthy_baselines_do_not_confirm_blur(monkeypatch):
    evaluator = _evaluator(freeze_duration_sec=10.0)
    stationary_indoor = [
        _feature_result(
            f'indoor-{index}',
            laplacian_variance=3.495,
            edge_density=0.0,
            entropy=6.156,
            gray_std=20.676,
        )
        for index in range(8)
    ]
    textured_then_isolated_low = [
        _feature_result(
            'textured-reference',
            laplacian_variance=259.425,
            edge_density=0.0565,
            entropy=7.0,
            gray_std=45.0,
        ),
        _feature_result(
            'isolated-low-1',
            laplacian_variance=5.0,
            edge_density=0.0,
            entropy=5.8,
            gray_std=24.0,
        ),
        _feature_result(
            'isolated-low-2',
            laplacian_variance=5.0,
            edge_density=0.0,
            entropy=5.8,
            gray_std=24.0,
        ),
        _feature_result(
            'normal-detail',
            laplacian_variance=30.0,
            edge_density=0.003,
            entropy=6.0,
            gray_std=30.0,
        ),
    ]
    _install_feature_sequence(
        monkeypatch, stationary_indoor + textured_then_isolated_low
    )

    decisions = []
    for index in range(12):
        received_sec = index * 0.125
        _add(evaluator, received_sec, _image(index))
        decisions.append(evaluator.evaluate(received_sec))

    assert all(decision.state != SensorHealth.FAULT for decision in decisions)
    assert decisions[-1].state == SensorHealth.HEALTHY
    assert decisions[-1].detected_fault == 'none'
    assert _metrics(decisions[-1])['low_information_candidate'] == 0.0


def test_one_underexposed_image_cannot_complete_confirmation(monkeypatch):
    evaluator = _evaluator(freeze_duration_sec=10.0)
    _install_feature_sequence(monkeypatch, [
        _feature_result(
            'one-dark-frame', mean_gray=0.0, p95=0.0, dark_ratio=1.0
        )
    ])
    _add(evaluator, 0.0, _image(0))

    first = evaluator.evaluate(0.0)
    later = evaluator.evaluate(0.7)

    assert first.state == SensorHealth.DEGRADED
    assert later.state == SensorHealth.DEGRADED
    assert later.detected_fault == 'underexposed'
    metrics = _metrics(later)
    assert metrics['underexposure_candidate_count'] == 1.0
    assert metrics['underexposure_candidate_duration_sec'] == 0.0
    assert metrics['fault_confirmation_elapsed_sec'] == 0.0


def test_healthy_dark_baseline_does_not_confirm_underexposure(monkeypatch):
    evaluator = _evaluator(freeze_duration_sec=10.0)
    brief_black_start = [
        _feature_result(
            f'start-{index}',
            mean_gray=0.274,
            p95=0.0,
            dark_ratio=0.994,
        )
        for index in range(2)
    ]
    black_keyboard = [
        _feature_result(
            f'keyboard-{index}',
            mean_gray=1.660,
            p95=10.545,
            dark_ratio=0.981,
        )
        for index in range(12)
    ]
    _install_feature_sequence(
        monkeypatch, brief_black_start + black_keyboard
    )

    decisions = []
    receive_times = [0.0, 0.063] + [0.126 + 0.125 * i for i in range(12)]
    for index, received_sec in enumerate(receive_times):
        _add(evaluator, received_sec, _image(index))
        decisions.append(evaluator.evaluate(received_sec))

    assert decisions[0].detected_fault == 'underexposed'
    assert decisions[0].state == SensorHealth.DEGRADED
    assert all(decision.state != SensorHealth.FAULT for decision in decisions)
    assert decisions[-1].state == SensorHealth.HEALTHY
    assert decisions[-1].detected_fault == 'none'
    assert _metrics(decisions[-1])['exposure_dark_candidate'] == 0.0


def test_stale_still_has_priority_over_underexposure(monkeypatch):
    evaluator = _evaluator(freeze_duration_sec=10.0)
    _install_feature_sequence(monkeypatch, [
        _feature_result(
            f'dark-{index}', mean_gray=0.0, p95=0.0, dark_ratio=1.0
        )
        for index in range(4)
    ])
    for index in range(4):
        received_sec = index * 0.2
        _add(evaluator, received_sec, _image(index))
        evaluator.evaluate(received_sec)

    stale_pending = evaluator.evaluate(1.61)
    stale_fault = evaluator.evaluate(2.12)

    assert stale_pending.detected_fault == 'stale'
    assert stale_pending.state == SensorHealth.FAULT
    assert stale_fault.detected_fault == 'stale'
    assert stale_fault.state == SensorHealth.FAULT


def test_invalid_parameters_are_rejected():
    with pytest.raises(ValueError, match='stale_timeout_sec'):
        _evaluator(stale_timeout_sec=0.0)
    with pytest.raises(ValueError, match='min_samples'):
        _evaluator(min_samples=1)
    with pytest.raises(ValueError, match='mean-gray'):
        _evaluator(
            dark_mean_gray_candidate=200.0,
            bright_mean_gray_candidate=100.0,
        )
    with pytest.raises(ValueError, match='dark_p95_candidate'):
        _evaluator(dark_p95_candidate=256.0)
    with pytest.raises(ValueError, match='bright percentile'):
        _evaluator(
            bright_p05_candidate=200.0,
            bright_p95_candidate=150.0,
        )
    with pytest.raises(ValueError, match='Laplacian reference'):
        _evaluator(
            blur_reference_laplacian_variance_min=5.0,
            blur_laplacian_variance_candidate=10.0,
        )
    with pytest.raises(ValueError, match='low-information edge'):
        _evaluator(
            low_information_reference_edge_density_min=0.0001,
            low_information_edge_density_candidate=0.0005,
        )
