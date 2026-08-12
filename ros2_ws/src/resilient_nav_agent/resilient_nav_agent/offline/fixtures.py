"""Versioned deterministic RA-1A reference fixtures, never recorded runs."""

from __future__ import annotations

from copy import deepcopy

from resilient_nav_agent.offline.case_builder import OfflineCaseBuilder
from resilient_nav_agent.offline.schemas import OfflineRobotCase
from resilient_nav_agent.schemas import EvidenceItem, EvidenceType


REFERENCE_FIXTURE_VERSION = 'ra1a-reference-v1'


def _health(
    component,
    source_topic,
    state,
    score,
    hint,
    reasons,
    metrics,
    *,
    observed_at=12.1,
):
    return {
        'header': {
            'stamp': {'sec': int(observed_at), 'nanosec': 100_000_000},
            'frame_id': '',
        },
        'sensor': component,
        'source_topic': source_topic,
        'state': state,
        'health_score': score,
        'confidence': 1.0 if state in (1, 3) else 0.9,
        'detected_fault': hint,
        'reasons': list(reasons),
        'metric_names': list(metrics),
        'metric_values': list(metrics.values()),
        'window_start': {'sec': 10, 'nanosec': 0},
        'window_end': {'sec': 12, 'nanosec': 0},
        'sample_count': 20,
    }


def _truth(
    component,
    fault_type,
    evidence_types,
    *,
    acceptable_fault_types=(),
    source_scenario=None,
    truth_metadata=None,
):
    return {
        'expected_incident': True,
        'expected_component': component,
        'expected_fault_type': fault_type,
        'acceptable_fault_types': list(acceptable_fault_types),
        'acceptable_components': [],
        'expected_should_diagnose': True,
        'minimum_required_evidence_types': list(evidence_types),
        'notes': 'Deterministic reference fixture; not a recorded real run.',
        'source_scenario': source_scenario,
        'truth_metadata': {
            'fixture_kind': 'reference_fixture',
            'fixture_version': REFERENCE_FIXTURE_VERSION,
            **(truth_metadata or {}),
        },
    }


def _reference_specs():
    healthy_wheel = _health(
        'wheel',
        '/wheel/odometry',
        1,
        1.0,
        'none',
        ['timing_within_limits'],
        {
            'message_age_sec': 0.02,
            'wheel_pose_span_m': 0.31,
            'wheel_angular_span_rad_s': 0.02,
        },
    )
    healthy_imu = _health(
        'imu',
        '/imu/data',
        1,
        1.0,
        'none',
        ['timing_within_limits'],
        {
            'message_age_sec': 0.02,
            'imu_wheel_corrected_residual_mean_rad_s': 0.01,
        },
    )
    return [
        {
            'case_id': 'CASE-001',
            'raw_health': [
                _health(
                    'imu',
                    '/faulted/imu/data',
                    3,
                    0.0,
                    'bias',
                    ['imu_wheel_residual_exceeded_fault_threshold'],
                    {
                        'message_age_sec': 0.02,
                        'imu_wheel_pair_count': 40.0,
                        'imu_wheel_corrected_residual_mean_rad_s': 0.15,
                        'imu_wheel_residual_stddev_rad_s': 0.012,
                    },
                ),
                healthy_wheel,
            ],
            'truth_mapping': _truth(
                'imu',
                'imu_bias',
                ['health'],
                acceptable_fault_types=['bias', 'z_gyro_bias'],
                source_scenario='imu_bias_demo',
                truth_metadata={
                    'scenario_seed': 20260803,
                    'parameters_yaml': 'bias_rad_s: 0.15',
                },
            ),
        },
        {
            'case_id': 'CASE-002',
            'raw_health': [
                _health(
                    'wheel',
                    '/faulted/wheel/odometry',
                    3,
                    0.0,
                    'freeze',
                    ['commanded_motion_but_wheel_odometry_static'],
                    {
                        'message_age_sec': 0.02,
                        'wheel_pose_span_m': 0.0,
                        'wheel_linear_span_mps': 0.0,
                        'commanded_linear_abs_mps': 0.2,
                    },
                ),
                healthy_imu,
            ],
            'truth_mapping': _truth(
                'wheel',
                'wheel_freeze',
                ['health', 'motion'],
                acceptable_fault_types=['freeze'],
                source_scenario='wheel_freeze_demo',
                truth_metadata={'scenario_seed': 20260803},
            ),
            'supplemental_evidence': [
                EvidenceItem(
                    evidence_id='E-CASE-002-MOTION-01',
                    evidence_type=EvidenceType.MOTION,
                    component='wheel',
                    source='motion-observer:wheel',
                    start_sec=10.0,
                    end_sec=12.0,
                    summary='Commanded motion continued while wheel pose stayed static.',
                    structured_data={
                        'commanded_linear_abs_mps': 0.2,
                        'wheel_pose_span_m': 0.0,
                        'sample_count': 20,
                    },
                ),
            ],
        },
        {
            'case_id': 'CASE-003',
            'raw_health': [
                _health(
                    'scan',
                    '/faulted/scan',
                    3,
                    0.0,
                    'sector_blindness',
                    ['contiguous_nan_sector_exceeded_fault_threshold'],
                    {
                        'message_age_sec': 0.03,
                        'scan_nan_ratio': 0.097,
                        'scan_nan_count': 70.0,
                        'longest_nan_sector_width_rad': 1.0,
                    },
                ),
            ],
            'truth_mapping': _truth(
                'scan',
                'lidar_sector_blindness',
                ['health'],
                acceptable_fault_types=['sector_blindness'],
                source_scenario='scan_sector_blindness_demo',
                truth_metadata={'sector_width_rad': 1.0},
            ),
        },
        {
            'case_id': 'CASE-004',
            'raw_health': [
                _health(
                    'camera',
                    '/camera/c920/image_raw',
                    3,
                    0.0,
                    'stale',
                    ['message_age_exceeded_stale_timeout'],
                    {
                        'message_age_sec': 1.35,
                        'rolling_observed_fps': 0.0,
                        'max_recent_gap_sec': 1.35,
                    },
                ),
            ],
            'truth_mapping': _truth(
                'camera',
                'camera_stale',
                ['camera_health'],
                acceptable_fault_types=['stale'],
            ),
        },
        {
            'case_id': 'CASE-005',
            'raw_health': [
                _health(
                    'camera',
                    '/camera/c920/image_raw',
                    3,
                    0.0,
                    'freeze',
                    ['image_stamps_progressed_while_fingerprint_remained_identical'],
                    {
                        'message_age_sec': 0.03,
                        'fingerprint_identical_duration_sec': 2.4,
                        'fingerprint_identical_count': 31.0,
                        'fingerprint_stamp_progress_continuous': 1.0,
                    },
                ),
            ],
            'truth_mapping': _truth(
                'camera',
                'camera_freeze',
                ['camera_health'],
                acceptable_fault_types=['freeze'],
                truth_metadata={'manual_event_model': 'freeze'},
            ),
        },
        {
            'case_id': 'CASE-006',
            'raw_health': [
                _health(
                    'camera',
                    '/camera/c920/image_raw',
                    3,
                    0.2,
                    'camera_quality',
                    ['sustained_dark_exposure_candidate'],
                    {
                        'message_age_sec': 0.03,
                        'mean_gray': 4.035,
                        'p95': 5.506,
                        'dark_ratio': 0.946,
                        'exposure_dark_candidate': 1.0,
                    },
                ),
            ],
            'truth_mapping': _truth(
                'camera',
                'underexposed',
                ['camera_health'],
                acceptable_fault_types=['underexposure'],
                truth_metadata={'manual_event_model': 'underexposure'},
            ),
        },
        {
            'case_id': 'CASE-007',
            'raw_health': [
                _health(
                    'camera',
                    '/camera/c920/image_raw',
                    3,
                    0.1,
                    'camera_quality',
                    ['texture_reference_lost_detail'],
                    {
                        'message_age_sec': 0.03,
                        'laplacian_variance': 6.853,
                        'edge_density': 0.0,
                        'entropy': 5.776,
                        'gray_std': 24.377,
                        'blur_reference_laplacian_variance': 309.68,
                        'blur_candidate': 1.0,
                    },
                ),
            ],
            'truth_mapping': _truth(
                'camera',
                'blurred',
                ['camera_health'],
                acceptable_fault_types=['blur'],
                truth_metadata={'manual_event_model': 'blur'},
            ),
        },
        {
            'case_id': 'CASE-008',
            'raw_health': [healthy_imu, healthy_wheel],
            'truth_mapping': {
                'expected_incident': False,
                'expected_component': None,
                'expected_fault_type': None,
                'acceptable_fault_types': [],
                'acceptable_components': [],
                'expected_should_diagnose': False,
                'minimum_required_evidence_types': [],
                'notes': 'Healthy reference fixture; not a recorded real run.',
                'source_scenario': None,
                'truth_metadata': {
                    'fixture_kind': 'reference_fixture',
                    'fixture_version': REFERENCE_FIXTURE_VERSION,
                },
            },
        },
    ]


def reference_robot_cases() -> list[OfflineRobotCase]:
    """Return fresh, deterministic CASE-001 through CASE-008 objects."""
    builder = OfflineCaseBuilder()
    return [builder.build(**spec) for spec in deepcopy(_reference_specs())]
