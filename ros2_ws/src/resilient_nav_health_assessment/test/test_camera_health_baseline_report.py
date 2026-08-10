"""Tests for multi-session descriptive camera baseline reports."""

import json

import numpy as np
import pytest
from resilient_nav_health_assessment.camera_health_baseline_report import (
    build_baseline_report,
    format_report_table,
    main,
    write_baseline_report,
)
from resilient_nav_health_assessment.camera_health_calibrate import (
    CameraHealthCalibrationSession,
)


def _record_session(
    root,
    *,
    scenario_label,
    session_id,
    values,
    interval_sec,
):
    session = CameraHealthCalibrationSession(
        output_directory=root,
        scenario_label=scenario_label,
        session_id=session_id,
        duration_sec=1.0,
        started_receive_time_sec=1000.0,
        started_monotonic_sec=10.0,
    )
    for index, value in enumerate(values):
        session.add_frame(
            np.full((8, 8), value, dtype=np.uint8),
            channel_order='rgb',
            header_stamp_sec=100 + index,
            header_stamp_nanosec=200 + index,
            receive_time_sec=1000.0 + index * interval_sec,
            receive_monotonic_sec=10.0 + index * interval_sec,
        )
    session.finalize(
        stop_reason='test_complete',
        controls_snapshot='read only\n',
        finished_receive_time_sec=1001.0,
        finished_monotonic_sec=11.0,
    )
    return session


@pytest.fixture
def baseline_root(tmp_path):
    """Create three independently persisted sessions in two scenarios."""
    _record_session(
        tmp_path,
        scenario_label='indoor',
        session_id='indoor-a',
        values=(0, 100),
        interval_sec=0.1,
    )
    _record_session(
        tmp_path,
        scenario_label='indoor',
        session_id='indoor-b',
        values=(100, 140),
        interval_sec=0.2,
    )
    _record_session(
        tmp_path,
        scenario_label='outdoor',
        session_id='outdoor-a',
        values=(200, 220),
        interval_sec=0.25,
    )
    return tmp_path


def test_report_has_global_and_scenario_descriptive_statistics(baseline_root):
    report = build_baseline_report(
        baseline_root, generated_at_epoch_sec=0.0
    )

    assert report['description_only'] is True
    assert report['generated_at_utc'] == '1970-01-01T00:00:00+00:00'
    assert report['scanned_session_count'] == 3
    assert report['skipped_session_count'] == 0
    assert set(report['by_scenario_label']) == {'indoor', 'outdoor'}
    assert report['global']['session_count'] == 3
    assert report['global']['sample_count'] == 6
    assert report['global']['session_metrics']['observed_fps']['mean'] == (
        pytest.approx((10.0 + 5.0 + 4.0) / 3.0)
    )
    assert report['global']['session_metrics']['max_gap_sec']['max'] == (
        pytest.approx(0.25)
    )
    assert report['global']['sample_metrics']['interarrival_sec']['p50'] == (
        pytest.approx(0.2)
    )
    assert report['global']['sample_metrics']['mean_gray']['mean'] == (
        pytest.approx(126.6666667)
    )
    assert report['global']['sample_metrics']['frame_diff_mean']['mean'] == (
        pytest.approx((100.0 + 40.0 + 20.0) / 3.0)
    )
    assert report['by_scenario_label']['indoor']['session_count'] == 2
    assert report['by_scenario_label']['indoor']['sample_count'] == 4


def test_report_json_and_terminal_table_are_classification_free(
    baseline_root,
):
    report, output_path = write_baseline_report(
        baseline_root, generated_at_epoch_sec=0.0
    )

    assert output_path == baseline_root / 'baseline_report.json'
    saved = json.loads(output_path.read_text(encoding='utf-8'))
    assert saved == report
    report_text = output_path.read_text(encoding='utf-8')
    assert 'HEALTHY' not in report_text
    assert 'FAULT' not in report_text

    table = format_report_table(report)
    assert 'scenario' in table
    assert '[global]' in table
    assert 'indoor' in table
    assert 'outdoor' in table
    assert 'fps_mean' in table
    assert 'gap_max' in table


def test_incomplete_and_invalid_sessions_are_reported_as_skipped(tmp_path):
    incomplete = tmp_path / 'incomplete'
    incomplete.mkdir()
    (incomplete / 'baseline_summary.json').write_text('{}')
    invalid = tmp_path / 'invalid'
    invalid.mkdir()
    (invalid / 'baseline_summary.json').write_text('{}')
    (invalid / 'feature_samples.csv').write_text('bad,column\n')

    report = build_baseline_report(tmp_path, generated_at_epoch_sec=0.0)

    assert report['scanned_session_count'] == 0
    assert report['skipped_session_count'] == 2
    assert report['global']['session_count'] == 0
    assert report['global']['sample_count'] == 0
    assert report['global']['session_metrics']['observed_fps']['mean'] is None


def test_sample_metadata_mismatch_skips_the_whole_session(tmp_path):
    session = _record_session(
        tmp_path,
        scenario_label='indoor',
        session_id='session-a',
        values=(10, 20),
        interval_sec=0.1,
    )
    csv_path = session.session_directory / 'feature_samples.csv'
    csv_path.write_text(
        csv_path.read_text().replace('indoor,session-a', 'wrong,session-a'),
        encoding='utf-8',
    )

    report = build_baseline_report(tmp_path)

    assert report['scanned_session_count'] == 0
    assert report['skipped_session_count'] == 1
    assert 'metadata mismatch' in report['skipped_sessions'][0]['reason']


def test_main_writes_default_report_and_prints_table(
    baseline_root, capsys,
):
    main([str(baseline_root)])

    output = capsys.readouterr().out
    assert '[global]' in output
    assert 'Wrote descriptive report:' in output
    assert (baseline_root / 'baseline_report.json').is_file()


def test_missing_baseline_root_is_rejected(tmp_path):
    with pytest.raises(ValueError, match='not a directory'):
        build_baseline_report(tmp_path / 'missing')
