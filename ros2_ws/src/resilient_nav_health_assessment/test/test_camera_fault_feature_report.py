"""Tests for truth-labelled camera fault feature reports."""

import csv
import json

import pytest
from resilient_nav_health_assessment.camera_fault_feature_report import (
    build_fault_feature_report,
    FEATURE_FAMILIES,
    format_fault_report_table,
    main,
    write_fault_feature_report,
)


ALL_METRICS = tuple(dict.fromkeys(
    metric
    for metrics in FEATURE_FAMILIES.values()
    for metric in metrics
))


def _session(tmp_path, *, model='underexposure', complete=True):
    session = tmp_path / 'fault-session'
    session.mkdir()
    transitions = [
        {
            'event_id': 'camera-event-1',
            'truth_model': model,
            'truth_state': 'ACTIVE',
            'severity': 0.8,
            'received_time_sec': 100.0,
            'start_time_sec': 100.0,
            'end_time_sec': 200.0,
        },
    ]
    if complete:
        transitions.append({
            'event_id': 'camera-event-1',
            'truth_model': model,
            'truth_state': 'ENDED',
            'severity': 0.8,
            'received_time_sec': 200.0,
            'start_time_sec': 100.0,
            'end_time_sec': 200.0,
        })
    summary = {
        'session_id': 'fault-session',
        'scenario_label': 'manual_visual_fault',
        'record_fault_truth': True,
        'fault_truth': {'recorded': True, 'transitions': transitions},
    }
    (session / 'baseline_summary.json').write_text(
        json.dumps(summary), encoding='utf-8'
    )

    samples = [
        (97.0, 'SCHEDULED'),
        (98.0, 'SCHEDULED'),
        (99.0, 'SCHEDULED'),
        (102.0, 'ACTIVE'),
        (103.0, 'ACTIVE'),
        (197.0, 'ACTIVE'),
        (198.0, 'ACTIVE'),
        (199.0, 'ACTIVE'),
        (202.0, 'ENDED'),
        (203.0, 'ENDED'),
    ]
    fields = (
        'event_id',
        'truth_model',
        'truth_state',
        'severity',
        'receive_time_sec',
        *ALL_METRICS,
    )
    with (session / 'feature_samples.csv').open(
        'w', newline='', encoding='utf-8'
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index, (receive_time_sec, truth_state) in enumerate(samples):
            row = {
                'event_id': 'camera-event-1',
                'truth_model': model,
                'truth_state': truth_state,
                'severity': 0.8,
                'receive_time_sec': receive_time_sec,
            }
            row.update({
                metric: index + offset / 10.0
                for offset, metric in enumerate(ALL_METRICS)
            })
            writer.writerow(row)
    return session


def test_default_margin_excludes_both_transition_neighbourhoods(tmp_path):
    report = build_fault_feature_report(
        _session(tmp_path), generated_at_epoch_sec=0.0
    )

    assert report['transition_margin_sec'] == 2.0
    assert report['groups']['healthy_pre']['sample_count'] == 1
    assert report['groups']['active_fault']['sample_count'] == 2
    assert report['groups']['recovered_post']['sample_count'] == 1
    assert report['excluded_transition_sample_count'] == 6
    assert report['input_sample_count'] == 10


def test_zero_margin_keeps_all_samples_strictly_between_transitions(tmp_path):
    report = build_fault_feature_report(
        _session(tmp_path), transition_margin_sec=0.0
    )

    assert report['groups']['healthy_pre']['sample_count'] == 3
    assert report['groups']['active_fault']['sample_count'] == 5
    assert report['groups']['recovered_post']['sample_count'] == 2
    assert report['excluded_transition_sample_count'] == 0


def test_report_contains_required_families_and_candidate_intervals(tmp_path):
    report = build_fault_feature_report(_session(tmp_path))

    assert report['description_only'] is True
    assert report['thresholds_generated'] is False
    assert report['candidate_intervals_are_thresholds'] is False
    assert set(report['feature_families']) == {
        'exposure', 'blur', 'low_information'
    }
    assert report['feature_families']['exposure']['metrics'] == [
        'mean_gray', 'p05', 'p95', 'dark_ratio', 'bright_ratio'
    ]
    assert report['feature_families']['blur']['metrics'] == [
        'laplacian_variance', 'edge_density', 'entropy'
    ]
    assert report['feature_families']['low_information']['metrics'] == [
        'gray_std', 'edge_density', 'entropy', 'dark_ratio', 'bright_ratio'
    ]
    interval = report['candidate_p05_p95_intervals']['exposure'][
        'mean_gray'
    ]['active_fault']
    assert len(interval) == 2
    assert interval[0] <= interval[1]


@pytest.mark.parametrize(
    ('model', 'family', 'expected_metric'),
    [
        ('underexposure', 'exposure', 'mean_gray'),
        ('blur', 'blur', 'laplacian_variance'),
        ('occlusion', 'low_information', 'gray_std'),
    ],
)
def test_terminal_table_focuses_on_truth_model(
    tmp_path, model, family, expected_metric
):
    report = build_fault_feature_report(_session(tmp_path, model=model))
    table = format_fault_report_table(report)

    assert f'focus={family}' in table
    assert expected_metric in table
    assert 'healthy_pre' in table
    assert 'active_fault' in table
    assert 'recovered_post' in table


def test_report_is_written_as_json_and_main_prints_table(tmp_path, capsys):
    session = _session(tmp_path)
    report, output_path = write_fault_feature_report(
        session, generated_at_epoch_sec=0.0
    )

    assert output_path == session / 'camera_fault_feature_report.json'
    assert json.loads(output_path.read_text()) == report
    main([str(session), '--transition-margin-sec', '2.0'])
    output = capsys.readouterr().out
    assert 'active_fault' in output
    assert 'Wrote descriptive report:' in output


def test_invalid_margin_and_incomplete_truth_are_rejected(tmp_path):
    session = _session(tmp_path, complete=False)

    with pytest.raises(ValueError, match='non-negative'):
        build_fault_feature_report(session, transition_margin_sec=-0.1)
    with pytest.raises(ValueError, match='exactly one event'):
        build_fault_feature_report(session)
