"""Compare descriptive camera features across one truth-labelled session."""

import argparse
import csv
from datetime import datetime, timezone
import json
from math import isfinite
import os
from pathlib import Path
import time

import numpy as np


DEFAULT_TRANSITION_MARGIN_SEC = 2.0

FEATURE_FAMILIES = {
    'exposure': (
        'mean_gray',
        'p05',
        'p95',
        'dark_ratio',
        'bright_ratio',
    ),
    'blur': (
        'laplacian_variance',
        'edge_density',
        'entropy',
    ),
    'low_information': (
        'gray_std',
        'edge_density',
        'entropy',
        'dark_ratio',
        'bright_ratio',
    ),
}

PHASES = ('healthy_pre', 'active_fault', 'recovered_post')
STATISTIC_FIELDS = (
    'count',
    'mean',
    'std',
    'min',
    'p05',
    'p50',
    'p95',
    'max',
)


def _utc_now_text(epoch_sec=None):
    """Return an ISO-8601 UTC timestamp for report metadata."""
    if epoch_sec is None:
        epoch_sec = time.time()
    return datetime.fromtimestamp(epoch_sec, timezone.utc).isoformat()


def _descriptive_statistics(values):
    """Return finite descriptive statistics with stable null fields."""
    finite_values = [float(value) for value in values if isfinite(value)]
    result = {field: None for field in STATISTIC_FIELDS}
    result['count'] = len(finite_values)
    if not finite_values:
        return result
    array = np.asarray(finite_values, dtype=np.float64)
    result.update({
        'mean': float(np.mean(array)),
        'std': float(np.std(array)),
        'min': float(np.min(array)),
        'p05': float(np.percentile(array, 5)),
        'p50': float(np.percentile(array, 50)),
        'p95': float(np.percentile(array, 95)),
        'max': float(np.max(array)),
    })
    return result


def _finite_float(value, *, field, source):
    """Parse one required finite numeric session value."""
    try:
        parsed = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{source}: {field} is not numeric') from error
    if not isfinite(parsed):
        raise ValueError(f'{source}: {field} must be finite')
    return parsed


def _complete_truth_event(summary, summary_path):
    """Return the only event with both ACTIVE and ENDED transitions."""
    transitions = (summary.get('fault_truth') or {}).get('transitions') or []
    by_event = {}
    for transition in transitions:
        event_id = str(transition.get('event_id', '')).strip()
        if event_id:
            by_event.setdefault(event_id, []).append(transition)

    complete = []
    for event_id, event_transitions in by_event.items():
        active = next((
            item for item in event_transitions
            if item.get('truth_state') == 'ACTIVE'
        ), None)
        ended = next((
            item for item in event_transitions
            if item.get('truth_state') == 'ENDED'
        ), None)
        if active is not None and ended is not None:
            complete.append((event_id, active, ended))
    if len(complete) != 1:
        raise ValueError(
            f'{summary_path}: expected exactly one event with ACTIVE and ENDED'
        )

    event_id, active, ended = complete[0]
    active_sec = _finite_float(
        active.get('received_time_sec'),
        field='ACTIVE.received_time_sec',
        source=summary_path,
    )
    ended_sec = _finite_float(
        ended.get('received_time_sec'),
        field='ENDED.received_time_sec',
        source=summary_path,
    )
    if ended_sec <= active_sec:
        raise ValueError(f'{summary_path}: ENDED must follow ACTIVE')
    return {
        'event_id': event_id,
        'truth_model': str(active.get('truth_model', '')),
        'severity': _finite_float(
            active.get('severity'), field='severity', source=summary_path
        ),
        'active_received_time_sec': active_sec,
        'ended_received_time_sec': ended_sec,
        'truth_start_time_sec': active.get('start_time_sec'),
        'truth_end_time_sec': ended.get('end_time_sec'),
    }


def _load_truth_session(session_directory):
    """Load one fault-development session and its complete truth event."""
    session_directory = Path(session_directory).expanduser()
    if not session_directory.is_dir():
        raise ValueError(
            f'session directory is not a directory: {session_directory}'
        )
    summary_path = session_directory / 'baseline_summary.json'
    samples_path = session_directory / 'feature_samples.csv'
    try:
        summary = json.loads(summary_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f'{summary_path}: cannot read valid JSON: {error}')
    if not summary.get('record_fault_truth'):
        raise ValueError(f'{summary_path}: fault truth recording is not enabled')
    event = _complete_truth_event(summary, summary_path)

    required_fields = {
        'event_id',
        'truth_model',
        'truth_state',
        'severity',
        'receive_time_sec',
        *{
            metric
            for metrics in FEATURE_FAMILIES.values()
            for metric in metrics
        },
    }
    try:
        with samples_path.open(newline='', encoding='utf-8') as stream:
            reader = csv.DictReader(stream)
            missing = required_fields.difference(reader.fieldnames or ())
            if missing:
                raise ValueError(
                    f'{samples_path}: missing columns: '
                    + ', '.join(sorted(missing))
                )
            rows = list(reader)
    except OSError as error:
        raise ValueError(f'{samples_path}: cannot read CSV: {error}')
    return session_directory, summary, event, rows


def _phase_for_sample(row, event, transition_margin_sec):
    """Classify a sample outside both transition exclusion margins."""
    sample_sec = _finite_float(
        row.get('receive_time_sec'),
        field='receive_time_sec',
        source='feature sample',
    )
    active_sec = event['active_received_time_sec']
    ended_sec = event['ended_received_time_sec']
    state = row.get('truth_state', '')
    event_id = row.get('event_id', '')
    if (
        sample_sec < active_sec - transition_margin_sec
        and state in {'', 'SCHEDULED'}
        and event_id in {'', event['event_id']}
    ):
        return 'healthy_pre'
    if (
        sample_sec > active_sec + transition_margin_sec
        and sample_sec < ended_sec - transition_margin_sec
        and state == 'ACTIVE'
        and event_id == event['event_id']
    ):
        return 'active_fault'
    if (
        sample_sec > ended_sec + transition_margin_sec
        and state == 'ENDED'
        and event_id == event['event_id']
    ):
        return 'recovered_post'
    return None


def _focus_family(truth_model):
    """Choose the compact terminal feature family for a truth model."""
    normalized = truth_model.strip().lower()
    if normalized in {'underexposure', 'overexposure', 'exposure'}:
        return 'exposure'
    if normalized in {'blur', 'blurred'}:
        return 'blur'
    if normalized in {'occlusion', 'low_information'}:
        return 'low_information'
    return 'exposure'


def build_fault_feature_report(
    session_directory,
    *,
    transition_margin_sec=DEFAULT_TRANSITION_MARGIN_SEC,
    generated_at_epoch_sec=None,
):
    """Build phase-separated descriptive statistics without thresholds."""
    if (
        not isfinite(transition_margin_sec)
        or transition_margin_sec < 0.0
    ):
        raise ValueError('transition_margin_sec must be finite and non-negative')
    session_directory, summary, event, rows = _load_truth_session(
        session_directory
    )
    grouped_rows = {phase: [] for phase in PHASES}
    excluded_count = 0
    for row in rows:
        phase = _phase_for_sample(row, event, transition_margin_sec)
        if phase is None:
            excluded_count += 1
        else:
            grouped_rows[phase].append(row)

    phase_statistics = {}
    for phase in PHASES:
        phase_statistics[phase] = {
            metric: _descriptive_statistics([
                _finite_float(
                    row[metric], field=metric, source='feature sample'
                )
                for row in grouped_rows[phase]
            ])
            for metric in {
                metric
                for metrics in FEATURE_FAMILIES.values()
                for metric in metrics
            }
        }

    families = {}
    candidate_intervals = {}
    for family, metrics in FEATURE_FAMILIES.items():
        families[family] = {
            'metrics': list(metrics),
            'phase_statistics': {
                phase: {
                    metric: phase_statistics[phase][metric]
                    for metric in metrics
                }
                for phase in PHASES
            },
        }
        candidate_intervals[family] = {
            metric: {
                phase: [
                    phase_statistics[phase][metric]['p05'],
                    phase_statistics[phase][metric]['p95'],
                ]
                for phase in PHASES
            }
            for metric in metrics
        }

    return {
        'schema_version': 1,
        'description_only': True,
        'thresholds_generated': False,
        'candidate_intervals_are_thresholds': False,
        'session_directory': str(session_directory),
        'session_id': summary.get('session_id'),
        'scenario_label': summary.get('scenario_label'),
        'generated_at_utc': _utc_now_text(generated_at_epoch_sec),
        'transition_margin_sec': float(transition_margin_sec),
        'transition_policy': (
            'strictly outside ACTIVE/ENDED +/- margin; boundary samples '
            'and truth-state mismatches are excluded'
        ),
        'truth_event': event,
        'input_sample_count': len(rows),
        'excluded_transition_sample_count': excluded_count,
        'groups': {
            phase: {
                'sample_count': len(grouped_rows[phase]),
            }
            for phase in PHASES
        },
        'feature_families': families,
        'candidate_p05_p95_intervals': candidate_intervals,
        'terminal_focus_family': _focus_family(event['truth_model']),
    }


def _format_number(value):
    if value is None:
        return '-'
    return f'{value:.3f}'


def format_fault_report_table(report):
    """Return a compact phase comparison for the relevant feature family."""
    family = report['terminal_focus_family']
    metrics = report['feature_families'][family]['metrics']
    headers = ('phase', 'n', *metrics)
    rows = []
    for phase in PHASES:
        statistics = report['feature_families'][family][
            'phase_statistics'
        ][phase]
        rows.append((
            phase,
            str(report['groups'][phase]['sample_count']),
            *(_format_number(statistics[metric]['mean']) for metric in metrics),
        ))
    widths = [
        max(len(headers[index]), *(len(row[index]) for row in rows))
        for index in range(len(headers))
    ]

    def format_row(row):
        return '  '.join(
            item.ljust(widths[index])
            for index, item in enumerate(row)
        )

    prefix = (
        f'model={report["truth_event"]["truth_model"]} '
        f'focus={family} margin={report["transition_margin_sec"]:.3f}s '
        f'excluded={report["excluded_transition_sample_count"]}'
    )
    return '\n'.join([
        prefix,
        format_row(headers),
        '  '.join('-' * width for width in widths),
        *(format_row(row) for row in rows),
    ])


def _atomic_write_json(path, document):
    """Atomically write one deterministic JSON report."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = path.with_name(path.name + '.tmp')
    with temporary_path.open('w', encoding='utf-8') as stream:
        json.dump(document, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary_path, path)


def write_fault_feature_report(
    session_directory,
    *,
    transition_margin_sec=DEFAULT_TRANSITION_MARGIN_SEC,
    output_path=None,
    generated_at_epoch_sec=None,
):
    """Build and persist one fault-development descriptive report."""
    report = build_fault_feature_report(
        session_directory,
        transition_margin_sec=transition_margin_sec,
        generated_at_epoch_sec=generated_at_epoch_sec,
    )
    if output_path is None:
        output_path = (
            Path(session_directory).expanduser()
            / 'camera_fault_feature_report.json'
        )
    else:
        output_path = Path(output_path).expanduser()
    _atomic_write_json(output_path, report)
    return report, output_path


def _argument_parser():
    parser = argparse.ArgumentParser(
        description=(
            'Compare truth-labelled camera features descriptively. No final '
            'fault thresholds are generated.'
        )
    )
    parser.add_argument('session_directory')
    parser.add_argument(
        '--transition-margin-sec',
        type=float,
        default=DEFAULT_TRANSITION_MARGIN_SEC,
    )
    parser.add_argument('--output')
    return parser


def main(argv=None):
    """Run the pure offline camera fault feature report tool."""
    arguments = _argument_parser().parse_args(argv)
    report, output_path = write_fault_feature_report(
        arguments.session_directory,
        transition_margin_sec=arguments.transition_margin_sec,
        output_path=arguments.output,
    )
    print(format_fault_report_table(report))
    print(f'Wrote descriptive report: {output_path}')


if __name__ == '__main__':
    main()
