"""Aggregate descriptive camera baseline sessions without classification."""

import argparse
import csv
from datetime import datetime, timezone
import json
from math import isfinite
import os
from pathlib import Path
import time

import numpy as np


SESSION_METRICS = (
    'observed_fps',
    'max_gap_sec',
)

SAMPLE_METRICS = (
    'interarrival_sec',
    'mean_gray',
    'laplacian_variance',
    'edge_density',
    'entropy',
    'frame_diff_mean',
)

STATISTIC_FIELDS = (
    'count',
    'mean',
    'std',
    'min',
    'p05',
    'p50',
    'p95',
    'p99',
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
        'p99': float(np.percentile(array, 99)),
        'max': float(np.max(array)),
    })
    return result


def _optional_finite_float(value, *, field, source):
    """Parse an optional finite float from one session artifact."""
    if value is None or value == '':
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{source}: {field} is not numeric') from error
    if not isfinite(parsed):
        raise ValueError(f'{source}: {field} must be finite')
    return parsed


def _load_session(session_directory):
    """Load and validate one session summary and feature CSV."""
    summary_path = session_directory / 'baseline_summary.json'
    samples_path = session_directory / 'feature_samples.csv'
    try:
        summary = json.loads(summary_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f'{summary_path}: cannot read valid JSON: {error}')

    scenario_label = str(summary.get('scenario_label', '')).strip()
    session_id = str(summary.get('session_id', '')).strip()
    if not scenario_label:
        raise ValueError(f'{summary_path}: scenario_label is missing')
    if not session_id:
        raise ValueError(f'{summary_path}: session_id is missing')
    if session_id != session_directory.name:
        raise ValueError(
            f'{summary_path}: session_id does not match directory name'
        )

    try:
        with samples_path.open(newline='', encoding='utf-8') as stream:
            reader = csv.DictReader(stream)
            required_fields = {
                'scenario_label',
                'session_id',
                *SAMPLE_METRICS,
            }
            missing = required_fields.difference(reader.fieldnames or ())
            if missing:
                names = ', '.join(sorted(missing))
                raise ValueError(f'{samples_path}: missing columns: {names}')
            rows = list(reader)
    except OSError as error:
        raise ValueError(f'{samples_path}: cannot read CSV: {error}')

    sample_metrics = {metric: [] for metric in SAMPLE_METRICS}
    for row_number, row in enumerate(rows, start=2):
        source = f'{samples_path}:{row_number}'
        if row['scenario_label'] != scenario_label:
            raise ValueError(f'{source}: scenario_label metadata mismatch')
        if row['session_id'] != session_id:
            raise ValueError(f'{source}: session_id metadata mismatch')
        for metric in SAMPLE_METRICS:
            value = _optional_finite_float(
                row[metric], field=metric, source=source
            )
            if value is not None:
                sample_metrics[metric].append(value)

    interarrival_summary = summary.get('interarrival_sec') or {}
    session_metrics = {
        'observed_fps': _optional_finite_float(
            summary.get('observed_fps'),
            field='observed_fps',
            source=summary_path,
        ),
        'max_gap_sec': _optional_finite_float(
            interarrival_summary.get('max_gap'),
            field='interarrival_sec.max_gap',
            source=summary_path,
        ),
    }
    return {
        'scenario_label': scenario_label,
        'session_id': session_id,
        'session_directory': str(session_directory),
        'sample_count': len(rows),
        'session_metrics': session_metrics,
        'sample_metrics': sample_metrics,
    }


def scan_baseline_sessions(baseline_root):
    """Load valid immediate session directories and record skipped artifacts."""
    root = Path(baseline_root).expanduser()
    if not root.is_dir():
        raise ValueError(f'baseline root is not a directory: {root}')

    sessions = []
    skipped = []
    for directory in sorted(root.iterdir(), key=lambda path: path.name):
        if not directory.is_dir():
            continue
        summary_exists = (directory / 'baseline_summary.json').exists()
        samples_exists = (directory / 'feature_samples.csv').exists()
        if not summary_exists and not samples_exists:
            continue
        if not summary_exists or not samples_exists:
            skipped.append({
                'session_directory': str(directory),
                'reason': 'incomplete session artifacts',
            })
            continue
        try:
            sessions.append(_load_session(directory))
        except ValueError as error:
            skipped.append({
                'session_directory': str(directory),
                'reason': str(error),
            })
    return root, sessions, skipped


def _aggregate_sessions(sessions):
    """Pool samples while keeping FPS and max-gap session-level."""
    session_values = {metric: [] for metric in SESSION_METRICS}
    sample_values = {metric: [] for metric in SAMPLE_METRICS}
    for session in sessions:
        for metric in SESSION_METRICS:
            value = session['session_metrics'][metric]
            if value is not None:
                session_values[metric].append(value)
        for metric in SAMPLE_METRICS:
            sample_values[metric].extend(session['sample_metrics'][metric])
    return {
        'session_count': len(sessions),
        'sample_count': sum(session['sample_count'] for session in sessions),
        'session_metrics': {
            metric: _descriptive_statistics(session_values[metric])
            for metric in SESSION_METRICS
        },
        'sample_metrics': {
            metric: _descriptive_statistics(sample_values[metric])
            for metric in SAMPLE_METRICS
        },
    }


def build_baseline_report(baseline_root, *, generated_at_epoch_sec=None):
    """Build global and per-scenario descriptive camera baseline statistics."""
    root, sessions, skipped = scan_baseline_sessions(baseline_root)
    grouped = {}
    for session in sessions:
        grouped.setdefault(session['scenario_label'], []).append(session)

    session_records = []
    for session in sessions:
        session_records.append({
            key: session[key]
            for key in (
                'scenario_label',
                'session_id',
                'session_directory',
                'sample_count',
                'session_metrics',
            )
        })
    return {
        'schema_version': 1,
        'description_only': True,
        'baseline_root_directory': str(root),
        'generated_at_utc': _utc_now_text(generated_at_epoch_sec),
        'scanned_session_count': len(sessions),
        'skipped_session_count': len(skipped),
        'skipped_sessions': skipped,
        'sessions': session_records,
        'global': _aggregate_sessions(sessions),
        'by_scenario_label': {
            label: _aggregate_sessions(grouped[label])
            for label in sorted(grouped)
        },
    }


def _format_number(value, precision=3):
    """Format a compact numeric table cell or a dash for missing data."""
    if value is None:
        return '-'
    return f'{value:.{precision}f}'


def format_report_table(report):
    """Return a concise global and per-scenario comparison table."""
    headers = (
        'scenario',
        'sessions',
        'samples',
        'fps_mean',
        'dt_p50',
        'dt_p95',
        'gap_max',
        'gray_mean',
        'lap_var',
        'edge',
        'entropy',
        'diff_mean',
    )

    def row(label, aggregate):
        session_metrics = aggregate['session_metrics']
        sample_metrics = aggregate['sample_metrics']
        return (
            label,
            str(aggregate['session_count']),
            str(aggregate['sample_count']),
            _format_number(session_metrics['observed_fps']['mean']),
            _format_number(sample_metrics['interarrival_sec']['p50'], 4),
            _format_number(sample_metrics['interarrival_sec']['p95'], 4),
            _format_number(session_metrics['max_gap_sec']['max'], 4),
            _format_number(sample_metrics['mean_gray']['mean']),
            _format_number(
                sample_metrics['laplacian_variance']['mean']
            ),
            _format_number(sample_metrics['edge_density']['mean'], 4),
            _format_number(sample_metrics['entropy']['mean']),
            _format_number(sample_metrics['frame_diff_mean']['mean']),
        )

    rows = [row('[global]', report['global'])]
    rows.extend(
        row(label, aggregate)
        for label, aggregate in report['by_scenario_label'].items()
    )
    widths = [
        max(len(headers[index]), *(len(item[index]) for item in rows))
        for index in range(len(headers))
    ]

    def format_row(items):
        return '  '.join(
            item.ljust(widths[index])
            for index, item in enumerate(items)
        )

    separator = '  '.join('-' * width for width in widths)
    return '\n'.join([
        format_row(headers),
        separator,
        *(format_row(items) for items in rows),
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


def write_baseline_report(
    baseline_root,
    *,
    output_path=None,
    generated_at_epoch_sec=None,
):
    """Build, persist, and return a descriptive baseline report."""
    report = build_baseline_report(
        baseline_root, generated_at_epoch_sec=generated_at_epoch_sec
    )
    if output_path is None:
        output_path = Path(baseline_root).expanduser() / 'baseline_report.json'
    else:
        output_path = Path(output_path).expanduser()
    _atomic_write_json(output_path, report)
    return report, output_path


def _argument_parser():
    """Create the command-line parser for the pure analysis tool."""
    parser = argparse.ArgumentParser(
        description=(
            'Aggregate descriptive camera baseline sessions. No thresholds '
            'or health classifications are generated.'
        )
    )
    parser.add_argument(
        'baseline_root',
        help='baseline root containing one subdirectory per session',
    )
    parser.add_argument(
        '--output',
        help='report path (default: <baseline_root>/baseline_report.json)',
    )
    return parser


def main(args=None):
    """Write baseline_report.json and print a compact comparison table."""
    parser = _argument_parser()
    options = parser.parse_args(args)
    try:
        report, output_path = write_baseline_report(
            options.baseline_root, output_path=options.output
        )
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(format_report_table(report))
    print(f'Wrote descriptive report: {output_path}')
    if report['skipped_session_count']:
        print(
            f'Skipped incomplete or invalid sessions: '
            f'{report["skipped_session_count"]}'
        )


if __name__ == '__main__':
    main()
