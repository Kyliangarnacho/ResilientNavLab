"""Tests for the run-grouped physical-reliability RF trainer."""

import csv
import json

import joblib

import numpy as np

import pytest

from resilient_nav_fusion.physical_reliability_dataset import FEATURE_COLUMNS
from resilient_nav_fusion.physical_reliability_rf import (
    RandomForestConfig,
    SUPPORTED_TARGETS,
    load_split_config,
    select_validation_threshold,
    train_models,
)


def test_split_config_rejects_run_overlap(tmp_path):
    """A run must never leak across split boundaries."""
    split = tmp_path / 'split.json'
    split.write_text(json.dumps({
        'split_schema_version': 1,
        'train_runs': ['a'],
        'validation_runs': ['b'],
        'test_runs': ['a', 'c'],
    }))

    with pytest.raises(ValueError, match='overlap'):
        load_split_config(split, {'a', 'b', 'c'})


def test_threshold_selection_uses_balanced_accuracy():
    """The decision threshold maximizes validation balanced accuracy."""
    labels = np.asarray([0, 0, 1, 1, 1, 1])
    probability_reliable = np.asarray([0.2, 0.7, 0.6, 0.8, 0.9, 0.95])

    threshold, score = select_validation_threshold(
        labels,
        probability_reliable,
        RandomForestConfig(threshold_grid_step=0.05),
    )

    assert threshold == pytest.approx(0.70)
    assert score == pytest.approx(0.875)


def test_training_writes_three_models_and_probabilities(tmp_path):
    """Training emits three RFs and no learned LiDAR probability."""
    dataset = tmp_path / 'dataset'
    dataset.mkdir()
    manifest = {
        'dataset_schema_version': 2,
        'scan_matcher_algorithm': 'robust_point_to_line_v2',
        'feature_columns': list(FEATURE_COLUMNS),
        'label_columns': [*SUPPORTED_TARGETS, 'lidar_translation_reliable'],
        'training_forbidden_files': ['label_audit_gt_only.csv'],
    }
    (dataset / 'manifest.json').write_text(json.dumps(manifest))
    columns = [
        'window_id', 'run_id', 'window_start_sec', 'window_end_sec',
        *FEATURE_COLUMNS, *SUPPORTED_TARGETS, 'lidar_translation_reliable',
    ]
    rows = []
    for run_index, run_id in enumerate(('train', 'validation', 'test')):
        for sample in range(8):
            label = sample % 2
            row = {
                'window_id': f'{run_id}:{sample}',
                'run_id': run_id,
                'window_start_sec': sample * 0.2,
                'window_end_sec': sample * 0.2 + 0.4,
                **{
                    feature: float(label + run_index * 0.01 + index * 0.001)
                    for index, feature in enumerate(FEATURE_COLUMNS)
                },
                **{target: label for target in SUPPORTED_TARGETS},
                'lidar_translation_reliable': 1,
            }
            rows.append(row)
    with (dataset / 'windows.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)

    split = tmp_path / 'split.json'
    split.write_text(json.dumps({
        'split_schema_version': 1,
        'train_runs': ['train'],
        'validation_runs': ['validation'],
        'test_runs': ['test'],
    }))
    output = tmp_path / 'output'

    result = train_models(
        dataset,
        split,
        output,
        RandomForestConfig(n_estimators=10, max_depth=3, n_jobs=1),
    )

    assert result['manifest']['trained_targets'] == list(SUPPORTED_TARGETS)
    assert result['manifest']['excluded_target']['name'] == (
        'lidar_translation_reliable'
    )
    prediction_header = (
        output / 'predict_proba.csv'
    ).read_text().splitlines()[0]
    for target in SUPPORTED_TARGETS:
        model_path = output / 'models' / f'{target}.joblib'
        assert model_path.is_file()
        assert joblib.load(model_path)['target'] == target
        assert f'{target}_probability_reliable' in prediction_header
    assert 'lidar_translation_reliable_probability' not in prediction_header
