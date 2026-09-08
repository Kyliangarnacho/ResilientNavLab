"""Train reproducible run-grouped Random Forest reliability models.

Only the schema-v2 runtime feature allowlist and independent-GT-derived label
columns in ``windows.csv`` are consumed.  The GT-only label audit is never
opened.  LiDAR reliability remains governed by the runtime ICP quality and
observability gate and is intentionally excluded from model training.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import platform
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import joblib

import numpy as np

import sklearn
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import (
    accuracy_score,
    average_precision_score,
    balanced_accuracy_score,
    brier_score_loss,
    classification_report,
    confusion_matrix,
    roc_auc_score,
)

from .physical_reliability_dataset import DATASET_SCHEMA_VERSION


TRAINING_SCHEMA_VERSION = 1
SUPPORTED_TARGETS = (
    'wheel_translation_reliable',
    'wheel_rotation_reliable',
    'imu_yaw_rate_reliable',
)
EXCLUDED_TARGET = 'lidar_translation_reliable'
FORBIDDEN_FEATURE_TOKENS = (
    'ground_truth',
    'scenario',
    'fault',
    'parameters_yaml',
)
SPLIT_KEYS = ('train', 'validation', 'test')


@dataclass(frozen=True)
class RandomForestConfig:
    """Fixed first-version model and decision contract."""

    n_estimators: int = 500
    criterion: str = 'gini'
    max_depth: int | None = 12
    min_samples_split: int = 4
    min_samples_leaf: int = 2
    max_features: str = 'sqrt'
    bootstrap: bool = True
    class_weight: str = 'balanced_subsample'
    random_state: int = 42
    n_jobs: int = 1
    decision_threshold: float = 0.5
    threshold_grid_min: float = 0.05
    threshold_grid_max: float = 0.95
    threshold_grid_step: float = 0.005

    def estimator_parameters(self) -> dict[str, Any]:
        """Return only arguments accepted by RandomForestClassifier."""
        values = asdict(self)
        for name in (
            'decision_threshold',
            'threshold_grid_min',
            'threshold_grid_max',
            'threshold_grid_step',
        ):
            values.pop(name)
        return values


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, indent=2, ensure_ascii=False, sort_keys=True) + '\n',
        encoding='utf-8',
    )


def _write_csv(
    path: Path,
    rows: list[dict[str, Any]],
    columns: Iterable[str],
) -> None:
    with path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(columns))
        writer.writeheader()
        writer.writerows(rows)


def load_split_config(
    path: Path,
    observed_runs: set[str],
) -> dict[str, list[str]]:
    """Load and fully validate a complete, disjoint run assignment."""
    raw = json.loads(Path(path).read_text(encoding='utf-8'))
    if raw.get('split_schema_version') != 1:
        raise ValueError('split_schema_version must be 1')
    splits = {
        'train': list(raw.get('train_runs', [])),
        'validation': list(raw.get('validation_runs', [])),
        'test': list(raw.get('test_runs', [])),
    }
    if any(not splits[key] for key in SPLIT_KEYS):
        raise ValueError('train, validation, and test must each contain runs')
    assigned = [run for key in SPLIT_KEYS for run in splits[key]]
    duplicates = sorted({run for run in assigned if assigned.count(run) > 1})
    if duplicates:
        raise ValueError(f'run split overlap: {duplicates}')
    missing = sorted(observed_runs - set(assigned))
    unknown = sorted(set(assigned) - observed_runs)
    if missing or unknown:
        raise ValueError(
            f'run split must cover the dataset exactly; missing={missing}, '
            f'unknown={unknown}'
        )
    return splits


def _load_dataset(dataset_dir: Path) -> tuple[
    dict[str, Any],
    list[dict[str, str]],
    list[str],
]:
    """Read only the declared manifest and windows table."""
    manifest_path = dataset_dir / 'manifest.json'
    windows_path = dataset_dir / 'windows.csv'
    if not manifest_path.is_file() or not windows_path.is_file():
        raise FileNotFoundError(
            'dataset requires manifest.json and windows.csv'
        )
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    if manifest.get('dataset_schema_version') != DATASET_SCHEMA_VERSION:
        raise ValueError(
            f'expected dataset schema {DATASET_SCHEMA_VERSION}, got '
            f'{manifest.get("dataset_schema_version")}'
        )
    forbidden_files = set(manifest.get('training_forbidden_files', []))
    if 'label_audit_gt_only.csv' not in forbidden_files:
        raise ValueError(
            'dataset must explicitly forbid GT-only audit training'
        )
    feature_columns = list(manifest.get('feature_columns', []))
    if (
        not feature_columns
        or len(feature_columns) != len(set(feature_columns))
    ):
        raise ValueError(
            'manifest feature_columns must be non-empty and unique'
        )
    normalized = ' '.join(feature_columns).lower()
    for token in FORBIDDEN_FEATURE_TOKENS:
        if token in normalized:
            raise ValueError(f'forbidden feature token: {token}')
    declared_labels = set(manifest.get('label_columns', []))
    required_labels = set(SUPPORTED_TARGETS) | {EXCLUDED_TARGET}
    if not required_labels.issubset(declared_labels):
        raise ValueError('dataset is missing required reliability labels')

    with windows_path.open('r', encoding='utf-8', newline='') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError('windows.csv is empty')
    required_columns = {
        'window_id', 'run_id', 'window_start_sec', 'window_end_sec',
        *feature_columns, *required_labels,
    }
    missing_columns = sorted(required_columns - set(rows[0]))
    if missing_columns:
        raise ValueError(f'windows.csv missing columns: {missing_columns}')
    if len({row['window_id'] for row in rows}) != len(rows):
        raise ValueError('window_id must be unique')
    return manifest, rows, feature_columns


def _matrix(
    rows: list[dict[str, str]],
    feature_columns: list[str],
) -> np.ndarray:
    values = np.asarray([
        [float(row[feature]) for feature in feature_columns]
        for row in rows
    ], dtype=np.float64)
    if values.ndim != 2 or not np.all(np.isfinite(values)):
        raise ValueError('all training features must be finite')
    return values


def _labels(rows: list[dict[str, str]], target: str) -> np.ndarray:
    values = np.asarray([int(row[target]) for row in rows], dtype=np.int64)
    if not set(values.tolist()).issubset({0, 1}):
        raise ValueError(f'{target} labels must be binary 0/1')
    return values


def _probability_columns(
    estimator: RandomForestClassifier,
    features: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    probabilities = estimator.predict_proba(features)
    class_index = {
        int(value): index
        for index, value in enumerate(estimator.classes_)
    }
    if set(class_index) != {0, 1}:
        raise ValueError('training data must contain both reliability classes')
    return probabilities[:, class_index[0]], probabilities[:, class_index[1]]


def _evaluate(
    labels: np.ndarray,
    predicted: np.ndarray,
    probability_unreliable: np.ndarray,
    probability_reliable: np.ndarray,
) -> dict[str, Any]:
    report = classification_report(
        labels,
        predicted,
        labels=[0, 1],
        target_names=['unreliable', 'reliable'],
        output_dict=True,
        zero_division=0,
    )
    matrix = confusion_matrix(labels, predicted, labels=[0, 1])
    has_both_classes = np.unique(labels).size == 2
    return {
        'window_count': int(labels.size),
        'class_support': {
            'unreliable': int(np.count_nonzero(labels == 0)),
            'reliable': int(np.count_nonzero(labels == 1)),
        },
        'accuracy': float(accuracy_score(labels, predicted)),
        'balanced_accuracy': float(balanced_accuracy_score(labels, predicted)),
        'roc_auc_reliable': (
            float(roc_auc_score(labels, probability_reliable))
            if has_both_classes else None
        ),
        'average_precision_unreliable': (
            float(average_precision_score(labels == 0, probability_unreliable))
            if has_both_classes else None
        ),
        'brier_score_reliable': float(
            brier_score_loss(labels, probability_reliable)
        ),
        'classification_report': report,
        'confusion_matrix': {
            'labels': [0, 1],
            'label_names': ['unreliable', 'reliable'],
            'rows': 'true labels',
            'columns': 'predicted labels',
            'matrix': matrix.astype(int).tolist(),
        },
    }


def select_validation_threshold(
    labels: np.ndarray,
    probability_reliable: np.ndarray,
    config: RandomForestConfig,
) -> tuple[float, float]:
    """Select a threshold using validation balanced accuracy only."""
    if np.unique(labels).size != 2:
        return config.decision_threshold, float('nan')
    candidates = np.arange(
        config.threshold_grid_min,
        config.threshold_grid_max + config.threshold_grid_step * 0.5,
        config.threshold_grid_step,
    )
    candidates = np.unique(np.append(candidates, config.decision_threshold))
    best: tuple[float, float, float] | None = None
    for threshold in candidates:
        predicted = (probability_reliable >= threshold).astype(np.int64)
        score = float(balanced_accuracy_score(labels, predicted))
        # Prefer a threshold near 0.5 on ties; then prefer the conservative
        # higher reliability threshold, which rejects more doubtful samples.
        candidate = (
            score,
            -abs(float(threshold) - config.decision_threshold),
            float(threshold),
        )
        if best is None or candidate > best:
            best = candidate
    assert best is not None
    return best[2], best[0]


def _class_counts(
    rows: list[dict[str, str]],
    target: str,
) -> dict[str, int]:
    values = _labels(rows, target)
    return {
        'unreliable': int(np.count_nonzero(values == 0)),
        'reliable': int(np.count_nonzero(values == 1)),
    }


def train_models(
    dataset_dir: Path,
    split_config_path: Path,
    output_dir: Path,
    config: RandomForestConfig | None = None,
) -> dict[str, Any]:
    """Train three RFs and persist models, probabilities, and evaluations."""
    config = config or RandomForestConfig()
    dataset_dir = Path(dataset_dir).resolve()
    split_config_path = Path(split_config_path).resolve()
    output_dir = Path(output_dir).resolve()
    if output_dir.exists():
        raise FileExistsError(f'refusing to overwrite existing {output_dir}')

    dataset_manifest, rows, feature_columns = _load_dataset(dataset_dir)
    observed_runs = {row['run_id'] for row in rows}
    splits = load_split_config(split_config_path, observed_runs)
    run_to_split = {
        run: split for split, runs in splits.items() for run in runs
    }
    split_rows = {
        split: [row for row in rows if run_to_split[row['run_id']] == split]
        for split in SPLIT_KEYS
    }
    train_matrix = _matrix(split_rows['train'], feature_columns)

    output_dir.mkdir(parents=True)
    model_dir = output_dir / 'models'
    model_dir.mkdir()
    metrics: dict[str, Any] = {
        'metrics_schema_version': 1,
        'threshold_selection': (
            'Per target, maximize balanced accuracy on validation only; '
            'test is never used for threshold selection.'
        ),
        'decision_threshold': {},
        'probability_note': (
            'Raw RandomForestClassifier predict_proba; not '
            'probability-calibrated.'
        ),
        'targets': {},
    }
    prediction_rows: list[dict[str, Any]] = [
        {
            'window_id': row['window_id'],
            'run_id': row['run_id'],
            'split': run_to_split[row['run_id']],
            'window_start_sec': row['window_start_sec'],
            'window_end_sec': row['window_end_sec'],
        }
        for row in rows
    ]
    row_index = {row['window_id']: index for index, row in enumerate(rows)}
    importance_rows: list[dict[str, Any]] = []
    metric_rows: list[dict[str, Any]] = []
    confusion_rows: list[dict[str, Any]] = []
    model_files: dict[str, str] = {}

    for target in SUPPORTED_TARGETS:
        train_labels = _labels(split_rows['train'], target)
        if np.unique(train_labels).size != 2:
            raise ValueError(f'{target} train split needs both classes')
        estimator = RandomForestClassifier(**config.estimator_parameters())
        estimator.fit(train_matrix, train_labels)

        validation_matrix = _matrix(split_rows['validation'], feature_columns)
        validation_labels = _labels(split_rows['validation'], target)
        _, validation_probability_1 = _probability_columns(
            estimator, validation_matrix
        )
        decision_threshold, threshold_score = select_validation_threshold(
            validation_labels,
            validation_probability_1,
            config,
        )
        metrics['decision_threshold'][target] = {
            'value': decision_threshold,
            'validation_balanced_accuracy': threshold_score,
        }

        target_metrics: dict[str, Any] = {}
        for split in SPLIT_KEYS:
            current_rows = split_rows[split]
            current_matrix = _matrix(current_rows, feature_columns)
            current_labels = _labels(current_rows, target)
            probability_0, probability_1 = _probability_columns(
                estimator, current_matrix
            )
            predicted = (
                probability_1 >= decision_threshold
            ).astype(np.int64)
            evaluation = _evaluate(
                current_labels, predicted, probability_0, probability_1
            )
            target_metrics[split] = evaluation

            report = evaluation['classification_report']
            metric_rows.append({
                'target': target,
                'split': split,
                'window_count': evaluation['window_count'],
                'unreliable_support': (
                    evaluation['class_support']['unreliable']
                ),
                'reliable_support': evaluation['class_support']['reliable'],
                'accuracy': evaluation['accuracy'],
                'balanced_accuracy': evaluation['balanced_accuracy'],
                'unreliable_precision': report['unreliable']['precision'],
                'unreliable_recall': report['unreliable']['recall'],
                'unreliable_f1': report['unreliable']['f1-score'],
                'reliable_precision': report['reliable']['precision'],
                'reliable_recall': report['reliable']['recall'],
                'reliable_f1': report['reliable']['f1-score'],
                'macro_f1': report['macro avg']['f1-score'],
                'roc_auc_reliable': evaluation['roc_auc_reliable'],
                'average_precision_unreliable': (
                    evaluation['average_precision_unreliable']
                ),
                'brier_score_reliable': evaluation['brier_score_reliable'],
            })
            cm = evaluation['confusion_matrix']['matrix']
            confusion_rows.append({
                'target': target,
                'split': split,
                'true_unreliable_pred_unreliable': cm[0][0],
                'true_unreliable_pred_reliable': cm[0][1],
                'true_reliable_pred_unreliable': cm[1][0],
                'true_reliable_pred_reliable': cm[1][1],
            })
            for local_index, row in enumerate(current_rows):
                destination = prediction_rows[row_index[row['window_id']]]
                destination[f'{target}_true'] = int(
                    current_labels[local_index]
                )
                destination[f'{target}_predicted'] = int(
                    predicted[local_index]
                )
                destination[f'{target}_probability_unreliable'] = float(
                    probability_0[local_index]
                )
                destination[f'{target}_probability_reliable'] = float(
                    probability_1[local_index]
                )

        ranked = sorted(
            zip(feature_columns, estimator.feature_importances_),
            key=lambda item: (-float(item[1]), item[0]),
        )
        for rank, (feature, importance) in enumerate(ranked, start=1):
            importance_rows.append({
                'target': target,
                'rank': rank,
                'feature': feature,
                'importance': float(importance),
            })

        model_name = f'{target}.joblib'
        artifact = {
            'artifact_schema_version': 1,
            'target': target,
            'class_semantics': {0: 'unreliable', 1: 'reliable'},
            'feature_columns': feature_columns,
            'decision_threshold': decision_threshold,
            'dataset_schema_version': DATASET_SCHEMA_VERSION,
            'dataset_windows_sha256': _sha256(dataset_dir / 'windows.csv'),
            'probability_contract': (
                'predict_proba class 1 is raw reliability probability; '
                'not calibrated'
            ),
            'estimator': estimator,
        }
        joblib.dump(artifact, model_dir / model_name, compress=3)
        model_files[target] = str(Path('models') / model_name)
        metrics['targets'][target] = target_metrics

    metric_columns = list(metric_rows[0])
    confusion_columns = list(confusion_rows[0])
    prediction_columns = list(prediction_rows[0])
    _write_csv(
        output_dir / 'classification_metrics.csv',
        metric_rows,
        metric_columns,
    )
    _write_csv(
        output_dir / 'confusion_matrices.csv',
        confusion_rows,
        confusion_columns,
    )
    _write_csv(
        output_dir / 'feature_importance.csv',
        importance_rows,
        ('target', 'rank', 'feature', 'importance'),
    )
    _write_csv(
        output_dir / 'predict_proba.csv',
        prediction_rows,
        prediction_columns,
    )
    _write_csv(
        output_dir / 'split_assignments.csv',
        [
            {'run_id': run, 'split': split}
            for split in SPLIT_KEYS for run in splits[split]
        ],
        ('run_id', 'split'),
    )
    _write_json(output_dir / 'metrics.json', metrics)

    manifest = {
        'training_schema_version': TRAINING_SCHEMA_VERSION,
        'training_status': 'COMPLETE',
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'dataset': {
            'directory': str(dataset_dir),
            'dataset_schema_version': (
                dataset_manifest['dataset_schema_version']
            ),
            'scan_matcher_algorithm': dataset_manifest.get(
                'scan_matcher_algorithm'
            ),
            'windows_sha256': _sha256(dataset_dir / 'windows.csv'),
            'manifest_sha256': _sha256(dataset_dir / 'manifest.json'),
            'window_count': len(rows),
            'feature_count': len(feature_columns),
        },
        'split': {
            'contract': 'disjoint complete runs; no window-level random split',
            'config_path': str(split_config_path),
            'config_sha256': _sha256(split_config_path),
            'runs': splits,
            'window_count': {
                split: len(split_rows[split]) for split in SPLIT_KEYS
            },
            'class_count': {
                target: {
                    split: _class_counts(split_rows[split], target)
                    for split in SPLIT_KEYS
                }
                for target in SUPPORTED_TARGETS
            },
        },
        'features': feature_columns,
        'trained_targets': list(SUPPORTED_TARGETS),
        'excluded_target': {
            'name': EXCLUDED_TARGET,
            'reason': 'No unreliable LiDAR examples in schema-v2 dataset.',
            'runtime_policy': 'Keep ICP quality plus observability gate.',
        },
        'random_forest': {
            **asdict(config),
            'selected_decision_threshold': {
                target: metrics['decision_threshold'][target]['value']
                for target in SUPPORTED_TARGETS
            },
            'probability_contract': (
                'class 1 predict_proba is reliability; raw and uncalibrated'
            ),
        },
        'software': {
            'python': platform.python_version(),
            'numpy': np.__version__,
            'scikit_learn': sklearn.__version__,
            'joblib': joblib.__version__,
        },
        'model_files': model_files,
        'outputs': {
            'classification_metrics': 'classification_metrics.csv',
            'confusion_matrices': 'confusion_matrices.csv',
            'feature_importance': 'feature_importance.csv',
            'predict_proba': 'predict_proba.csv',
            'metrics': 'metrics.json',
            'split_assignments': 'split_assignments.csv',
        },
        'ground_truth_boundary': (
            'GT-derived binary labels are read from windows.csv for '
            'supervised training. label_audit_gt_only.csv is never opened. '
            'Runtime model inputs are restricted to the manifest feature '
            'allowlist.'
        ),
    }
    manifest['model_sha256'] = {
        target: _sha256(output_dir / relative_path)
        for target, relative_path in model_files.items()
    }
    _write_json(output_dir / 'manifest.json', manifest)
    return {'manifest': manifest, 'metrics': metrics}


def main(argv: list[str] | None = None) -> None:
    """Run the offline RF training command."""
    parser = argparse.ArgumentParser(
        description='Train run-grouped schema-v2 physical-reliability RFs.'
    )
    parser.add_argument('--dataset-dir', required=True, type=Path)
    parser.add_argument('--split-config', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    arguments = parser.parse_args(argv)
    result = train_models(
        arguments.dataset_dir,
        arguments.split_config,
        arguments.output_dir,
    )
    compact = {
        target: {
            split: {
                'balanced_accuracy': values['balanced_accuracy'],
                'unreliable_f1': (
                    values['classification_report']['unreliable']['f1-score']
                ),
                'confusion_matrix': values['confusion_matrix']['matrix'],
            }
            for split, values in result['metrics']['targets'][target].items()
        }
        for target in SUPPORTED_TARGETS
    }
    print(json.dumps(compact, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
