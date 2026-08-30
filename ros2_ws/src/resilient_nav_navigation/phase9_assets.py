"""Frozen Phase 9 occupancy-map identities shared by Task 1 checks."""

from __future__ import annotations

import hashlib
from pathlib import Path

import yaml


FROZEN_ASSET_HASHES = {
    'occupancy/phase9_map.pgm': (
        '548a37d56084ca7a804c96341ef2784f45d22ec4772f5819ea4cd981fa8c3161'
    ),
    'occupancy/phase9_map.yaml': (
        '21499e0fcd079f11a276832ec4622bb1c69a0889dfa9cf7820c08b8c93e45161'
    ),
    'posegraph/phase9_posegraph.data': (
        '586c28fa47cc5def621eeaecd66e564861e10e6faad1b69c5368b1d60e018872'
    ),
    'posegraph/phase9_posegraph.posegraph': (
        'ee7152d9973ed87c26df2dd767fea495173941a9787a70c2cbb70dc6d74b6f20'
    ),
}

MAP_EXPECTED = {
    'frame_id': 'map',
    'resolution': 0.05000000074505806,
    'width': 227,
    'height': 226,
    'origin': (-2.222697386925495, -2.1610523495216207, 0.0),
}


def sha256(path: Path) -> str:
    """Return one stable content hash without changing the asset."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_frozen_assets(root: Path) -> dict[str, str]:
    """Fail closed if a source or installed Phase 9 asset drifted."""
    observed = {}
    for relative_path, expected_hash in FROZEN_ASSET_HASHES.items():
        observed_hash = sha256(root / relative_path)
        if observed_hash != expected_hash:
            raise ValueError(
                f'Phase 9 asset identity mismatch for {relative_path}: '
                f'{observed_hash} != {expected_hash}'
            )
        observed[relative_path] = observed_hash
    return observed


def map_yaml_metadata(map_yaml: Path) -> dict[str, object]:
    """Load only the saved-map metadata needed by the Map Server contract."""
    with map_yaml.open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError('map YAML must contain a mapping')
    return data
