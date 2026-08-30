"""Evaluate predeclared Local Costmap inflation profiles from read-only snapshots."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from pathlib import Path

import yaml

from costmap_contract import close


def load_json(path: Path) -> dict[str, object]:
    with path.open(encoding='utf-8') as stream:
        result = json.load(stream)
    if result.get('outcome') != 'PASS' or 'roi' not in result:
        raise ValueError(f'{path} is not a passing Local Costmap ROI snapshot')
    return result


def parse_snapshot_argument(value: str) -> tuple[str, str, Path]:
    """Parse profile:stage:path without accepting ambiguous experiment stages."""
    try:
        profile, stage, path = value.split(':', 2)
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            '--snapshot must be profile:before|marked|cleared:path'
        ) from error
    if stage not in ('before', 'marked', 'cleared') or not profile or not path:
        raise argparse.ArgumentTypeError(
            '--snapshot must be profile:before|marked|cleared:path'
        )
    return profile, stage, Path(path)


def snapshot_cells(snapshot: dict[str, object]) -> tuple[dict[tuple[int, int], int], float]:
    """Index ROI costs by resolution-aligned world coordinate."""
    roi = snapshot['roi']
    resolution = float(roi['resolution'])
    cells = {
        (round(float(x) / resolution), round(float(y) / resolution)): int(cost)
        for x, y, cost in roi['cells']
    }
    if not cells:
        raise ValueError('ROI snapshot contains no cells')
    return cells, resolution


def cell_position(key: tuple[int, int], resolution: float) -> tuple[float, float]:
    return key[0] * resolution, key[1] * resolution


def common_cells(
    before: dict[tuple[int, int], int], marked: dict[tuple[int, int], int]
) -> set[tuple[int, int]]:
    shared = set(before).intersection(marked)
    if len(shared) < 100:
        raise ValueError('too few common ROI cells between snapshots')
    return shared


def profile_metrics(
    before_snapshot: dict[str, object],
    marked_snapshot: dict[str, object],
    cleared_snapshot: dict[str, object],
    acceptance: dict[str, object],
) -> dict[str, object]:
    """Measure obstacle-derived cost geometry after subtracting the baseline ROI."""
    before, resolution = snapshot_cells(before_snapshot)
    marked, marked_resolution = snapshot_cells(marked_snapshot)
    cleared, cleared_resolution = snapshot_cells(cleared_snapshot)
    if not close(resolution, marked_resolution) or not close(resolution, cleared_resolution):
        raise ValueError('ROI resolutions differ across one profile')
    shared = common_cells(before, marked).intersection(cleared)
    if len(shared) < 100:
        raise ValueError('too few three-stage common ROI cells')
    lethal_threshold = int(acceptance['lethal_cost_threshold'])
    changed_tolerance = int(acceptance['changed_cost_tolerance'])
    changed = {
        key for key in shared
        if marked[key] > before[key] + changed_tolerance
    }
    lethal = {
        key for key in changed
        if marked[key] >= lethal_threshold
    }
    inflated = {
        key for key in changed
        if 0 < marked[key] < lethal_threshold
    }
    if not lethal or not inflated:
        raise ValueError('marked ROI has no new lethal and inflated cells')
    lethal_positions = [cell_position(key, resolution) for key in lethal]
    distances = {}
    for key in inflated:
        x, y = cell_position(key, resolution)
        distances[key] = min(
            math.hypot(x - lethal_x, y - lethal_y)
            for lethal_x, lethal_y in lethal_positions
        )
    annulus = [
        marked[key] for key, distance in distances.items()
        if float(acceptance['annulus_min_distance_m']) <= distance
        <= float(acceptance['annulus_max_distance_m'])
    ]
    if not annulus:
        raise ValueError('no inflated cells in the frozen comparison annulus')
    cleared_count = sum(
        abs(cleared[key] - before[key]) <= changed_tolerance for key in changed
    )
    roi_center = marked_snapshot['roi']['center']
    center_x = float(roi_center['x'])
    center_y = float(roi_center['y'])
    centre_key = min(
        shared,
        key=lambda key: math.hypot(
            cell_position(key, resolution)[0] - center_x,
            cell_position(key, resolution)[1] - center_y,
        ),
    )
    return {
        'resolution_m': resolution,
        'common_cell_count': len(shared),
        'new_changed_cell_count': len(changed),
        'new_lethal_cell_count': len(lethal),
        'new_inflated_cell_count': len(inflated),
        'new_inflated_area_m2': len(inflated) * resolution * resolution,
        'inflation_extent_from_new_lethal_m': max(distances.values()),
        'inflated_cost_sum': sum(marked[key] for key in inflated),
        'annulus_median_cost': statistics.median(annulus),
        'annulus_cell_count': len(annulus),
        'center_marked_cost': marked[centre_key],
        'center_cleared_cost': cleared[centre_key],
        'clearing_ratio': cleared_count / len(changed),
    }


def effective_profile(snapshot: dict[str, object]) -> dict[str, float]:
    parameters = snapshot['effective_parameters']
    return {
        'inflation_radius': float(parameters['inflation_layer.inflation_radius']),
        'cost_scaling_factor': float(
            parameters['inflation_layer.cost_scaling_factor']
        ),
    }


def validate_effective_profile(
    name: str, snapshot: dict[str, object], expected: dict[str, object]
) -> dict[str, float]:
    actual = effective_profile(snapshot)
    for key in ('inflation_radius', 'cost_scaling_factor'):
        if not close(actual[key], float(expected[key])):
            raise ValueError(
                f'{name} effective {key}={actual[key]} does not match frozen {expected[key]}'
            )
    return actual


def evaluate(
    profiles: dict[str, dict[str, object]],
    acceptance: dict[str, object],
    snapshots: dict[str, dict[str, dict[str, object]]],
) -> dict[str, object]:
    """Apply predeclared directional checks without run-derived thresholds."""
    expected_names = {'baseline', 'wider', 'steeper'}
    if set(profiles) != expected_names:
        raise ValueError(f'profile manifest must contain exactly {sorted(expected_names)}')
    metrics = {}
    effective = {}
    for name in sorted(expected_names):
        stages = snapshots.get(name, {})
        if set(stages) != {'before', 'marked', 'cleared'}:
            raise ValueError(f'{name} is missing before, marked, or cleared snapshot')
        effective[name] = validate_effective_profile(name, stages['marked'], profiles[name])
        metrics[name] = profile_metrics(
            stages['before'], stages['marked'], stages['cleared'], acceptance
        )
        if metrics[name]['center_marked_cost'] < int(acceptance['lethal_cost_threshold']):
            raise ValueError(f'{name} did not mark the controlled obstacle as lethal-like')
        if metrics[name]['clearing_ratio'] < float(acceptance['clearing_ratio_min']):
            raise ValueError(f'{name} did not clear the controlled obstacle sufficiently')

    baseline = metrics['baseline']
    wider = metrics['wider']
    steeper = metrics['steeper']
    if (
        wider['inflation_extent_from_new_lethal_m']
        < baseline['inflation_extent_from_new_lethal_m']
        + float(acceptance['wider_min_extent_increase_m'])
    ):
        raise ValueError('wider profile did not expand inflation extent as frozen')
    if wider['new_inflated_area_m2'] <= baseline['new_inflated_area_m2']:
        raise ValueError('wider profile did not increase inflated area')
    if abs(
        steeper['inflation_extent_from_new_lethal_m']
        - baseline['inflation_extent_from_new_lethal_m']
    ) > float(acceptance['same_radius_max_extent_difference_m']):
        raise ValueError('steeper profile changed same-radius inflation extent too much')
    if steeper['annulus_median_cost'] >= baseline['annulus_median_cost']:
        raise ValueError('steeper profile did not reduce shared-annulus median cost')
    if steeper['inflated_cost_sum'] >= baseline['inflated_cost_sum']:
        raise ValueError('steeper profile did not reduce inflated cost sum')
    return {
        'outcome': 'PASS',
        'effective_profiles': effective,
        'profile_metrics': metrics,
        'comparisons': {
            'wider_extent_increase_m': (
                wider['inflation_extent_from_new_lethal_m']
                - baseline['inflation_extent_from_new_lethal_m']
            ),
            'wider_area_increase_m2': (
                wider['new_inflated_area_m2'] - baseline['new_inflated_area_m2']
            ),
            'steeper_extent_difference_m': abs(
                steeper['inflation_extent_from_new_lethal_m']
                - baseline['inflation_extent_from_new_lethal_m']
            ),
            'steeper_annulus_median_cost_change': (
                steeper['annulus_median_cost'] - baseline['annulus_median_cost']
            ),
            'steeper_inflated_cost_sum_change': (
                steeper['inflated_cost_sum'] - baseline['inflated_cost_sum']
            ),
        },
    }


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profiles-file', required=True, type=Path)
    parser.add_argument('--snapshot', required=True, action='append', type=parse_snapshot_argument)
    parser.add_argument('--result-path', required=True, type=Path)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_arguments(argv)
    result: dict[str, object] = {'outcome': 'FAIL', 'evaluator': 'phase10_costmap_experiment'}
    try:
        with args.profiles_file.open(encoding='utf-8') as stream:
            manifest = yaml.safe_load(stream)
        snapshots: dict[str, dict[str, dict[str, object]]] = {}
        for profile, stage, path in args.snapshot:
            if stage in snapshots.setdefault(profile, {}):
                raise ValueError(f'duplicate {profile}:{stage} snapshot')
            snapshots[profile][stage] = load_json(path)
        result.update(evaluate(manifest['profiles'], manifest['acceptance'], snapshots))
    except Exception as error:
        result['error'] = str(error)
    finally:
        args.result_path.parent.mkdir(parents=True, exist_ok=True)
        args.result_path.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
    if result['outcome'] != 'PASS':
        print(json.dumps(result, indent=2), file=sys.stderr)
        return 1
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
