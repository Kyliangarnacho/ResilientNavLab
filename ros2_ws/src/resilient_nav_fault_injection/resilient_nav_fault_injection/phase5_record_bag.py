import argparse
from datetime import datetime
import os
from pathlib import Path

import yaml


DEFAULT_TOPICS = [
    '/clock',
    '/imu/data',
    '/faulted/imu/data',
    '/wheel/odometry',
    '/faulted/wheel/odometry',
    '/scan',
    '/faulted/scan',
    '/fault_injection/status',
    '/odometry/filtered',
    '/odometry/faulted',
    '/tf',
    '/tf_static',
]


def scenario_id_from_file(path):
    if path is None:
        return 'phase5'
    with Path(path).open('r', encoding='utf-8') as scenario_stream:
        scenario = yaml.safe_load(scenario_stream) or {}
    for node_config in scenario.values():
        parameters = node_config.get('ros__parameters', node_config)
        scenario_id = parameters.get('scenario_id')
        if scenario_id:
            return str(scenario_id)
    return Path(path).stem


def unique_output_dir(output_root, scenario_id):
    root = Path(output_root)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    candidate = root / f'{scenario_id}_{timestamp}'
    suffix = 1
    while candidate.exists():
        candidate = root / f'{scenario_id}_{timestamp}_{suffix:02d}'
        suffix += 1
    return candidate


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario-file')
    parser.add_argument('--output-root', default='phase5_bags')
    parser.add_argument('topics', nargs='*')
    parsed = parser.parse_args(args)

    scenario_id = scenario_id_from_file(parsed.scenario_file)
    output_dir = unique_output_dir(parsed.output_root, scenario_id)
    output_dir.parent.mkdir(parents=True, exist_ok=True)
    topics = parsed.topics or DEFAULT_TOPICS

    command = ['ros2', 'bag', 'record', '-o', str(output_dir), *topics]
    print('phase5_record_bag output:', output_dir, flush=True)
    os.execvp(command[0], command)


__all__ = ['DEFAULT_TOPICS', 'main', 'scenario_id_from_file']
