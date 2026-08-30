"""Minimal, read-only probe for Nav2 localization lifecycle startup."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class LocalizationLifecycleProbe(Node):
    """Observe the manager-owned Map Server and AMCL lifecycle chain only."""

    def __init__(self) -> None:
        super().__init__('phase10_localization_lifecycle_probe')
        self.manager_client = self.create_client(
            Trigger, '/lifecycle_manager_localization/is_active'
        )
        self.state_clients = {
            'map_server': self.create_client(GetState, '/map_server/get_state'),
            'amcl': self.create_client(GetState, '/amcl/get_state'),
        }

    def manager_active(self) -> dict[str, object]:
        if not self.manager_client.service_is_ready():
            return {'service_ready': False, 'active': False}
        future = self.manager_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        if not future.done() or future.result() is None:
            return {'service_ready': True, 'active': False, 'response_received': False}
        response = future.result()
        return {
            'service_ready': True,
            'active': bool(response.success),
            'response_received': True,
            'message': response.message,
        }

    def states(self) -> dict[str, object]:
        observed: dict[str, object] = {}
        for name, client in self.state_clients.items():
            if not client.service_is_ready():
                observed[name] = {'service_ready': False, 'state_id': None}
                continue
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
            if not future.done() or future.result() is None:
                observed[name] = {'service_ready': True, 'state_id': None}
                continue
            state = future.result().current_state
            observed[name] = {
                'service_ready': True,
                'state_id': int(state.id),
                'state_label': state.label,
            }
        return observed


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-path', required=True)
    parser.add_argument('--timeout-sec', type=float, default=30.0)
    return parser.parse_known_args(argv)[0]


def main(argv: list[str] | None = None) -> int:
    arguments = parse_arguments(argv)
    if arguments.timeout_sec <= 0.0:
        raise SystemExit('--timeout-sec must be positive')
    output = Path(arguments.output_path)
    evidence: dict[str, object] = {'status': 'FAIL', 'timeout_wall_sec': arguments.timeout_sec}
    rclpy.init(args=None)
    probe = LocalizationLifecycleProbe()
    try:
        deadline = time.monotonic() + arguments.timeout_sec
        last_manager: dict[str, object] = {'service_ready': False, 'active': False}
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.1)
            last_manager = probe.manager_active()
            if last_manager.get('active'):
                states = probe.states()
                evidence.update({'manager': last_manager, 'states': states})
                if all(
                    state.get('state_id') == State.PRIMARY_STATE_ACTIVE
                    for state in states.values() if isinstance(state, dict)
                ):
                    evidence['status'] = 'PASS'
                    break
            # Low-frequency observer cadence; this never drives lifecycle
            # transitions and is not a startup delay.
            rclpy.spin_once(probe, timeout_sec=0.9)
        else:
            evidence['manager'] = last_manager
            evidence['states'] = probe.states()
            evidence['error'] = 'localization Lifecycle Manager did not activate Map Server and AMCL'
    finally:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(evidence, indent=2, sort_keys=True) + '\n', encoding='utf-8')
        probe.destroy_node()
        rclpy.shutdown()
    if evidence['status'] != 'PASS':
        print(json.dumps(evidence, indent=2, sort_keys=True), file=sys.stderr)
        return 1
    print(f'PASS: localization lifecycle evidence written to {output}')
    return 0
