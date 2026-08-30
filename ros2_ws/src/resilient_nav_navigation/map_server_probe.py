"""Read-only Task 1.3 Map Server lifecycle and metadata probe."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from phase9_assets import MAP_EXPECTED, map_yaml_metadata, verify_frozen_assets


MAP_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class MapServerProbe(Node):
    """Observe a Map Server without loading, saving, or changing its map."""

    def __init__(self) -> None:
        super().__init__('phase10_map_server_probe')
        self.map_message: OccupancyGrid | None = None
        self.create_subscription(OccupancyGrid, '/map', self._on_map, MAP_QOS)
        self.lifecycle_client = self.create_client(GetState, '/map_server/get_state')

    def _on_map(self, message: OccupancyGrid) -> None:
        self.map_message = message

    def lifecycle_active(self) -> bool | None:
        if not self.lifecycle_client.service_is_ready():
            return None
        future = self.lifecycle_client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if not future.done() or future.result() is None:
            return None
        return future.result().current_state.id == State.PRIMARY_STATE_ACTIVE


def _finite_close(actual: float, expected: float, tolerance: float = 1e-9) -> bool:
    return math.isfinite(actual) and abs(actual - expected) <= tolerance


def validate_map(message: OccupancyGrid) -> dict[str, object]:
    """Validate runtime metadata against the frozen Phase 9 map identity."""
    origin = message.info.origin
    expected_x, expected_y, expected_yaw = MAP_EXPECTED['origin']
    checks = {
        'frame_id': message.header.frame_id == MAP_EXPECTED['frame_id'],
        'resolution': _finite_close(message.info.resolution, MAP_EXPECTED['resolution']),
        'width': message.info.width == MAP_EXPECTED['width'],
        'height': message.info.height == MAP_EXPECTED['height'],
        'origin_x': _finite_close(origin.position.x, expected_x),
        'origin_y': _finite_close(origin.position.y, expected_y),
        'origin_z': _finite_close(origin.position.z, 0.0),
        'origin_qx': _finite_close(origin.orientation.x, 0.0),
        'origin_qy': _finite_close(origin.orientation.y, 0.0),
        'origin_qz': _finite_close(origin.orientation.z, expected_yaw),
        'origin_qw': _finite_close(origin.orientation.w, 1.0),
        'data_length': len(message.data) == message.info.width * message.info.height,
        'occupancy_values': all(-1 <= value <= 100 for value in message.data),
    }
    if not all(checks.values()):
        failed = ', '.join(name for name, passed in checks.items() if not passed)
        raise ValueError(f'/map failed frozen metadata checks: {failed}')
    return {
        'frame_id': message.header.frame_id,
        'resolution': message.info.resolution,
        'width': message.info.width,
        'height': message.info.height,
        'origin': {
            'x': origin.position.x,
            'y': origin.position.y,
            'z': origin.position.z,
            'yaw': expected_yaw,
        },
        'data_length': len(message.data),
    }


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout-sec', type=float, default=15.0)
    parser.add_argument('--result-path', required=True)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_arguments(argv)
    if not math.isfinite(args.timeout_sec) or args.timeout_sec <= 0.0:
        raise SystemExit('--timeout-sec must be finite and positive')

    slam_share = Path(get_package_share_directory('resilient_nav_slam'))
    map_root = slam_share / 'maps' / 'phase9'
    result_path = Path(args.result_path)
    result = {'outcome': 'FAIL', 'probe': 'phase10_map_server_probe'}
    rclpy.init(args=argv)
    probe = MapServerProbe()
    try:
        asset_hashes = verify_frozen_assets(map_root)
        map_yaml = map_root / 'occupancy' / 'phase9_map.yaml'
        metadata = map_yaml_metadata(map_yaml)
        deadline = time.monotonic() + args.timeout_sec
        lifecycle_active = False
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.2)
            state = probe.lifecycle_active()
            lifecycle_active = state is True
            if lifecycle_active and probe.map_message is not None:
                break
        if not lifecycle_active:
            raise RuntimeError('map_server did not reach ACTIVE lifecycle state')
        if probe.map_message is None:
            raise RuntimeError('did not receive a transient-local /map message')
        result.update({
            'outcome': 'PASS',
            'lifecycle_state': 'active',
            'installed_map_root': str(map_root),
            'asset_hashes': asset_hashes,
            'yaml_metadata': metadata,
            'runtime_map': validate_map(probe.map_message),
        })
    except Exception as error:  # Report a single auditable failure envelope.
        result['error'] = str(error)
    finally:
        result_path.parent.mkdir(parents=True, exist_ok=True)
        result_path.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
        probe.destroy_node()
        rclpy.shutdown()
    if result['outcome'] != 'PASS':
        print(json.dumps(result, indent=2), file=sys.stderr)
        return 1
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
