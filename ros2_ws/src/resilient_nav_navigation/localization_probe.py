"""Read-only Task 1 AMCL readiness and TF-continuity probe."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.msg import ParticleCloud
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from map_server_probe import validate_map


LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class LocalizationProbe(Node):
    """Observe readiness without publishing TF, motion, parameters, or truth."""

    def __init__(self) -> None:
        super().__init__('phase10_localization_probe')
        self.map_message: OccupancyGrid | None = None
        self.last_scan: LaserScan | None = None
        self.last_amcl_pose: PoseWithCovarianceStamped | None = None
        self.particle_count = 0
        self.clock_count = 0
        self.tf_edges = {
            'map_to_odom': 0,
            'odom_to_base_footprint': 0,
        }
        self.create_subscription(OccupancyGrid, '/map', self._on_map, LATCHED_QOS)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, LATCHED_QOS
        )
        self.create_subscription(
            ParticleCloud, '/particle_cloud', self._on_particles, qos_profile_sensor_data
        )
        self.create_subscription(Clock, '/clock', self._on_clock, qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf', self._on_tf, qos_profile_sensor_data)
        self.map_lifecycle = self.create_client(GetState, '/map_server/get_state')
        self.amcl_lifecycle = self.create_client(GetState, '/amcl/get_state')

    def _on_map(self, message: OccupancyGrid) -> None:
        self.map_message = message

    def _on_scan(self, message: LaserScan) -> None:
        self.last_scan = message

    def _on_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        self.last_amcl_pose = message

    def _on_particles(self, _: ParticleCloud) -> None:
        self.particle_count += 1

    def _on_clock(self, _: Clock) -> None:
        self.clock_count += 1

    def _on_tf(self, message: TFMessage) -> None:
        """Count the two authoritative dynamic TF edges without publishing one."""
        for transform in message.transforms:
            edge = (transform.header.frame_id, transform.child_frame_id)
            if edge == ('map', 'odom'):
                self.tf_edges['map_to_odom'] += 1
            elif edge == ('odom', 'base_footprint'):
                self.tf_edges['odom_to_base_footprint'] += 1

    def lifecycle_active(self, client) -> bool:
        if not client.service_is_ready():
            return False
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        return bool(
            future.done()
            and future.result() is not None
            and future.result().current_state.id == State.PRIMARY_STATE_ACTIVE
        )

def covariance_summary(message: PoseWithCovarianceStamped) -> dict[str, float]:
    """Reject non-finite values while preserving the full diagnostic diagonals."""
    values = tuple(message.pose.covariance)
    if len(values) != 36 or not all(math.isfinite(value) for value in values):
        raise ValueError('/amcl_pose covariance contains non-finite values')
    summary = {'x': values[0], 'y': values[7], 'yaw': values[35]}
    if any(value < 0.0 for value in summary.values()):
        raise ValueError('/amcl_pose covariance diagonal contains a negative value')
    return summary


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--observation-sec', type=float, default=10.0)
    parser.add_argument(
        '--allow-no-particles',
        action='store_true',
        help='For the no-motion smoke only; motion evidence still requires particles.',
    )
    parser.add_argument('--result-path', required=True)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_arguments(argv)
    if not math.isfinite(args.observation_sec) or args.observation_sec <= 0.0:
        raise SystemExit('--observation-sec must be finite and positive')
    result_path = Path(args.result_path)
    result: dict[str, object] = {'outcome': 'FAIL', 'probe': 'phase10_localization_probe'}
    rclpy.init(args=argv)
    probe = LocalizationProbe()
    try:
        deadline = time.monotonic() + args.observation_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.2)
        if not probe.lifecycle_active(probe.map_lifecycle):
            raise RuntimeError('map_server is not ACTIVE')
        if not probe.lifecycle_active(probe.amcl_lifecycle):
            raise RuntimeError('amcl is not ACTIVE')
        if probe.map_message is None:
            raise RuntimeError('no /map received')
        if probe.last_scan is None or probe.last_scan.header.frame_id != 'lidar_link':
            raise RuntimeError('no valid lidar_link /scan received')
        if probe.last_amcl_pose is None or probe.last_amcl_pose.header.frame_id != 'map':
            raise RuntimeError('no map-frame /amcl_pose received')
        if not args.allow_no_particles and probe.particle_count < 1:
            raise RuntimeError('no /particle_cloud received')
        if probe.clock_count < 2:
            raise RuntimeError('/clock did not advance')
        if any(count < 2 for count in probe.tf_edges.values()):
            raise RuntimeError('required dynamic TF ownership edges were not continuous')
        result.update({
            'outcome': 'PASS',
            'lifecycle': {'map_server': 'active', 'amcl': 'active'},
            'runtime_map': validate_map(probe.map_message),
            'scan_frame_id': probe.last_scan.header.frame_id,
            'particle_messages': probe.particle_count,
            'particles_required': not args.allow_no_particles,
            'clock_messages': probe.clock_count,
            'amcl_covariance_diagonal': covariance_summary(probe.last_amcl_pose),
            'tf_dynamic_edge_messages': probe.tf_edges,
            'map_to_base_footprint_chain': 'map -> odom -> base_footprint',
        })
    except Exception as error:
        result['error'] = str(error)
        result['tf_dynamic_edge_messages'] = probe.tf_edges
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
