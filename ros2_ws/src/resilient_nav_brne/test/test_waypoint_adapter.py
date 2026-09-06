"""Focused unit and static contracts for the real-data BRNE input bridge."""

from pathlib import Path

import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from resilient_nav_brne.brne_shadow_input_adapter import BrneShadowInputAdapter
from resilient_nav_brne.waypoint_adapter import (
    select_local_waypoint,
    transform_points_se2,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ADAPTER_PATH = PACKAGE_ROOT / 'resilient_nav_brne' / 'brne_shadow_input_adapter.py'
SOURCE_PATH = PACKAGE_ROOT / 'resilient_nav_brne' / 'brne_synthetic_pedestrian_source.py'
LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'brne_shadow_real_data.launch.py'
SETUP_PATH = PACKAGE_ROOT / 'setup.py'


class _TimestampRecordingBuffer:
    """Return one finite TF while retaining the exact query time for assertion."""

    def __init__(self):
        self.request = None

    def lookup_transform(self, target_frame, source_frame, query_time, *, timeout):
        self.request = (target_frame, source_frame, query_time, timeout)
        transform = TransformStamped()
        transform.transform.rotation.w = 1.0
        return transform


class _MissingTimestampBuffer:
    """Record a stamped lookup which deliberately has no transform available."""

    def __init__(self):
        self.request = None

    def lookup_transform(self, target_frame, source_frame, query_time, *, timeout):
        self.request = (target_frame, source_frame, query_time, timeout)
        raise RuntimeError('requested timestamp is not in the TF buffer')


def test_map_path_is_transformed_into_odom_and_forward_waypoint_is_selected():
    """The adapter uses map-to-odom geometry and looks ahead along the Path."""
    transformed = transform_points_se2(
        [(1.0, 0.0), (1.4, 0.0), (1.8, 0.0), (2.2, 0.0)],
        translation_x=10.0,
        translation_y=-2.0,
        yaw=0.0,
    )
    assert np.allclose(transformed[0], (11.0, -2.0))
    selected = select_local_waypoint(
        transformed, robot_xy=(11.05, -2.0), lookahead_distance=0.75,
        maximum_nearest_distance=0.75,
    )
    assert selected is not None
    index, waypoint, nearest_distance = selected
    assert index == 2
    assert np.allclose(waypoint, (11.8, -2.0))
    assert nearest_distance < 0.1


def test_invalid_or_distant_path_is_rejected_without_guessing_a_goal():
    """Empty/non-finite/distant Path data must yield no isolated goal."""
    assert transform_points_se2([], 0.0, 0.0, 0.0) is None
    assert transform_points_se2([(float('nan'), 0.0)], 0.0, 0.0, 0.0) is None
    assert select_local_waypoint(
        [(10.0, 0.0)], robot_xy=(0.0, 0.0), lookahead_distance=0.8,
        maximum_nearest_distance=0.75,
    ) is None


def test_adapter_remains_a_read_only_bridge_with_explicit_fail_closed_checks():
    """Only isolated outputs are published after Path, TF, and stale validation."""
    source = ADAPTER_PATH.read_text(encoding='utf-8')
    assert "'/odometry/filtered'" in source
    assert "'/plan'" in source
    assert "'/brne/odom'" in source
    assert "'/brne/goal_pose'" in source
    assert "lookup_transform(" in source
    assert 'self.odom_frame, self.map_frame' in source
    assert 'Time.from_msg(self._odom.header.stamp)' in source
    assert 'self.odom_frame, self.map_frame, Time()' not in source
    assert "'stale odometry or Path'" in source
    assert "'/cmd_vel'" not in source
    assert 'Twist' not in source


def test_pedestrian_only_source_and_overlay_do_not_create_nav2_or_cmd_vel_owners():
    """The overlay retains the fixture while leaving Phase 10 and /cmd_vel alone."""
    source = SOURCE_PATH.read_text(encoding='utf-8')
    launch = LAUNCH_PATH.read_text(encoding='utf-8')
    setup = SETUP_PATH.read_text(encoding='utf-8')
    assert "'/brne/pedestrians'" in source
    assert "'/brne/odom'" not in source
    assert "'/brne/goal_pose'" not in source
    assert "'/cmd_vel'" not in source
    assert 'resilient_nav_navigation' not in launch
    assert "brne_shadow_input_adapter = resilient_nav_brne.brne_shadow_input_adapter:main" in setup
    assert "brne_synthetic_pedestrian_source = resilient_nav_brne.brne_synthetic_pedestrian_source:main" in setup


def test_adapter_queries_tf_at_the_selected_odom_stamp_without_latest_fallback():
    """The Path is transformed only with TF synchronized to its chosen odometry."""
    adapter = object.__new__(BrneShadowInputAdapter)
    adapter.odom_frame = 'odom'
    adapter.map_frame = 'map'
    adapter.tf_timeout_sec = 0.05
    adapter.last_failure = None
    adapter._odom = Odometry()
    adapter._odom.header.stamp.sec = 42
    adapter._odom.header.stamp.nanosec = 123_000_000
    buffer = _TimestampRecordingBuffer()
    adapter.tf_buffer = buffer

    assert adapter._map_to_odom_transform() is not None
    target_frame, source_frame, query_time, timeout = buffer.request
    assert (target_frame, source_frame) == ('odom', 'map')
    assert query_time.nanoseconds == 42_123_000_000
    assert timeout.nanoseconds == 50_000_000


def test_missing_timestamped_tf_fails_closed_without_a_latest_transform_fallback():
    """An unavailable historical transform produces no substitute geometry."""
    adapter = object.__new__(BrneShadowInputAdapter)
    adapter.odom_frame = 'odom'
    adapter.map_frame = 'map'
    adapter.tf_timeout_sec = 0.05
    adapter.last_failure = None
    adapter._odom = Odometry()
    adapter._odom.header.stamp.sec = 42
    adapter._odom.header.stamp.nanosec = 123_000_000
    buffer = _MissingTimestampBuffer()
    adapter.tf_buffer = buffer

    assert adapter._map_to_odom_transform() is None
    assert buffer.request[2].nanoseconds == 42_123_000_000
    assert adapter.last_failure == 'map-to-odom TF unavailable: RuntimeError'
