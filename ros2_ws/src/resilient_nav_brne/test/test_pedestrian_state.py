"""Deterministic contracts for the fixed Gazebo child-frame conversion."""

import math

import pytest

from resilient_nav_brne.pedestrian_state import state_in_odom


def test_child_frame_twist_is_rotated_then_transformed_into_demo_odom_frame():
    state = state_in_odom(
        world_x=-3.0,
        world_y=-4.0,
        world_yaw=math.pi / 2.0,
        twist_x=0.25,
        twist_y=0.0,
        odom_origin_world_x=-3.5,
        odom_origin_world_y=-3.5,
        odom_origin_world_yaw=0.0,
    )
    assert state is not None
    assert (state.x, state.y) == pytest.approx((0.5, -0.5))
    assert (state.velocity_x, state.velocity_y) == pytest.approx((0.0, 0.25))
    assert state.yaw == pytest.approx(math.pi / 2.0)


def test_nonfinite_child_frame_input_fails_closed():
    assert state_in_odom(
        world_x=0.0,
        world_y=0.0,
        world_yaw=0.0,
        twist_x=math.nan,
        twist_y=0.0,
        odom_origin_world_x=0.0,
        odom_origin_world_y=0.0,
        odom_origin_world_yaw=0.0,
    ) is None
