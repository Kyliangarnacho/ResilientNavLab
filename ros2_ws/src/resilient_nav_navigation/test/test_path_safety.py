"""Unit contracts for the Task 3 full-footprint path sweep."""

from __future__ import annotations

import math

from path_safety import LETHAL_COST, RawCostmapGrid, UNKNOWN_COST, full_footprint_path_sweep


def make_grid(size: int = 80, resolution: float = 0.1) -> RawCostmapGrid:
    return RawCostmapGrid(
        resolution=resolution,
        width=size,
        height=size,
        origin_x=0.0,
        origin_y=0.0,
        data=(0,) * (size * size),
    )


def set_cell(grid: RawCostmapGrid, column: int, row: int, value: int) -> RawCostmapGrid:
    values = list(grid.data)
    values[row * grid.width + column] = value
    return RawCostmapGrid(
        resolution=grid.resolution,
        width=grid.width,
        height=grid.height,
        origin_x=grid.origin_x,
        origin_y=grid.origin_y,
        data=tuple(values),
    )


def test_safe_straight_path_reports_sampling_geometry():
    evidence = full_footprint_path_sweep(
        make_grid(), [(2.0, 2.0), (3.0, 2.0)], initial_yaw=0.0, goal_yaw=0.0
    )
    assert evidence['safe'] is True
    assert evidence['sweep_pose_count'] >= 21
    assert evidence['translation_spacing_m'] == 0.05
    assert evidence['lethal_intersection_count'] == 0


def test_centerline_clear_but_full_footprint_lethal_is_rejected():
    # The centerline is y=2.0; the padded forward/side footprint reaches this
    # cell even though no origin sample does.
    grid = set_cell(make_grid(), 21, 21, LETHAL_COST)
    evidence = full_footprint_path_sweep(
        grid, [(2.0, 2.0), (3.0, 2.0)], initial_yaw=0.0, goal_yaw=0.0
    )
    assert evidence['safe'] is False
    assert evidence['first_violation']['lethal_cell_count'] > 0


def test_terminal_orientation_sweep_checks_asymmetric_rear_footprint():
    # This cell is free of the origin path and yaw=0 footprint, but intersects
    # the asymmetric rear of the robot during the required terminal rotation.
    grid = set_cell(make_grid(), 18, 16, LETHAL_COST)
    evidence = full_footprint_path_sweep(
        grid, [(2.0, 2.0)], initial_yaw=0.0, goal_yaw=math.pi / 2.0
    )
    assert evidence['safe'] is False
    assert evidence['first_violation']['lethal_cell_count'] > 0


def test_unknown_and_out_of_bounds_are_not_accepted_as_safe_space():
    unknown_grid = set_cell(make_grid(), 21, 21, UNKNOWN_COST)
    unknown = full_footprint_path_sweep(
        unknown_grid, [(2.0, 2.0), (3.0, 2.0)], initial_yaw=0.0, goal_yaw=0.0
    )
    out_of_bounds = full_footprint_path_sweep(
        make_grid(), [(0.1, 0.1), (1.0, 0.1)], initial_yaw=0.0, goal_yaw=0.0
    )
    assert unknown['safe'] is False
    assert unknown['first_violation']['unknown_cell_count'] > 0
    assert out_of_bounds['safe'] is False
    assert out_of_bounds['first_violation']['out_of_bounds'] is True
