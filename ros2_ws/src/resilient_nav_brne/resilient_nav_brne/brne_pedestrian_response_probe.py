"""Deterministic, ROS-free causal-response probe for the upstream BRNE core."""

from __future__ import annotations

import json
import math

import numpy as np

from . import brne
from .shadow_planner import BrneShadowPlanner


SEED = 1
ROBOT_POSE = (0.0, 0.0, 0.0)
GOAL = (2.0, 0.0)
SCENARIOS = {
    "clear_far": ((4.0, 2.0, 0.0, 0.0),),
    "stationary_left": ((0.8, 0.4, 0.0, 0.0),),
    "stationary_right": ((0.8, -0.4, 0.0, 0.0),),
    "crossing_from_left": ((0.8, 0.5, 0.0, -0.35),),
    "moving_away_left": ((0.8, 0.5, 0.0, 0.35),),
    "close_ahead": ((0.10, 0.0, 0.0, 0.0),),
}


def run_probe() -> dict[str, object]:
    """Run fixed relative-pedestrian cases, resetting upstream's global RNG each time."""
    cases: dict[str, dict[str, object]] = {}
    for name, pedestrians in SCENARIOS.items():
        brne.rng = np.random.default_rng(SEED)
        result = BrneShadowPlanner().plan(ROBOT_POSE, GOAL, pedestrians)
        terminal_pose = tuple(float(value) for value in result.trajectory[-1])
        values = (result.linear_velocity, result.angular_velocity, *terminal_pose)
        if not all(math.isfinite(value) for value in values):
            raise AssertionError(f"{name}: non-finite planner result {values!r}")
        cases[name] = {
            "pedestrians": pedestrians,
            # The upstream Numba reductions vary at sub-ULP level across calls;
            # retain a stable probe record without changing the planner output.
            "linear_x": round(result.linear_velocity, 12),
            "angular_z": round(result.angular_velocity, 12),
            "terminal_pose": tuple(round(value, 12) for value in terminal_pose),
        }

    _validate_causal_contract(cases)
    return {"seed": SEED, "robot_pose": ROBOT_POSE, "goal": GOAL, "cases": cases}


def _validate_causal_contract(cases: dict[str, dict[str, object]]) -> None:
    left = float(cases["stationary_left"]["angular_z"])
    right = float(cases["stationary_right"]["angular_z"])
    crossing_linear = float(cases["crossing_from_left"]["linear_x"])
    away_linear = float(cases["moving_away_left"]["linear_x"])
    close_linear = float(cases["close_ahead"]["linear_x"])
    close_angular = float(cases["close_ahead"]["angular_z"])

    if not left < 0.0 < right:
        raise AssertionError(f"left/right steering did not reverse: {left}, {right}")
    # With per-step mixed controls, the crossing case is causally more
    # conservative in forward speed than the otherwise matched moving-away case.
    if crossing_linear > away_linear + 1e-12:
        raise AssertionError(
            "crossing speed was not more conservative than moving-away: "
            f"{crossing_linear}, {away_linear}"
        )
    if close_linear != 0.0 or close_angular != 0.0:
        raise AssertionError(f"close-ahead stop contract violated: {close_linear}, {close_angular}")


def main() -> None:
    print(json.dumps(run_probe(), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
