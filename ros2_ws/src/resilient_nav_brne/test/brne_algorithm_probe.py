"""Measure the actual BRNE closed-loop runtime profile in a fresh process."""

import json
import time

import numpy as np

from resilient_nav_brne.shadow_planner import BrneShadowPlanner, ShadowPlannerConfig


RUNTIME_CONFIG = ShadowPlannerConfig()
RUNTIME_ROBOT_POSE = (0.0, 0.0, 0.0)
RUNTIME_GOAL = (0.8, 0.0)
RUNTIME_PEDESTRIANS = ((0.5, -1.0, 0.0, 0.25),)


def _call(planner: BrneShadowPlanner):
    """Exercise the same complete planner call used after node warm-up."""
    return planner.plan(
        robot_pose=RUNTIME_ROBOT_POSE,
        goal=RUNTIME_GOAL,
        pedestrians=RUNTIME_PEDESTRIANS,
    )


def main():
    """Print cold warm-up and warm planning latency for the runtime profile."""
    planner = BrneShadowPlanner(RUNTIME_CONFIG)
    start = time.perf_counter_ns()
    warm_up_result = planner.warm_up()
    cold_warmup_ms = (time.perf_counter_ns() - start) / 1_000_000.0

    warm_calls = 50
    warm_samples_ms = []
    for _ in range(warm_calls):
        start = time.perf_counter_ns()
        warm_result = _call(planner)
        warm_samples_ms.append((time.perf_counter_ns() - start) / 1_000_000.0)
    warm_total_ms = sum(warm_samples_ms)

    assert warm_up_result is not None
    assert warm_result is not None
    assert np.isfinite(warm_result.trajectory).all()
    assert 0.0 <= warm_result.linear_velocity <= RUNTIME_CONFIG.max_linear_velocity
    assert abs(warm_result.angular_velocity) <= RUNTIME_CONFIG.max_angular_velocity
    print(json.dumps({
        'runtime_profile': {
            'maximum_agents': RUNTIME_CONFIG.maximum_agents,
            'num_samples': RUNTIME_CONFIG.num_samples,
            'dt': RUNTIME_CONFIG.dt,
            'plan_steps': RUNTIME_CONFIG.plan_steps,
            'kernel_a1': RUNTIME_CONFIG.kernel_a1,
            'kernel_a2': RUNTIME_CONFIG.kernel_a2,
            'cost_a1': RUNTIME_CONFIG.cost_a1,
            'cost_a2': RUNTIME_CONFIG.cost_a2,
            'cost_a3': RUNTIME_CONFIG.cost_a3,
            'ped_sample_scale': RUNTIME_CONFIG.pedestrian_sample_scale,
        },
        'agents_in_measurement': 2,
        'control_period_ms': 200.0,
        'cold_warmup_ms': round(cold_warmup_ms, 3),
        'warm_calls': warm_calls,
        'warm_total_ms': round(warm_total_ms, 3),
        'warm_mean_ms': round(warm_total_ms / warm_calls, 3),
        'warm_p50_ms': round(float(np.percentile(warm_samples_ms, 50)), 3),
        'warm_p95_ms': round(float(np.percentile(warm_samples_ms, 95)), 3),
        'warm_max_ms': round(float(np.max(warm_samples_ms)), 3),
        'trajectory_shape': list(warm_result.trajectory.shape),
        'trajectory_finite': bool(np.isfinite(warm_result.trajectory).all()),
    }, sort_keys=True))


if __name__ == '__main__':
    main()
