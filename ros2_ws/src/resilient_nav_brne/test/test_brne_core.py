"""Deterministic numerical contracts for the vendored BRNE core."""

import numpy as np
import pytest

from resilient_nav_brne import brne


def _trajectory_samples():
    """Return a small fixed two-agent, four-sample trajectory fixture."""
    robot_x = np.array([
        [0.00, 0.10, 0.20],
        [0.00, 0.09, 0.18],
        [0.00, 0.11, 0.22],
        [0.00, 0.08, 0.16],
    ])
    robot_y = np.array([
        [-0.10, -0.10, -0.10],
        [-0.03, -0.03, -0.03],
        [0.03, 0.03, 0.03],
        [0.10, 0.10, 0.10],
    ])
    pedestrian_x = np.array([
        [0.16, 0.16, 0.16],
        [0.18, 0.18, 0.18],
        [0.20, 0.20, 0.20],
        [0.22, 0.22, 0.22],
    ])
    pedestrian_y = np.array([
        [-0.06, -0.04, -0.02],
        [-0.02, 0.00, 0.02],
        [0.02, 0.04, 0.06],
        [0.06, 0.08, 0.10],
    ])
    return (
        np.vstack((robot_x, pedestrian_x)),
        np.vstack((robot_y, pedestrian_y)),
    )


def test_covariance_factor_is_finite_and_reconstructs_covariance():
    """Validate the Gaussian-process covariance and its Cholesky factor."""
    train_ts = np.array([0.0, 0.1, 0.2])
    test_ts = np.array([0.05, 0.15, 0.25, 0.35])
    factor, covariance = brne.get_Lmat_nb(
        train_ts,
        test_ts,
        np.full(train_ts.shape, 1e-3),
        kernel_a1=0.5,
        kernel_a2=0.2,
    )

    assert factor.shape == (4, 4)
    assert covariance.shape == (4, 4)
    assert np.isfinite(factor).all()
    assert np.isfinite(covariance).all()
    assert np.allclose(covariance, covariance.T)
    assert np.all(np.diag(factor) > 0.0)
    assert np.allclose(factor @ factor.T, covariance, atol=1e-10)


def test_trajectory_ensemble_contract_is_finite_and_deterministic():
    """Validate vectorized RK4 trajectory shape and the fixed straight motion."""
    states = np.zeros((3, 4))
    controls = np.zeros((3, 4, 2))
    controls[:, :, 0] = 0.2

    trajectory = brne.traj_sim_essemble(states, controls, dt=0.1)

    assert trajectory.shape == (3, 3, 4)
    assert np.isfinite(trajectory).all()
    assert np.allclose(trajectory[:, 0, :], [[0.02] * 4, [0.04] * 4, [0.06] * 4])
    assert np.allclose(trajectory[:, 1:, :], 0.0)


@pytest.mark.xfail(
    strict=True,
    raises=TypeError,
    reason='upstream traj_sim omits the required dt argument to dyn_step',
)
def test_upstream_scalar_trajectory_contract_is_currently_unusable():
    """Document, without changing, the pinned upstream scalar-helper defect."""
    brne.traj_sim(
        np.zeros(3),
        np.array([[0.2, 0.0], [0.2, 0.0]]),
        dt=0.1,
    )


def test_costs_and_weights_are_finite_with_mean_normalization():
    """Validate BRNE pairwise-cost and iterative-weight numerical contracts."""
    trajs_x, trajs_y = _trajectory_samples()
    costs = brne.costs_nb(
        trajs_x,
        trajs_y,
        num_agents=2,
        num_pts=4,
        tsteps=3,
        cost_a1=8.0,
        cost_a2=1.0,
        cost_a3=20.0,
    )
    weights = brne.brne_nav(
        trajs_x,
        trajs_y,
        num_agents=2,
        tsteps=3,
        num_pts=4,
        cost_a1=8.0,
        cost_a2=1.0,
        cost_a3=20.0,
        ped_sample_scale=1.0,
        y_min=-0.2,
        y_max=0.2,
    )

    assert costs.shape == (8, 8)
    assert np.isfinite(costs).all()
    assert np.all(costs >= 0.0)
    assert weights is not None
    assert weights.shape == (2, 4)
    assert np.isfinite(weights).all()
    assert np.all(weights > 0.0)
    assert np.allclose(weights.mean(axis=1), 1.0)


def test_robot_with_no_in_bounds_candidate_returns_none():
    """Validate the upstream no-safe-robot-sample sentinel contract."""
    trajs_x, trajs_y = _trajectory_samples()
    trajs_y[:4, :] = 1.0

    assert brne.brne_nav(
        trajs_x,
        trajs_y,
        num_agents=2,
        tsteps=3,
        num_pts=4,
        cost_a1=8.0,
        cost_a2=1.0,
        cost_a3=20.0,
        ped_sample_scale=1.0,
        y_min=-0.2,
        y_max=0.2,
    ) is None
