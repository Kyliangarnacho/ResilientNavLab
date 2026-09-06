# BRNE upstream provenance

This package intentionally contains the Python/Numba numerical core in
`resilient_nav_brne/brne.py` and a separated ROS shadow wrapper.  The core is
a verbatim copy of:

- Repository: <https://github.com/MurpheyLab/brne>
- Pinned revision: `633a5cdcb39ab27f18b596cb8cb1968644f82391`
- Source path: `brne_nav/brne_py/brne_py/brne.py`
- Retrieved for this task: 2026-09-02

The runtime algorithm profile is separately pinned to
`brne_nav/crowd_nav/config/brne.yaml` at the same revision.  The wrapper
defaults match its BRNE-specific values: `maximum_agents=5`, `num_samples=196`,
`dt=0.1`, `plan_steps=25`, `kernel_a1=0.2`, `kernel_a2=0.2`,
`cost_a1=15.0`, `cost_a2=3.0`, `cost_a3=20.0`, and
`ped_sample_scale=0.1`.  Robot velocity limits, geometry-derived stop distance,
goal tolerance, corridor bounds, and replan frequency remain ResilientNavLab
configuration rather than copied Unitree/environment settings.

`resilient_nav_brne/brne_shadow_node.py` borrows only the upstream wrapper's
numerical assembly pattern from `brne_nav/brne_py/brne_py/brne_nav.py`.  It is
an independent, intentionally reduced adaptation: its I/O is exclusively
`/brne/*`, its Path frame is `odom`, and it has no Unitree angular-drift
compensation, visualisation, action status, launch dependency, or hardware
logic.  The minimal Pedestrian messages are reproduced in the existing local
`resilient_nav_interfaces` package from the pinned upstream message contracts.

The upstream repository root `LICENSE` at the pinned revision is GNU GPL v3.
This package is therefore marked `GPL-3.0-only`.  The upstream
`brne_nav/brne_py/setup.py` and `package.xml` instead declare `Proprietary`;
that metadata conflicts with the repository-root license and is recorded here
without reinterpretation.  This personal experiment does not distribute the
copy; any later distribution requires resolving and complying with the
upstream licensing terms.
