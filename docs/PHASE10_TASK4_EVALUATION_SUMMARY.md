# Phase 10 Task 4 — Final Evaluation Summary

## Closure decision

**CLOSED — engineering accepted with a known infrastructure limitation**
(2026-08-30).

Task 4's GT-isolated healthy-navigation benchmark architecture, offline
evaluator, fresh-process trial isolation, condition-based teardown barrier,
and evidence retention are accepted. This is an engineering acceptance of the
healthy baseline, not a claim that the strict automated 9/9 contract passed.
No further Task 4 dynamic verification is planned.

## Final evidence set

The final manual fresh-process set is retained in
`docs/phase10_task4_batch_manual_20260830_r02/`:

- `manifest.json` and `benchmark_summary.json`;
- one `result.json` for each of the nine declared logical trials;
- isolated `GZ_PARTITION` and `ROS_DOMAIN_ID`, plus process/evidence teardown
  records for every trial.

## Verdict

- **8/8 valid navigation trials PASS.**
- `simple_reachable`: **3/3 PASS**.
- `static_obstacle_detour`: **3/3 PASS**.
- `multi_turn_healthy`: **2 valid PASS**, with frozen initial geometry
  preserved (`7.5349 m`, two turns); **1 pre-goal infrastructure-invalid**
  trial (`r03`).
- The strict automated **9/9** contract is therefore technically **not met**.

`multi_turn_healthy-r03` failed before a `NavigateToPose` goal was sent:
the runner's `localization_tf` readiness read saw a past TF extrapolation
(`requested 0.300 s`, earliest available `47.333 s`). Its record is classified
as `infrastructure_localization_tf`, not as a navigation or evaluator
failure. The corresponding teardown record is complete and evidence was
flushed.

## Valid-run aggregate evidence

| Scenario | Valid PASS | Mean navigation time | Mean final GT position error | Mean final GT yaw error | Mean replans |
| --- | ---: | ---: | ---: | ---: | ---: |
| `simple_reachable` | 3/3 | 7.434 s | 0.1366 m | 0.1229 rad | 8 |
| `static_obstacle_detour` | 3/3 | 35.479 s | 0.0919 m | 0.0600 rad | 40 |
| `multi_turn_healthy` | 2/2 valid | 44.036 s | 0.0490 m | 0.0794 rad | 46 |

Every valid run satisfied its frozen action, full-footprint safety, GT
quality/alignment, endpoint, final-stop, and scenario-geometry checks. The
offline summary's `FAIL` status is solely the strict 9/9 accounting of the
pre-goal infrastructure-invalid third multi-turn trial.

## Frozen boundaries

- Task 1–3 remain accepted baselines and were not modified for this closure.
- No Nav2 Planner, Controller, Costmap, BT Navigator, or AMCL algorithm
  parameter was changed.
- The runner remains GT-free and sends only `NavigateToPose`; Ground Truth
  remains evaluator-only.
- No additional dynamic trials will be appended to turn this engineering
  decision into a strict 9/9 result.
