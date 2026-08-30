# Phase 10 Summary

## Closure

**CLOSED — engineering accepted with known limitations** (2026-08-30).

Phase 10 freezes a healthy, saved-map Nav2 baseline for future resilience
work. The formal runtime chain is:

`Saved Map -> Map Server + AMCL -> Global/Local Costmaps -> Navfn -> RPP -> BT NavigateToPose -> cmd_vel`.

Ground Truth is evaluator-only. The Runner sends only `NavigateToPose`; it
does not read GT, command Planner/Controller actions, or publish nonzero
velocity.

## Accepted capabilities

- Task 1: saved map, Map Server and AMCL localization PASS.
- Task 2: Planner-owned Global Costmap, Controller-owned Local Costmap,
  polygon footprint, Static/Obstacle/Inflation layers PASS.
- Task 3: Navfn, RPP, official no-Recovery BT Navigator and 1 Hz replanning
  PASS.
- Task 4: healthy evaluator infrastructure CLOSED; 8/8 valid trials PASS
  (simple 3/3, detour 3/3, multi-turn 2/2 valid).
- Task 5.1: sensor-observed, dynamically spawned detour obstacle engineering
  PASS.
- Task 5.2: no-Recovery fully blocked safe failure PASS: Planner-first
  `NO_VALID_PATH/208 -> BT abort -> Controller stop`.
- Task 5.3: official Nav2 Recovery engineering PASS: Recovery occurred,
  independent temporary-wall deletion occurred, navigation resumed and ended
  SUCCESS.
- Task 5.4: native NavigateToPose Goal Cancel PASS: CANCELED terminal, zero
  Recovery, Controller stop, odom/GT stop, durable evidence and cleanup.

## Frozen limitations

- Task 4 strict automated 9/9 was not met: one multi-turn run was invalid
  before goal dispatch due to infrastructure TF readiness. The engineering
  verdict uses 8/8 valid PASS results.
- AMCL/localization versus GT can show about 0.3 m endpoint deviation; Task
  5 dynamic contracts retain it as a warning rather than tune Task 1–3.
- Scan/physical-wall angular misalignment remains a scan/TF technical debt.
- Fully blocked long-wall terminal latency is variable; safety and Planner
  failure semantics, not a short observation window, are the acceptance fact.
- Task 5.3 cannot prove clearing every cell along the former 11.32 m wall
  using finite-range/occluded LiDAR. This is an evidence limitation, not a
  Recovery-chain failure.
- The final Codex-run shared healthy regression is infrastructure-invalid:
  first sandbox DDS blocking, then foreground supervisor interruption after
  `Goal succeeded` but before terminal evidence. It does not supersede the
  retained Task 4 evidence.

## Phase 11 handoff

Phase 11 may consume the frozen saved-map Nav2 baseline, official optional
Recovery profile, evidence schema, and known limitations. It must not rewrite
Task 1–5 parameters to hide these observations. Sensor faults during
navigation, Health/Fusion-to-Nav2 adaptation, fault-aware localization/SLAM,
Robot-Agent control, and real-robot execution remain outside Phase 10.

See `PHASE10_EVIDENCE_INDEX.md` for retained evidence and
`PHASE10_TASK4_EVALUATION_SUMMARY.md` / `PHASE10_TASK5_DYNAMIC_OBSTACLES.md`
for task-level contracts.
