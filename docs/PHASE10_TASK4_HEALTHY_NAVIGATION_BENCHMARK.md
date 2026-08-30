# Phase 10 Task 4 — Healthy Navigation Benchmark

## Status

**CLOSED — engineering accepted with a known infrastructure limitation** on
2026-08-30. The final evidence and quantitative summary are in
`docs/PHASE10_TASK4_EVALUATION_SUMMARY.md` and
`docs/phase10_task4_batch_manual_20260830_r02/`.

Eight of eight valid navigation trials passed: simple 3/3, detour 3/3, and
multi-turn 2/2 valid. The remaining multi-turn r03 trial was invalid before
goal dispatch because `localization_tf` readiness saw past TF extrapolation.
The strict automated 9/9 contract is consequently not met. No further Task 4
dynamic verification will be added; this closure does not change Task 1–3 or
any Nav2 parameter.

## Historical investigation

The fresh `multi_turn_healthy` validation PASSed on 2026-08-29: initial path
`7.5349 m`, two effective turns, final GT error `0.0689 m / 0.1162 rad`,
navigation time `42.094 s`, zero recovery, 43 replans, and final-stop checks
all passed. Evidence is retained in `docs/phase10_task4_multi_turn_20260829_host/`.

The first formal 3x3 batch and its startup investigation are retained as
historical engineering evidence only. The final manual r02 evidence set and
the closure decision in `PHASE10_TASK4_EVALUATION_SUMMARY.md` supersede the
earlier provisional wording: Task 4 is CLOSED with 8/8 valid navigation
trials passing and one pre-goal infrastructure-invalid trial. No later Task 5
change reopened the Task 4 contracts or Nav2 baseline.

## Frozen scope

- The normal navigation chain is unchanged from Task 3:
  `NavigateToPose -> BT Navigator -> ComputePathToPose/Navfn -> FollowPath/RPP -> cmd_vel`.
- The benchmark runner sends only `NavigateToPose`; it never reads evaluator
  Ground Truth or invokes a lower-level Nav2 action. Its shutdown publisher
  emits all-zero `cmd_vel` only.
- Ground Truth remains evaluator-only on `/evaluation/ground_truth_pose`.
  Navigation feedback and `/clock` belong to GT-free navigation evidence;
  the offline evaluator joins that evidence with GT after the trial ends.
- The frozen scenarios are `simple_reachable`, `static_obstacle_detour`, and
  `multi_turn_healthy`. The latter remains goal `(4.7, 5.3, pi/2)` with a
  minimum initial path of `7.0 m` and two effective turns.

## Minimal readiness contract

The official Nav2 Lifecycle Manager alone configures and activates Planner,
Controller, and BT Navigator. The runner performs no lifecycle transition.
It observes, in this order, only the facts needed to send the benchmark goal:

1. one monotonic `/clock` publisher and samples;
2. filtered odometry plus `map -> base_footprint`;
3. the official `/lifecycle_manager_navigation/is_active` aggregate state;
4. the `NavigateToPose` action server;
5. both raw Costmaps used as benchmark evidence.

All stages share one broad 120 s wall-clock deadline. Startup time is not a
navigation metric; `navigation_time_sec` remains action-result minus
goal-accepted simulation time.

## Fresh-process and failure policy

Each trial has a unique `GZ_PARTITION`, `ROS_DOMAIN_ID`, run directory,
initial-pose result, launch log, and `trial_manifest.json`. After the launch
parent exits, a condition-based barrier verifies that its process group and
its partition have no remaining processes, then fsyncs evidence and manifest
files. A later trial may not start until that barrier completes. A trial wall
timeout is derived from its declared readiness timeout and scenario action
timeout, plus explicit post-goal/result grace; it is not a fixed 600 s cap.

Diagnostic rosbag recording is optional and disabled for the normal healthy
run so it does not join localization startup discovery. It can be enabled for
an explicitly requested forensic run; GT isolation is unchanged.

There is exactly one infrastructure attempt per logical run. The batch stops
at the first failure and retains its evidence. It does not use unconditional
inter-trial sleeps. Per-partition Gazebo cleanup is bounded and verified
before the next fresh trial.

Failure categories retain the earliest observable missing dependency:
`infrastructure_gazebo_clock`, `infrastructure_localization_tf`,
`infrastructure_nav2_lifecycle_service`, Nav2 action/Costmap readiness, GT
evidence, or navigation/evaluation failure. A lifecycle service error logged
only after shutdown is not treated as the original startup cause.

## Strict automated acceptance (not achieved)

Task 4 passes only when all nine fresh logical runs pass their frozen action,
full-footprint safety, GT quality/alignment, endpoint, final-stop, and
scenario geometry contracts; all evidence and aggregate manifests must remain
available outside transient `/tmp` paths.
