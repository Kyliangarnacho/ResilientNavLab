# Phase 10 Task 5 — Dynamic Obstacles

## Status

**Task 5.1 CLOSED — engineering accepted with a known localization limitation**
on 2026-08-30. **Task 5.2 CLOSED — no-Recovery safe-failure baseline**: host
r03 recorded Planner-first Navfn `NO_VALID_PATH` / `208`, BT abort, FollowPath
cancel and Controller stop.  Its 12 s post-detection observation window is a
recorded timing warning, not a failure of the causal/safety chain. **Task 5.3
CLOSED — engineering accepted with an experiment/evidence limitation.** Host
r02 established the core official-Recovery causal chain: Recovery occurred
(`recovery_count=1`), the independently scheduled temporary wall was deleted
after 45 simulation seconds, `NavigateToPose` succeeded with error code `0`,
and the Controller resumed then stopped normally. The strict evidence
requirement that Global Costmap clear the entire former 11.32 m wall region is
not met: finite LiDAR observation range, occlusion, and the wall geometry make
that whole-region observation unavailable. This is retained as a limitation,
not a blocker for Recovery; the user approved engineering acceptance and no
rerun, scenario change, or evaluator change is authorized for it.

**Task 5.4 CLOSED — Goal Cancel PASS.** Host
`phase10_task5_goal_cancel_acceptance_host_r01/` records native
`NavigateToPose` status `CANCELED`, `recovery_count=0`, Controller-originated
post-cancel zero velocity, settled odometry/GT, no failures, and complete
cleanup/evidence flush.

Task 5.1–5.4 add one thin, Gazebo-only event overlay to the closed Task 4
trial. They do not modify Planner, Controller, Costmap, AMCL, the saved map,
or any Task 1–3 algorithm parameter. Task 5.3 selects an official BT profile;
it does not modify the frozen baseline profile.

## Borrow / Adapt / Reject

- **Borrow:** `ros_gz_bridge` exposes Gazebo's official
  `/world/resilient_lab/create@ros_gz_interfaces/srv/SpawnEntity` service.
  The injector calls it once; it does not create an injection protocol.
- **Borrow:** existing Static/Obstacle/Inflation layers, the unmodified
  `navigate_w_replanning_time.xml` 1 Hz `RateController`, official
  `navigate_to_pose_w_replanning_and_recovery.xml`, official Behavior Server
  / Clear Costmap / Spin / Wait / BackUp, Task 4 fresh trial, GT recorder,
  evaluator, final-stop and full-footprint machinery.
- **Adapt:** a single event injector observes an initial `/plan` and normal
  filtered-odometry displacement of `0.20 m`, creates the frozen SDF model
  and, for Task 5.3 only, deletes it on a frozen `/clock` deadline. It never
  observes Recovery. Offline-only metrics merge that evidence with Task 4
  navigation and Ground Truth evidence.  Task 5.3 also opts into RPP
  1.3.12's native full-Path closest-pose search; all baseline profiles retain
  the existing 3.0 m search bound.
- **Reject:** direct Costmap writes, planner/controller actions, goal changes,
  GT-controlled triggering, velocity publication, custom recovery BT/state
  machines, retries, readiness gates, sleeps, broad Nav2 tuning, or treating
  `INVALID_PATH` as a successful/recoverable result without fixing its cause.

## Frozen scenarios

Both cases reuse the Task 4 `static_obstacle_detour` start/goal `(0,0) ->
(4,4,0)`.

- `dynamic_obstacle_detour` (Task 5.1) creates a `0.50 x 0.50 x 1.00 m` box
  at map `(1.55, 0.85, 0)`, after the initial Path and motion gate. Offline
  saved-map connectivity, expanded by the frozen footprint, retains a route.
  Task 5.1 PASS requires SpawnEntity ACK, Global and Local Costmap detection,
  `NavigateToPose` success, a safe post-detection global Path, no recovery,
  final stop, and no GT padded-footprint intersection with the spawned box.
  Task 4 endpoint/localization metrics remain recorded as inherited warnings;
  they do not redefine this dynamic-obstacle acceptance contract.
- `dynamic_fully_blocked` (Task 5.2) creates a `11.32 x 0.20 x 1.00 m` wall
  centered at map `(3.44, 1.40, 0)`. Offline free-space analysis separates the
  fixed start and goal after the event. PASS is a safe failure, not arrival:
  both Costmaps detect the obstacle, no GT padded-footprint intersection,
  no recovery, controller zero plus settled odometry, and a natural ABORTED
  `NavigateToPose`. r03 froze the Planner-first `208` branch.  Terminal
  latency under this long wall remains a warning/evidence value; any external
  cancellation or timeout is inconclusive, never a PASS.
- `dynamic_temporary_blocked_recovery` (Task 5.3) reuses the same frozen wall,
  but chooses the official Nav2
  `navigate_to_pose_w_replanning_and_recovery.xml` and starts the official
  `behavior_server`.  Spawn occurs after the initial Path and `0.20 m` odom
  gate.  DeleteEntity is scheduled exactly `45.0` simulation seconds after
  SpawnEntity ACK, independent of any BT/recovery event.  PASS requires
  pre-delete Planner/Controller failure, official clear evidence, at least one
  Spin/Wait/BackUp RUNNING event, `recovery_count>0`, DeleteEntity ACK, a safe
  replan, Controller resumed motion, NavigateToPose SUCCESS, final stop and
  no GT footprint collision while the wall exists. r02 satisfies this core
  contract; its unavailable all-wall Global Costmap-clear coverage is the
  recorded engineering-acceptance limitation above.

Installed Nav2 Jazzy 1.3.12 can surface Navfn `NO_VALID_PATH` (208), or a
controller/progress result such as `PATIENCE_EXCEEDED` (104) or
`FAILED_TO_MAKE_PROGRESS` (105). Which terminal source wins is deliberately
measured rather than assumed or accepted from this candidate list.

## Evidence boundary

`navigation_obstacle_event_injector.py` never imports or subscribes to
Ground Truth. The new evaluator-only geometry check transforms recorded GT
after the trial and checks the padded footprint against the frozen spawned
obstacle. The benchmark Runner still sends only `NavigateToPose`; it does not
call lower Nav2 actions or publish non-zero velocity.

The runner preserves Task 4's strict live Path sweep. For Task 5 it records
the pre-event sweep but makes the acceptance decision on post-detection Paths:
otherwise a transient pre-event scan observation could retroactively reject a
route before the dynamic event. A post-detection unsafe sweep remains a hard
offline evaluation failure.

## Task 5.1 formal host evidence

`docs/phase10_task5_dynamic_obstacle_detour_host_r01/` is the retained fresh
host-process result.  It proves the Task 5.1 causal contract:

- initial Path at `31.368 s`, then normal odometry motion `0.2020 m`;
- SpawnEntity ACK at `32.906 s`;
- Local/Global Costmap detection at `33.502/34.218 s`;
- the frozen box intersects the initial Path corridor;
- first post-global-detection replan latency `0.394 s`, with 40 safe
  post-obstacle replans;
- `NavigateToPose` SUCCESS, error code `0`, no Recovery;
- GT padded-footprint collision with the spawned box is false;
- Controller stop, odometry settle, GT settle, evidence flush and process
  cleanup are all complete.

The original Task 4 healthy endpoint metric is retained as a warning:
`final_gt_position_error_m=0.309411786 m`, above its inherited `0.25 m`
threshold.  The evidence identifies this as a localization-estimate versus GT
limitation; it is not evidence that the dynamic obstacle chain failed.  No
AMCL, Planner, Controller, Costmap or Task 3 baseline parameter was changed
to remove it.  The Task 5 evaluator therefore reports this value under
`warnings` / `inherited_healthy_baseline_metrics`, while its Task 5 dynamic
contract passes.

## Task 5.2 retained host evidence

`docs/phase10_task5_dynamic_fully_blocked_discovery_host_r03/` is the retained
host discovery. Its current offline reassessment is
`offline_reassessment.json`: PASS with Planner-first `208`, no Recovery, no
GT footprint collision/crossing, Controller zero, and settled GT. The original
`result.json` records the now-superseded observation-window evaluator failure;
both records are retained for traceability. It is not to be rerun merely to
chase the old 12-second observation window.

## Task 5.3 retained host evidence

### r01 runtime finding and isolated correction

`docs/phase10_task5_dynamic_temporary_recovery_discovery_host_r01/` ended at
simulation time about `73.54 s`, before the independently frozen obstacle
removal deadline `74.954 s`.  Navfn had continued producing paths and the
Controller had accepted roughly 1 Hz replacements, but RPP reported
`Resulting plan has 0 poses in it.` and Controller Server mapped the
`nav2_core::InvalidPath` exception to FollowPath/NavigateToPose code `103`.

In Nav2 1.3.12, RPP searches for the robot only within the first
`max_robot_pose_search_dist` of its current Path.  Because this project did
not override the parameter, the default was half the 6 m Local Costmap:
`3.0 m`.  The long-wall run reached a replaced/pruned Path state whose
bounded candidate was already outside the same 3 m costmap extent, leaving
the transformed plan empty.  TF transformation itself did not fail; that
would have produced code `102` instead.  The official
`WouldAControllerRecoveryHelp` condition checks UNKNOWN/104/105/106, not
103, so the stock Recovery tree correctly propagated this unhandled
controller-contract error without running Behavior Server actions.

The isolated correction leaves `nav2_controller.yaml` and all no-Recovery
launches unchanged.  Only `use_recovery:=true` supplies the upstream-native
negative `max_robot_pose_search_dist`, which Nav2 1.3.12 defines as searching
every Path pose for the closest robot pose.  It does not modify the official
Recovery XML or swallow a genuine `INVALID_PATH`.

`docs/phase10_task5_dynamic_temporary_recovery_discovery_host_r02/` is the
retained host evidence. Do not rerun Task 5.3 merely to obtain full Global
Costmap coverage of the removed long wall.

## Task 5.4 Goal Cancel — closed PASS

`goal_cancel` reuses the existing Task 4/5 fresh-process supervisor, GT
recorder, Runner, Probe, final-stop evidence, and offline evaluator. It adds
no obstacle entity service and does not change any Task 1–3/Nav2 parameter.
The Runner owns the sole `/navigate_to_pose` ActionClient and, after its goal
is accepted, an initial global Path exists, and filtered odometry has advanced
the frozen `0.20 m`, calls native `cancel_goal_async()` on that same goal
handle. Neither the trigger nor the cancellation action observes GT.

PASS requires native cancel acknowledgement, action status `CANCELED`, no
Recovery, a nonzero Controller command before cancel, a Controller-originated
zero command after cancel (captured before Runner teardown publishes its
safety-only zeros), settled filtered odometry and GT, safe initial
full-footprint Path evidence, and complete durable navigation/GT/manifest
evidence. Goal endpoint error is intentionally not an acceptance metric,
because cancellation is required before arrival.

Host `phase10_task5_goal_cancel_acceptance_host_r01/` passed this contract.
Its result has `failures=[]`, native action status `CANCELED`, no Recovery,
Controller stop before Runner teardown safety-zero, and settled odometry/GT.
There is intentionally no `obstacle_event.json`: Task 5.4 does not start the
Gazebo entity bridge/injector. No repetition or retry is planned.
