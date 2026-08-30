# Phase 10 Task 4 fresh multi-turn validation — 2026-08-29

## Outcome

PASS. This was the one post-refactor fresh `multi_turn_healthy` validation,
run in its own Gazebo partition and ROS domain with diagnostics bag recording
disabled to avoid adding diagnostic DDS load during lifecycle validation.

| Contract | Observed |
| --- | ---: |
| Initial Navfn path | 7.5349 m |
| Effective turns | 2 |
| Navigation time | 42.094 s |
| GT travel distance | 7.4847 m |
| Final GT position / yaw error | 0.0689 m / 0.1162 rad |
| Replanning count | 43 |
| Recoveries | 0 |
| GT invalid samples | 0 |
| Final stop | PASS |

`lifecycle_manager_navigation` configured and activated Planner, Controller,
and BT Navigator without `async_send_request failed`. The first transient TF
wait during startup resolved after robot spawn, clock bridge, AMCL initial pose
and map TF became available; it was not included in navigation time.

## Retained evidence

- `docs/phase10_task4_multi_turn_20260829_host/navigation.json`
- `docs/phase10_task4_multi_turn_20260829_host/ground_truth.json`
- `docs/phase10_task4_multi_turn_20260829_host/initial_pose.json`
- `docs/phase10_task4_multi_turn_20260829_host/result.json`
- `docs/phase10_task4_multi_turn_20260829_host/ros_logs/`

The initial sandboxed attempt is retained separately only as an environment
failure: DDS UDP socket creation and `getifaddrs` were denied before readiness.
It is not benchmark evidence.

## Formal batch stop record

The subsequent formal batch preserved its first-failure evidence in
`docs/phase10_task4_batch_20260829/`. `simple_reachable-r01` PASSed, then
`static_obstacle_detour-r01` stopped before goal acceptance because the
localization lifecycle manager did not receive the map-server change-state
response. The runner correctly recorded the downstream `localization_tf`
absence. The preserved original result keeps that runner classification;
the post-run batch classifier regression now promotes the earlier
lifecycle-service log event to `infrastructure_nav2_lifecycle_service` on a
future run without rewriting raw evidence.
