# Phase 10 Evidence Index

## Retained acceptance evidence

| Scope | Retained evidence | Closure use |
| --- | --- | --- |
| Task 1–3 | `PHASE10_TASK1_LOCALIZATION_SMOKE.md`, `PHASE10_TASK2_COSTMAP_SMOKE.md`, `PHASE10_TASK3_*` | Saved-map localization, Costmap/footprint, Navfn/RPP/BT healthy baseline |
| Task 4 | `phase10_task4_batch_manual_20260830_r02/` and `PHASE10_TASK4_EVALUATION_SUMMARY.md` | 8/8 valid healthy trials PASS; one pre-goal infrastructure-invalid multi-turn run |
| Task 5.1 | `phase10_task5_dynamic_obstacle_detour_host_r01/` | Dynamic detour engineering PASS |
| Task 5.2 | `phase10_task5_dynamic_fully_blocked_discovery_host_r03/` plus `offline_reassessment.json` | Planner-first `NO_VALID_PATH/208` safe failure PASS |
| Task 5.3 | `phase10_task5_dynamic_temporary_recovery_discovery_host_r02/` | Engineering PASS with finite-observation Global Costmap-clear limitation |
| Task 5.4 | `phase10_task5_goal_cancel_acceptance_host_r01/` | Native Goal Cancel PASS |

## Retained infrastructure history

- `phase10_final_shared_healthy_regression_20260830_r01/` is invalid because
  the Codex sandbox prohibited DDS/UDP sockets before ROS graph creation.
- `phase10_final_shared_healthy_regression_20260830_r02/` reached Nav2
  `Goal succeeded` but the external foreground supervisor ended before the
  Runner/GT recorder wrote terminal evidence. It is not a navigation failure
  and is not counted as a new acceptance result.
- Earlier Task 5 discovery directories remain as causal debugging history;
  they must not replace the listed retained acceptance evidence.

No evidence under this index is a runtime input to Nav2 or the Robot Agent.
