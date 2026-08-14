# RA-1A Final Audit

## Scope

RA-1A is an offline/sim Robot Diagnostic Agent validation slice. It is not a
Live ROS Robot controller, planner, recovery system, or safety-action executor.

## Final architecture

```text
Offline/sim health observations
  -> allowlist Sanitizer
  -> HealthObservation + Evidence + Incident
  -> OfflineAgentInput
  -> RobotDomainExtension
  -> agent-core AgentRuntime
  -> 0..N read-only Robot Tools
  -> strict DiagnosisResult parsing/validation
  -> OfflineDiagnosisRun
  -> isolated BenchmarkScorer + BenchmarkReport
```

The Fake and real-model paths share the same `OfflineAgentInput`, Runtime,
Tools, strict output boundary, run record, and Scorer. They branch only at the
completion source:

```text
PIPELINE / FAKE BENCHMARK -> ReferencePipelineFakeCompletion
REAL MODEL BENCHMARK      -> CompatibleModelClient -> compatible provider API
```

`ra1a_real_benchmark` defaults to CASE-001 (IMU bias), CASE-002 (wheel
freeze), CASE-006 (camera underexposed), and CASE-008 (healthy). `--all`
selects CASE-001 through CASE-008. It passes only `OfflineRobotCase.agent_view()`
to its client factory; `BenchmarkTruth` remains within the post-run Scorer.

## Safety boundaries

- The Agent never sees `BenchmarkTruth`, `FaultStatus`,
  `/fault_injection/status`, scenario seeds, injection parameters, or answers.
- `/faulted/*` identifiers are sanitized before the Agent input boundary.
- Sanitizer checks apply to offline input, context, model output, Tool records,
  and trace before accepting a diagnosis.
- Tools are bounded, local, and read-only. There is no shell/code execution,
  ROS write operation, `/cmd_vel`, parameter change, node management, recovery,
  or controller action.
- `CompatibleModelClient` accepts Runtime-owned `stream=False`; provider options
  cannot override Runtime/client-owned message, stream, tool, or format fields.

## Validation

| Validation | Result |
| --- | --- |
| Fake reference benchmark | `PIPELINE / FAKE BENCHMARK`, 8/8 passed; pipeline evidence only, not model accuracy |
| Real four-case smoke | Not executed: current shell lacks `AGENT_CORE_MODEL_API_KEY` and `AGENT_CORE_MODEL_NAME` |
| Real eight-case benchmark | Not executed; available only through explicit `ra1a_real_benchmark --all` |
| No-network real-runner/client regression | Passed: CASE-001 traversed Analyzer -> Tool -> final through `CompatibleModelClient`; 3 requests, all `stream=False`, no leakage |
| RA-1A targeted/adversarial/leakage tests | 31 passed |
| `resilient_nav_agent` pytest / colcon test | 91 passed / 91 tests, 0 errors, 0 failures |
| agent-core pytest | 99 passed (only its read-only checkout could not write pytest cache) |

When configuration is absent, the real command makes no API request and emits
`REAL MODEL BENCHMARK` with `status: blocked` and the exact blocker:
`Real API execution blocked by missing environment configuration.`

Configure only through the existing agent-core variables; never store or print
credentials:

```bash
export AGENT_CORE_MODEL_API_KEY=...
export AGENT_CORE_MODEL_NAME=...
# Optional: AGENT_CORE_MODEL_BASE_URL and AGENT_CORE_MODEL_TIMEOUT_SECONDS
ros2 run resilient_nav_agent ra1a_real_benchmark
ros2 run resilient_nav_agent ra1a_real_benchmark --all
```

## Real-model failures

No real provider call was made in this closeout environment, so there are no
model-baseline results or model failures to report. No infrastructure failure
remains in the tested client/runtime/tool path; missing credentials/model name
is an execution-environment block, not a benchmark result.

## Deferred

- Live ROS adapter and trigger;
- rosbag/live incident ingestion;
- Planner and Recovery;
- deterministic Safety Gate actions;
- Nav2 recovery/control;
- Robot RAG or Memory;
- Multi-Agent.
