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
- The real Qwen runner disables only the optional Analyzer `response_format`
  capability and uses the provider option `enable_thinking=false`; strict JSON
  prompts and Pydantic validation remain mandatory.

## Validation

| Validation | Result |
| --- | --- |
| Fake reference benchmark | `PIPELINE / FAKE BENCHMARK`, 8/8 passed; pipeline evidence only, not model accuracy |
| Real four-case smoke | `REAL MODEL BENCHMARK`, 2/4 passed; component 3/3, fault/top-k 1/3, Evidence validity 1.0, 8 requests, 14,677.1 ms, leakage/false diagnosis 0/0 |
| Real eight-case baseline | 3/8 passed; component 7/7, fault/top-k 2/7, Evidence validity 1.0, 16 requests, 32,647.7 ms, leakage/false diagnosis 0/0 |
| No-network real-runner/client regression | Passed: CASE-001 traversed Analyzer -> Tool -> final through `CompatibleModelClient`; 3 requests, all `stream=False`, no leakage |
| RA-1A targeted/adversarial/leakage tests | 31 passed |
| `resilient_nav_agent` pytest / colcon test | 91 passed / 91 tests, 0 errors, 0 failures |
| agent-core pytest | 100 passed (only its read-only checkout could not write pytest cache) |

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

Provider: DashScope/Qwen compatible API; model: `qwen3.7-flash`. Real network
requests were confirmed. The direct eight-case CLI was attempted, but its
roughly 34-second output exceeded the host command-output window; the reported
eight-case baseline is the same `BatchBenchmarkRunner` executed as two real
four-case groups (001/002/006/008 and 003/004/005/007).

| Case | Route / status | Primary diagnosis | Scorer result | Latency |
| --- | --- | --- | --- | ---: |
| 001 | diagnose / completed | imu / bias | pass | 3,605.2 ms |
| 002 | diagnose / completed | wheel / Wheel Mechanical or Encoder Freeze | fault mismatch | 5,242.9 ms |
| 003 | diagnose / completed | scan / sector_blindness | pass | 4,693.3 ms |
| 004 | diagnose / completed | camera / sensor_stream_stale | fault mismatch | 3,991.7 ms |
| 005 | diagnose / completed | camera / image-pipeline freeze wording | fault mismatch | 4,795.3 ms |
| 006 | diagnose / completed | camera / sustained_dark_exposure | fault mismatch | 3,745.7 ms |
| 007 | diagnose / completed | camera / image-blur wording | fault mismatch | 4,490.3 ms |
| 008 | healthy / no_diagnosis | none | pass | 2,083.3 ms |

All eight runs had valid Evidence references, zero Tool calls/failures, two
model requests, zero leakage, and zero healthy false diagnosis. The model did
not select an optional read-only Tool in this baseline.

Infrastructure fixes:

- Qwen model rejected `response_format=json_object`; agent-core now supports an
  explicit optional Analyzer response-format capability while retaining its
  default and strict schema parsing.
- The Analyzer now receives its Pydantic JSON schema, and Robot final routes
  receive the `DiagnosisResult` schema. This fixes the general contract, not a
  CASE answer.

Model baseline failures are CASE-002/004/005/006/007 fault-label granularity;
all retain the correct primary component. No fixture, truth, Scorer, or
CASE-specific prompt was changed. Fake 8/8 remains pipeline validation only,
not model accuracy.

## Deferred

- Live ROS adapter and trigger;
- rosbag/live incident ingestion;
- Planner and Recovery;
- deterministic Safety Gate actions;
- Nav2 recovery/control;
- Robot RAG or Memory;
- Multi-Agent.
