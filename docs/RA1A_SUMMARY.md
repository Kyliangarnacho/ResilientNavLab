# RA-1A 总结：离线 Robot Diagnostic Agent

## 完成架构

```text
Offline health observations
  -> allowlist Sanitizer
  -> HealthObservation / Evidence / Incident
  -> OfflineAgentInput
  -> RobotDomainExtension
  -> external agent-core AgentRuntime
  -> read-only Tools
  -> strict DiagnosisResult
  -> isolated BenchmarkScorer
```

RA-1A 依赖独立 `Kyliangarnacho/agent-core`，本仓库不复制 Core。三个 Tool 只读取 sanitized context；
Ground Truth、`FaultStatus`、场景参数和 `/faulted/*` 命名不得进入 Agent Input、Tool、Trace 或缓存。

## 验证

- 8-case Fake pipeline：8/8，通过的只是合同与管线，不代表模型能力。
- DashScope/Qwen 四 Case smoke：2/4。
- 八 Case baseline：3/8；primary component 7/7，fault/top-k 2/7，Evidence validity 1.0，零 leakage，
  healthy false diagnosis 为零。
- 主要失败是 fault-label 粒度，不通过修改 fixture、truth 或 Scorer 美化。

## 安全边界

当前只允许离线、只读 Diagnosis。没有 Live ROS、Planner、Recovery、参数写入、节点管理、任意 shell
或 `/cmd_vel`。LLM 永远不进入实时控制闭环。

