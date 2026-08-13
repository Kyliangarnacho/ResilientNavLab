# RA-1A Offline Robot Diagnosis Final

记录日期：2026-08-13

## 1. 完成结论

RA-1A 已形成第一次完整、可重复、可判卷的离线只读诊断闭环：

```text
OfflineRobotCase
        |
        +-- agent_view() --> OfflineAgentInput
        |                         |
        |                         v
        |              Offline Diagnosis Runtime
        |                         |
        |                         v
        |              RobotDomainExtension / Analyzer
        |                         |
        |                         v
        |                 agent-core AgentRuntime
        |                         |
        |                         +--> 0..N registered read-only Tools
        |                         |
        |                         v
        |                 strict DiagnosisResult
        |                         |
        |                         v
        |                 OfflineDiagnosisRun
        |                         |
        +-- BenchmarkTruth -------+--> BenchmarkScorer
                                          |
                                          v
                                 BenchmarkCaseResult
                                          |
                                          v
                                  BenchmarkReport
```

左侧执行链只取得 `OfflineAgentInput`。`BenchmarkTruth` 首次在执行结束后的
Scorer 调用中出现，不进入 Prompt、Context、Tool、completion messages 或 Trace。

本阶段仍是 Offline、read-only Diagnosis，不含 Live ROS、Planner、Recovery 或控制。

## 2. Robot Tools

三个 Tool 使用外部 agent-core 0.1.0 的 `ToolRegistry` / `ToolRuntime`，没有新建
第二套编排框架：

| Tool | 输入 | 返回 | 诊断边界 |
| --- | --- | --- | --- |
| `get_incident_health_snapshot` | 空对象 | 当前 Incident、全部 canonical Health 和 Evidence references | 只复制已有 sanitized observation |
| `compare_component_health` | 两个不同 canonical component | 两侧 state/score/metric 及 shared-metric difference | 只做确定性比较，不推断 cause |
| `inspect_metric_window` | component + metric name | value、window、sample count、state、Evidence references | metric 不存在时安全失败 |

`OfflineDiagnosisContext` 从 Agent input deep copy，Evidence/Health sequence 使用 tuple，
并由 frozen Pydantic model 约束。Registry handler 只闭包读取该 context；没有 ROS import、
文件访问、Shell、BenchmarkTruth、参数写入或机器人状态修改接口。

agent-core 负责 Tool definition、显式注册、参数 Pydantic validation、unknown Tool、
安全 error record、bounded repair、partial success、顺序执行和 Trace。Robot Domain 只负责
Tool 的机器人语义和返回 Schema。

## 3. Offline Diagnosis Runtime

正式入口为：

```python
run_offline_diagnosis(agent_input, completion, request_options=None)
```

入口只接受 `OfflineAgentInput`；传入 builder-only `OfflineRobotCase` 会直接拒绝。每次调用：

1. deep copy 并重新执行 Agent-side safe payload scan；
2. 有 Incident 时创建 truth-free `OfflineDiagnosisContext`；
3. 创建 `RobotDomainExtension` 和一次性显式 Robot Tool Registry；
4. 调用独立 agent-core `AgentRuntime`；
5. 保留 Core `RunTrace`、Tool records、model request count 和 latency；
6. 对 final output 执行受保护内容扫描、JSON parse、Pydantic validation 和 Incident contract check；
7. 返回 truth-free `OfflineDiagnosisRun`。

`OfflineDiagnosisRun` 状态为 `completed`、`no_diagnosis`、
`insufficient_evidence`、`blocked` 或 `structured_failure`。它可以保留安全 failure code，
但不保存 rejected raw model text，也不含任何 evaluator truth。

无 Incident 的 healthy control 仍经过 Core Analyzer/final path，但 route 由 Domain
确定性固定为 `healthy`；合法 final 必须是 `status=no_diagnosis`、`incident_id=null`、
空 hypotheses。这样不需要构造假的 healthy Incident。

## 4. Strict DiagnosisResult

agent-core 当前只对 Analyzer 提供 structured schema；final answer 仍是 text。Robot service
因此在 Domain boundary 严格处理 final：

- 只接受一个 JSON object；Markdown fence、普通文本和 malformed JSON 不会被包装；
- 使用 `DiagnosisResult` 的 `extra=forbid` Pydantic contract；
- `diagnosed` 必须有 primary hypothesis，primary ID 必须解析到 hypotheses；
- `no_diagnosis` 必须没有 Incident、primary 或 hypotheses；
- 有 Incident 的结果必须引用当前 sanitized Incident ID；
- 支持/反证 Evidence ID 在 Schema 内保持唯一与互斥；跨 Evidence catalog 的存在性由
  deterministic Scorer 显式判卷，便于保留并审计模型错误；
- final 中出现受保护 evaluator/experiment 标识会成为 `prohibited_output` structured failure。

安全 failure code 包含 `invalid_json`、`invalid_schema`、`contract_mismatch`、
`prohibited_output` 和 `runtime_error`。系统不会把非法文字转换成诊断成功。

## 5. Benchmark Scorer V1

`BenchmarkScorer.score()` 的唯一输入是：

```text
OfflineDiagnosisRun + BenchmarkTruth
```

它不调用模型，使用通用规则评估：

- Incident presence 与 Diagnosis expectation；
- primary component；
- normalized fault type 与 `acceptable_fault_types` aliases；
- top-k hypothesis 是否在同一 hypothesis 中覆盖 component + fault；
- 所有 supporting/contradicting Evidence ID 是否存在；
- supporting Evidence 是否覆盖 minimum required Evidence types；
- 可确定 unsupported claim：hypothesis 没有有效 supporting Evidence，或支持 Evidence
  的 component 与 hypothesis component 不一致；
- distinct Tool call、最终 Tool failure 和 model request 数；
- Agent-side prohibited-data leakage；
- healthy control false diagnosis，包括随后被 Runtime contract 拒绝的诊断尝试。

healthy case 不能只凭“没有 diagnosed result”通过；它必须提供 strict
`no_diagnosis`。这防止 malformed 或 blocked output 被误当作正确 healthy 行为。

Batch report 汇总 case/diagnostic/healthy count、component/fault accuracy、top-k、
Evidence validity、required Evidence success、leakage、false diagnosis、Tool/model usage、
latency 和所有 run status（包括 0 count）。

## 6. Fake Pipeline Benchmark

安装并加载 workspace 后可重复运行：

```bash
source ros2_ws/install/setup.bash
ros2 run resilient_nav_agent ra1a_fake_benchmark
```

本轮结果：

| 指标 | 结果 |
| --- | ---: |
| 标识 | `PIPELINE / FAKE BENCHMARK` |
| cases / diagnostic / healthy | `8 / 7 / 1` |
| passed | `8 / 8` |
| component accuracy | `1.0` |
| fault-type accuracy | `1.0` |
| top-k coverage | `1.0` |
| Evidence-reference validity | `1.0` |
| required-Evidence success | `1.0` |
| Tool calls / failures | `6 / 0` |
| model requests | `22` |
| leakage / false diagnosis | `0 / 0` |
| status | `completed=7, no_diagnosis=1, insufficient=0, blocked=0, structured_failure=0` |

`ReferencePipelineFakeCompletion` 是显式、确定性的 pipeline fixture。它读取 sanitized
input 并产生测试输出，用于验证消息、Tool、Schema 和 Scorer 数据流；以上 1.0 指标
不得解释为真实 Robot Agent 智力或模型质量。

## 7. Adversarial Validation

自动测试验证 pipeline/Scorer 能发现：

- strict correct diagnosis 和 explicit insufficient evidence；
- malformed final JSON；
- unknown Tool；
- invalid Tool arguments 和 bounded repair 后仍失败；
- 一次 multi-call 中一个成功、一个失败的 partial Tool execution；
- 不存在的 Evidence ID 与因此产生的 deterministic unsupported claim；
- detector hint `camera_quality` 与 truth wording `underexposed` 不一致；
- final 试图提及受保护 evaluation data；
- healthy control false diagnosis；
- direct no-tool 与 Tool path。

测试通过少量 scripted Fake variation 完成，没有为 CASE-001 至 CASE-008 在 Scorer
加入私有分支。

## 8. Ground Truth Boundary 复核

新增回归扫描：

- `OfflineAgentInput`；
- Domain context；
- `OfflineDiagnosisContext`；
- 三个 Tool output 和 Tool records；
- 传给 completion 的 Analyzer/Tool/final messages；
- Core Trace；
- `OfflineDiagnosisRun`。

禁止 token 命中为 0。Agent system prompt 也不再通过列举内部字段名来提醒模型；安全边界
由 Sanitizer、类型隔离、Context 构建和 output guard 实现，而不是要求模型“不要看”。

`BenchmarkTruth` 只存在于 builder envelope 和 Scorer/Batch evaluator side。Report 是
evaluator 产物，不会回注同一次 diagnosis。

## 9. 真实 Qwen E2E

未执行。只读配置检查确认当前环境没有 `AGENT_CORE_MODEL_*`、Qwen/DashScope 或
OpenAI API key/model 配置。未请求新 secret，未发起网络调用，也未进行无意义重试。

最初的无网络最小复现确认 agent-core 0.1.0 存在独立 public API 缺口：
`AgentRuntime` 传 `stream=False`，但 `CompatibleModelClient.complete` 不接受该参数，
underlying completion 因此为 0 次调用。该问题随后已在 sibling agent-core working tree
以通用 Core 修复和 integration regression 解决，没有在 Robot Domain 增加 wrapper 或
monkey patch。

修复后的无网络 CASE-001 probe 使用 `CompatibleModelClient` 包裹 injected transport，
完整穿过 Analyzer → Tool selection/execution → strict final；结果为 `completed/diagnose`、
IMU bias、1 条 Tool record、3 次 model/transport request，全部 `stream=False`，benchmark
通过且 Ground Truth leakage 为 0。该结果仍是 deterministic integration probe，不是
真实模型结果；Core 修复尚未 commit 或发布。

## 10. 验证结果

```text
targeted offline/formal/ground-truth tests: 24 passed
package pytest:                         88 passed
package colcon build:                   1 package finished
package colcon test:                    88 tests, 0 errors, 0 failures, 0 skipped
workspace colcon build:                 9 packages finished
workspace final result:                 491 tests, 0 errors, 0 failures, 1 skipped
git diff --check:                       passed
agent-core targeted integration tests: 49 passed
agent-core full pytest:                 99 passed
CompatibleModelClient Robot probe:     CASE-001 passed, 3 requests, 0 leakage
Robot offline targeted regression:     18 passed
```

完整 workspace 首轮有 3 个既有 ROS runtime failure，日志均为受限环境 DDS
`getifaddrs/socket Operation not permitted`。使用项目已有 `/tmp` ROS log workaround，
并在允许本机 DDS 的环境只复跑受影响的 health/camera 两包后全部通过。没有为环境问题
修改 ROS 代码或测试。

## 11. 明确未实现

- Live ROS subscriber / Adapter / Incident auto-trigger；
- rosbag ingestion；
- real Qwen benchmark；
- RAG、Skills、MCP 或 Multi-Agent；
- Planner、Recovery、Nav2 Agent control 或 Safety Gate action；
- Shell/code execution、ROS 参数写入、节点管理或 `/cmd_vel`；
- EKF、Health Monitor、Fault Injection 或相机算法改写。

下一自然节点不是扩大 Robot Domain，而是 review、commit 并发布独立 agent-core 的
client/runtime 兼容修复；获得安全模型配置后，再在不改变 prompt/fixture 的前提下选
4 至 6 个代表 case 做一次小规模真实 Qwen Offline E2E，审计 structured output、
Evidence 引用、Tool usage、编造状态和 insufficient-evidence 行为。
