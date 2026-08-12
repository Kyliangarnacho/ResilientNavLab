# RA-1A Step 1：Robot Agent Bootstrap

记录日期：2026-08-13

## 1. 结果与边界

RA-1A Step 1 首次在 ResilientNavLab 中建立 `resilient_nav_agent`。它是独立的 `ament_python` ROS 2 包，但当前实现完全是离线、只读的纯 Python Robot Domain：定义严格数据合同、建立 Ground Truth Sanitizer、构造 Incident/Evidence，并用 Fake completion 接通仓库外的 `Kyliangarnacho/agent-core`。

本 Step 没有调用真实 LLM API，没有 Live ROS subscription、Robot Tool、RAG、Planner、Recovery、`/cmd_vel`、参数写入或节点管理，也没有修改 `SensorHealth.msg`、`FaultStatus.msg`、Health Monitor、EKF 或 Fault Injection 逻辑。

## 2. 为什么 Robot Domain 与 ROS Adapter 分离

现有 `SensorHealth.msg` 是 ROS transport contract，而 Robot Agent 需要跨离线 fixture、未来 rosbag case 和 Live ROS 使用同一套可审计语义。如果 Domain 直接 import `rclpy`、`sensor_msgs` 或 `nav_msgs`，单元测试会依赖 ROS graph，Agent Core 也会被机器人中间件细节污染。

因此依赖方向固定为：

```text
ROS message / offline DTO
          |
          v
   future ROS Adapter
          |
          v
  plain Mapping / DTO
          |
          v
 AgentInputSanitizer
          |
          v
 pure Robot Domain schemas/builders
          |
          v
 external agent-core
```

RA-1A Step 1 只实现 `Mapping → Sanitizer → Domain`。未来 Adapter 才能负责 `SensorHealth.msg → Mapping`；当前没有 subscriber 或 ROS message import。

## 3. 当前目录结构

```text
ros2_ws/src/resilient_nav_agent/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/resilient_nav_agent
├── resilient_nav_agent/
│   ├── __init__.py
│   ├── schemas.py
│   ├── sanitizer.py
│   ├── evidence.py
│   ├── incidents.py
│   ├── prompts.py
│   ├── extension.py
│   └── offline/__init__.py
└── test/
    ├── test_schemas.py
    ├── test_sanitizer.py
    ├── test_ground_truth_boundary.py
    ├── test_incidents.py
    ├── test_agent_core_integration.py
    ├── test_architecture.py
    ├── test_flake8.py
    └── test_pep257.py
```

## 4. 依赖方向与 agent-core

依赖只允许单向流动：

```text
ResilientNav SensorHealth-like Mapping
                   |
                   v
        resilient_nav_agent Domain
                   |
                   v
   installed Kyliangarnacho/agent-core
```

`setup.py` 声明 `agent-core>=0.1.0` 与 `pydantic>=2.8`。本次机器上系统 Python 由 PEP 668 管理，因此在仓库根目录创建被 `.gitignore` 排除的 `.venv`，并从 sibling checkout `/home/kylian/projects/agent-core` 做 editable install。实际 import 为：

```text
/home/kylian/projects/agent-core/agent_core/__init__.py
agent-core 0.1.0
Pydantic 2.13.4
```

绝对路径只作为本次环境证据记录，没有写入 package 源码或 metadata。本仓库内不存在 `agent_core/` copy；自动架构测试同时验证 import 路径在 ResilientNavLab 之外。

## 5. Canonical Schemas

所有主要 Schema 使用 Pydantic v2、`extra="forbid"`、有限长度、有限数值和明确 Enum。

### 5.1 HealthObservation

`HealthObservation` 是 Sanitizer 后 Agent 真正可见的一次健康观测。它包含 canonical component、Health state、可空 score/confidence、detector hint、reasons、由平行数组转换的 metrics、时间窗、样本数和观测时间。

既有 `SensorHealth` 在 UNKNOWN 时使用 `health_score=-1.0` 哨兵；Sanitizer 将其转换为 `None`，避免非法单位区间值进入 JSON contract。NaN 和正负无穷全部拒绝。

### 5.2 RobotIncident

`RobotIncident` 是一次有边界的离线诊断工作单元。当前 builder 只允许 DEGRADED 或 FAULT 触发：

- DEGRADED → `operational_severity=warning`
- FAULT → `operational_severity=fault`
- HEALTHY / UNKNOWN → 不创建 fault diagnosis incident

severity 只来自 Agent 可见 Health state，不读取 `FaultStatus.severity`。本 Step 不实现多组件 critical propagation 或 Live Incident lifecycle。

### 5.3 EvidenceItem

`EvidenceItem` 是有限、结构化、可引用的观测。首版类型为 `health`、`sensor_metric`、`motion`、`camera_health`、`comparison`、`probe_result`；Schema 不接受 `ground_truth`、`fault_status` 或 `benchmark_answer` 类型。

Health evidence 的 source 是 canonical `health-monitor:<component>`，不会保留 ROS transport topic。`structured_data` 只允许有限 JSON，拒绝非有限数值、Data URL、Base64 marker 和过深/过大结构。

### 5.4 DiagnosisHypothesis / DiagnosisResult

Hypothesis 只使用 `low`、`medium`、`high` 定性支持度，不伪造模型概率。`DiagnosisResult.primary_hypothesis_id` 非空时必须引用本结果中的 hypothesis；证据 ID 的跨 Store 完整性留给后续 service/validator 层。

## 6. Sanitizer 是 Trust Boundary

Sanitizer 表达的是：

```text
EXPERIMENT WORLD
      |
      v
AgentInputSanitizer  -- fail closed --> SanitizationError
      |
      v
AGENT WORLD
```

它先递归扫描整棵输入，再按 allowlist 构造一个全新的 `HealthObservation`，而不是从完整实验对象中删除少数字段。稳定 `SanitizationError.code` 和固定文案不会回显被拒绝 payload、绝对路径、凭据或内部异常文本。

### 6.1 Ground Truth denylist

首版拒绝：

- `FaultStatus`、`fault_status`、`fault_injection_status`
- `scenario_id`、`scenario_seed`、`event_id`
- `faulted_topic`、`model`、`severity`、`affected_fields`
- `start_time`、`end_time`
- `parameters_yaml`
- `ground_truth`、`truth_label`
- `expected_fault`、`expected_state`
- `fault_model_truth`、`benchmark_answer`

其中 `event_id`、`faulted_topic`、`model`、`start_time`、`end_time`、`severity` 和 `affected_fields` 来自本仓库真实 `FaultStatus.msg`。递归检测覆盖嵌套 Mapping 与 Sequence，`metadata.scenario_seed` 也会拒绝。

### 6.2 `/faulted/*` 处理

现有 IMU/wheel/scan Health Monitor 默认读取 `/faulted/*`。即使 payload 本身合法，topic 名也直接暴露实验分支。

首版明确选择：顶层 `source_topic` 只可临时用于 component normalization，随后完全丢弃；最终 `HealthObservation` 没有 source topic 字段。若 `/faulted/` 出现在 reason、metadata 或其他 Agent-facing 字符串中，则立即拒绝。`/fault_injection/status` 在任何字符串位置都拒绝。

### 6.3 Metric conversion

当前 `SensorHealth.msg` 使用 `metric_names[]` 和 `metric_values[]`。Sanitizer 验证两者长度相等、名称唯一且有限长、数值有限，再转换为 `dict[str, float]`。长度不一致、重复名称、布尔值、NaN 或 Inf 都 fail closed。

### 6.4 当前四类兼容

测试 fixture 忠实使用当前 `SensorHealth.msg` 字段和 Monitor 常见 metric/hint，覆盖：

- IMU：bias/residual
- wheel：freeze/pose span/commanded motion
- scan：sector blindness/NaN sector
- camera：underexposed/gray/Laplacian health

四类都能得到统一 `HealthObservation`，无需修改 `SensorHealth.msg` 或既有算法。

## 7. detected_fault_hint 不是 Ground Truth

`detected_fault_hint` 来自确定性 Health Monitor 的 `detected_fault` 输出，例如 `bias`、`freeze` 或 `underexposed`。它是 detector observation，可以作为诊断证据，但可能误报、漏报或类别不完整。

Ground Truth 则来自 fault injection scenario、`FaultStatus` 或人工 benchmark label。二者必须隔离；字段名中的 “fault” 不能作为简单禁词，否则会错误拒绝合法 detector hint。因此 Sanitizer 使用精确 truth key、危险 source 和递归结构规则，而不是粗暴删除所有 fault 字样。

## 8. RobotDomainExtension Bootstrap

`RobotDomainExtension` 真实继承外部 `agent_core.DomainExtension`，extension ID 为 `resilient-nav.robot-diagnostics`。最小 `RobotAnalysis` 只表达 incident category、primary component、evidence 是否充分、是否请求 Tool、是否需要更多证据和短理由。

route 仅有：

- `diagnose`
- `needs_more_evidence`
- `blocked`

RA-1A 没有 Tool Registry；分析若请求 Tool 会直接 blocked。Fake integration 进行了两次纯内存 completion：第一次返回合法结构化 analysis，第二次返回 fake answer，最终得到 `AgentResult`、`diagnose` route、`RunStatus.COMPLETED` 和空 tool records。没有创建 OpenAI client，也没有读取或发送 API key。

## 9. 测试

### 9.1 Targeted

```text
python -m pytest -q test/test_schemas.py test/test_sanitizer.py
44 passed

python -m pytest -q test/test_incidents.py test/test_ground_truth_boundary.py
7 passed

python -m pytest -q test/test_agent_core_integration.py test/test_architecture.py
5 passed
```

### 9.2 Package

```text
python -m pytest -q
58 passed

colcon build --symlink-install --packages-select resilient_nav_agent
1 package finished

colcon test --packages-select resilient_nav_agent
colcon test-result --test-result-base build/resilient_nav_agent --verbose
58 tests, 0 errors, 0 failures, 0 skipped
```

### 9.3 Workspace regression

```text
colcon build --symlink-install
9 packages finished

colcon test
colcon test-result --verbose
461 tests, 0 errors, 0 failures, 1 skipped
```

全工作空间的一次完整运行发生在 Agent 包有 51 项测试时，总数为 454。首次受限沙箱运行得到 5 个环境失败：DDS `getifaddrs/socket Operation not permitted` 与默认 `~/.ros/log` 只读。只对 `resilient_nav_fault_injection`、`resilient_nav_health_assessment`、`resilient_nav_camera` 设置 `ROS_LOG_DIR=/tmp/resilient_nav_ra1a_ros_logs` 并在允许本机 DDS 的环境复跑一次后全部通过。

随后只扩展了 7 项 Agent denylist/安全错误测试并修改 Sanitizer，不涉及既有包；按 targeted-first 规则只重新构建和测试 `resilient_nav_agent`，得到 58 项通过。保留其他八包刚完成的结果后，最终 `colcon test-result --verbose` 汇总为上述 461 项；没有无新证据地重复完整工作空间测试，也没有修改既有 ROS 代码或测试。

### 9.4 Ground Truth leakage scan

`test_agent_input_contains_no_ground_truth` 将最终 `HealthObservation`、`EvidenceItem` 和 `RobotIncident` 序列化并递归扫描以下 token：

```text
FaultStatus
/fault_injection/status
/faulted/
scenario_id
scenario_seed
parameters_yaml
ground_truth
benchmark_answer
```

最终命中数为 0。另有真实 `FaultStatus.msg` 全字段 fixture，确认无法作为 Health 输入通过 Sanitizer。

## 10. 当前没有实现

- Live ROS Adapter 或 subscriber
- 自动 Incident listener / lifecycle
- rosbag Case Builder
- 正式 Robot Diagnosis prompt 与 Diagnosis service
- 正式 Robot Tools、Shell、代码执行或 ROS 参数写入
- Robot RAG、Skills、MCP、Multi-Agent
- Planner、Nav2、Behavior Tree、Recovery 或 `/cmd_vel`
- 真实 Qwen/LLM API E2E
- diagnostic graph、DAG 或 fault propagation graph

## 11. Known Limitations

- 当前输入是 plain Mapping；尚未有运行时 `SensorHealth.msg → Mapping` Adapter。
- component normalization 只支持 `imu`、`wheel`、`scan`、`camera`。
- Incident builder 只做单 trigger 的 warning/fault 映射，不处理 critical propagation。
- `EvidenceItem.structured_data` 有严格 JSON/大小边界，但跨 Evidence Store 的引用完整性尚无 service 层。
- `RobotAnalysis` 与 route 只用于验证外部 Core 可被 Robot Domain 消费，不代表完整诊断策略。
- agent-core 当前通过本机 sibling editable install 提供；新的开发环境必须单独取得该仓库并安装，不能从 ResilientNavLab 内 vendor。
- `package.xml` 可表达 Python Pydantic 运行依赖，但 agent-core 是独立 Python 包且当前没有 ROS rosdep key；精确 Python 版本约束以 `setup.py` 为准。

## 12. 后续 Offline Case Checkpoint

同日后续收口在 Step 1 基础上新增了三个严格分离的合同：
`OfflineAgentInput`、`BenchmarkTruth` 和 builder-only `OfflineRobotCase`。
`OfflineRobotCase.agent_view()` 只返回前者；Agent Runtime/Domain context 不接收
case envelope 或 truth。`OfflineCaseBuilder` 的两条路径为：

```text
Raw Health -> Sanitizer -> HealthObservation -> Evidence -> RobotIncident
           -> OfflineAgentInput

FaultStatus/scenario mapping -> BenchmarkTruth
```

新增 8 个 `ra1a-reference-v1` Python reference fixture：IMU bias、wheel
freeze、Lidar sector blindness、camera stale/freeze/underexposed/blurred 和
healthy control。它们是与当前 monitor 指标/阈值一致的确定性代表数据，不是
rosbag 或 recorded real run。healthy control 保留健康 Evidence，但不创建
RobotIncident；underexposed case 的 detector hint 为 `camera_quality`，不等于
truth wording。

`SYSTEM_PROMPT` 已升级为 Robot Diagnostic Agent V1；`RobotAnalysis` 只负责
evidence sufficiency、Tool need、candidate checks 和 route，不承载最终 hypothesis。
当前正式 route 为 `diagnose`、`needs_more_evidence`、`blocked`。新增的
`OfflineDiagnosisContext` 只是后续只读 Tool 的 sanitized dependency groundwork，
本 checkpoint 没有注册或执行 Tool。

新增模块 targeted tests 为 17 项通过；package pytest 与 colcon test 均为
70 tests、0 errors、0 failures、0 skipped，package build 通过。`colcon test`
必须由仓库 `.venv` Python 启动，原因是 Pydantic 与 editable agent-core 只安装
在该环境；直接使用 `/usr/bin/colcon` 的系统 Python 会在 collection 阶段缺少
Pydantic。按紧急 STOP POINT 没有重跑 workspace regression，461 项仍是 Step 1
的最近完整工作空间基线。

明确 deferred：三个正式 Robot Tools、OfflineDiagnosisRunner、strict final
DiagnosisResult service、Benchmark Scorer/CaseResult/Report、batch benchmark、
Langfuse-style observability、真实 Qwen、Live ROS/Adapter、rosbag parser、RAG、
Skills、MCP、Planner 和 Recovery。
