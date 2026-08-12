# Robot Agent 长期开发规则

本文定义 ResilientNavLab Robot Diagnostic Agent 的长期开发合同。它约束后续阶段的职责、数据边界、权限升级和安全审查，不记录某个临时 Step 的类名、目录布局或实现进度。

规范性用语中，“必须”和“禁止”是硬约束；“应”表示默认选择，偏离时必须记录理由并获得当前任务授权。

## 1. 系统职责分层

Robot Agent 是健康监测之上的诊断子系统，不是新的传感器检测器，也不是实时控制器。

### 1.1 Detection

Detection 由确定性的 Health Monitor 负责，包括时序、陈旧、冻结、偏置、遮挡和其他可重复判定。它输出结构化健康状态、候选故障、指标、时间和置信度。

LLM 不负责替换基础检测规则，不得通过阅读 benchmark truth 直接“检测”故障。新增检测算法仍应作为可独立测试、可复现的监测模块实现。

### 1.2 Diagnosis

Diagnosis 是 Agent 当前和近期的主要职责。Agent 把多个 Health、Incident 和 Evidence 关联起来，形成故障假设、支持证据、反证、置信度、未知项和建议的下一步只读检查。

诊断输出是可审计推断，不是 Ground Truth。证据不足时必须保留不确定性，不得为了给出唯一答案而编造因果关系。

### 1.3 Decision

Decision 指根据诊断结果提出降级、任务调整、规划或恢复选择。它只能在后续阶段单独授权后进入系统，并应先经历“生成建议、人工确认、不执行”的只读阶段。

诊断正确不自动授予执行权。Decision 的输入、允许动作、风险等级和审批规则必须另有明确 Schema 与验收。

### 1.4 Real-time Control

实时控制、稳定环和紧急制动等确定性链路不得依赖 LLM。LLM 永远不进入实时控制闭环，不直接生成电机指令或持续控制 `/cmd_vel`。

即使未来加入 Planner 或 Recovery，实际执行也必须由有界、确定性、可超时和可撤销的控制组件承担。

## 2. Agent Input 与 Benchmark Truth 双通道

系统必须把 Agent 可见信息和评价真值作为两个独立通道维护。

### 2.1 Agent Input 通道

Agent Input 只包含完成诊断所需的、已经 Sanitizer 审核的观测信息，例如：

- 结构化 Health 状态与连续指标；
- 时间对齐后的 Evidence；
- 不泄露答案的设备、拓扑和能力元数据；
- 只读 Tool 返回的、经过 Schema 验证的观测；
- 为诊断构建的 Incident 上下文。

在线与离线 Agent 使用同一真值隔离标准。离线数据集、缓存、Trace 和测试 fixture 不能因为“不连接机器人”而绕过该标准。

### 2.2 Benchmark Truth 通道

Benchmark Truth 只供 Benchmark/Evaluator 计算检测、诊断、分类和恢复指标。该通道可以保存故障标签、实验场景和复现参数，但不得进入 Agent 的提示词、上下文、Tool 返回、检索库、缓存或 Trace。

以下信息属于禁止进入 Agent 的 truth 或 answer leakage：

- `FaultStatus` 和 `/fault_injection/status`；
- `scenario_id`、`scenario_seed`；
- fault model truth 和注入模型内部状态；
- `parameters_yaml` 及可反推出注入答案的配置；
- benchmark answer、期望分类和人工答案字段；
- `/faulted/*` 等直接暴露实验分支的内部命名。

Evaluator 可以在输出侧将 Agent 诊断与 Ground Truth 对齐，但不得把对齐结果回注到同一次诊断。

### 2.3 隔离要求

- 两个通道应使用独立 Schema、接口和存储命名，避免共享“万能事件对象”。
- Agent fixture 与 evaluator fixture 应分别构建；禁止先装载完整 truth 对象再靠提示词要求模型忽略字段。
- Trace 默认遵循 Agent Input 边界。需要记录 truth 的评价 Trace 必须单独标识、隔离访问并防止被后续诊断检索。
- 任何无法确认来源或语义的字段都按可能泄露处理，采用 fail-closed 策略。

## 3. Sanitizer 边界

Sanitizer 位于数据进入 Robot Domain 的 Agent Input 之前。ROS Adapter、离线 loader、replay 工具和测试数据生成器都不得绕过它。

Sanitizer 必须：

1. 按允许字段构造新的输入对象，而不是在完整对象上做易遗漏的删除。
2. 拒绝 truth 字段、答案标签和未登记的扩展字段。
3. 将 `/faulted/*` 等泄露性 topic、frame、source 或文件名转换为不带答案的稳定标识，或拒绝该输入。
4. 验证嵌套对象、自由文本、metadata、路径和 Trace 属性，避免真值藏在非主字段中。
5. 记录 Sanitizer 版本、拒绝原因和输入来源，但日志本身不得复制敏感真值到 Agent 可见位置。
6. 在 Schema 不兼容、字段未知或转换失败时停止输入，不做静默降级。

Sanitizer 负责防止答案泄露，不负责美化诊断结果，也不应修改真实传感器值来迎合模型。

## 4. Incident 与 Evidence

### 4.1 Incident

Incident 是一次有时间边界的诊断工作单元，应包含稳定 ID、观测窗口、涉及资产、当前健康摘要、Evidence 引用和诊断生命周期状态。

Incident 不等于故障真值。它可以由 Health 状态变化、规则聚合或人工只读请求触发，也可以在证据不足时以 unresolved 结束。

### 4.2 Evidence

Evidence 是不可变或追加式的结构化观测。每条 Evidence 应尽量包含：

- 采集时间与适用时间窗；
- Sanitizer 后的来源标识；
- 观测类型、值、单位和有效性；
- 质量、置信度或误差边界；
- 获取方式和必要的 provenance；
- 与原始 Health/观测的可审计引用。

自由文本日志可以作为补充 Evidence，但不能成为默认诊断协议。Agent 应优先使用明确 Schema 的 Health/Evidence，并区分“观测事实”“规则结果”“Agent 推断”和“Benchmark Truth”。

### 4.3 诊断产物

诊断结果应至少表达候选原因、支持证据、冲突证据、置信度、未知项和建议的下一步只读检查。结果必须可追溯到 Evidence ID，不得把模型的自然语言解释当作新的传感器事实。

## 5. 权限升级路线

权限按以下路线逐级提升，后一级不自动继承授权。每次升级必须有独立任务、威胁分析、Schema、测试、审计方案和验收结论。

### 5.1 Offline

读取已 Sanitizer 的静态 fixture、导出数据或录制数据，执行纯诊断。不得连接实时 ROS 图、调用写 Tool 或改变外部状态。

### 5.2 Read-only

允许受控读取非实时或受限数据源，仍不得修改 ROS、文件中的实验事实或机器人状态。Tool 必须只读、返回有界结构化结果并可 Mock。

### 5.3 Live ROS

允许通过 ROS Adapter 订阅白名单 topic、查询白名单只读状态并构建 Incident。此阶段仍无发布、参数修改、生命周期切换、节点管理或控制权限。

### 5.4 Planner

允许 Agent 生成符合 Schema 的候选计划，由确定性验证器检查。默认只展示、不执行；涉及状态变化时需要 Human-in-the-loop 和独立 Safety Gate。

### 5.5 Recovery

只允许执行预先登记的恢复动作。每个动作必须有明确前置条件、参数范围、超时、幂等策略、失败模式、回滚或安全停止路径，并产生完整审计记录。

任何阶段都不得直接开放任意 shell、任意代码执行、任意 ROS service/action/topic 或通用参数写入口。

## 6. Agent Core、Robot Domain 与 ROS Adapter

### 6.1 Agent Core

Robot Agent 必须依赖独立的 `Kyliangarnacho/agent-core`。Agent Core 负责跨领域通用能力，例如模型/Provider 抽象、结构化调用协议、通用 Tool 编排、Trace、依赖注入和可复用 Guardrail 接口。

禁止复制 `agent_core` 源码到本仓库。若通用能力存在缺陷，应在最小、无机器人依赖的复现中确认，再回 agent-core 修复和发布；不得在 Robot Domain 中维护私有 Core fork、影子实现或永久 monkey patch。

### 6.2 Robot Domain

Robot Domain 负责机器人诊断语义，包括 Health/Incident/Evidence/Diagnosis Schema、Sanitizer 规则、故障假设、证据关联、领域 Tool 合同和安全策略。核心逻辑应尽量是纯 Python，并能在无 ROS、无真实模型 API 的环境中测试。

跨模块边界优先使用明确的 Pydantic Schema。随意 `dict`、隐式字段约定和自由文本拼接不能作为稳定接口。

### 6.3 ROS Adapter

ROS Adapter 负责把白名单 ROS 消息转换为 Robot Domain Schema，并在未来授权时把经过 Safety Gate 的有界动作交给确定性 ROS 执行组件。

`rclpy`、`sensor_msgs`、`nav_msgs` 和其他 ROS 依赖不得进入 agent-core。Robot Domain 也不应要求单元测试启动 ROS graph；消息类型转换和 QoS 等细节留在 Adapter 层测试。

## 7. Safety Gate 原则

Safety Gate 是所有潜在状态变更的独立、确定性授权边界，不是提示词，也不能由 Agent 自行关闭。

一个可执行动作至少必须经过：

- 明确的白名单 Tool 与版本化输入 Schema；
- 调用方权限、当前模式和人工审批检查；
- 机器人状态、环境状态和互斥条件检查；
- 参数范围、速率、持续时间和资源上限检查；
- dry-run 或等价的执行前预览；
- 超时、幂等、失败处理和安全停止路径；
- 请求、批准、执行结果和拒绝原因的审计记录。

Safety Gate 必须 fail closed。模型文本、Tool 名称相似、历史成功或 evaluator 高分都不能替代显式授权。高风险动作应由机器人侧确定性安全机制拥有最终否决权。

## 8. 测试与模型调用合同

- Domain 测试默认使用 Fake/Mock，不调用真实模型 API。
- Sanitizer 必须有 allowlist、denylist、嵌套泄露、未知字段和 fail-closed 测试。
- Diagnosis 测试应验证证据引用、未知项和不确定性，而不只比较自然语言措辞。
- ROS Adapter 使用构造消息或最小隔离图测试；不因 Domain 单元测试启动完整仿真。
- 每个小步骤先跑 targeted tests，一个完整 Step 收口再跑相关回归。
- 真实 Qwen E2E 只有任务明确要求、凭据和数据边界获准时才运行。
- 没有新证据时，不重复运行耗时的大型构建、仿真或模型测试。

## 9. 当前 RA-1A 边界

RA-1A 仅允许离线、只读诊断。当前明确禁止：

- 创建任何可改变机器人状态的 Agent Tool；
- 发布 `/cmd_vel` 或其他控制 topic；
- 任意 shell 或 code execution；
- 任意 `ros2 param set`；
- 重启、kill 或切换 ROS node 生命周期；
- 修改 EKF、控制器、导航或硬件参数；
- 接入 Planner、Recovery、Nav2 或实时控制；
- 把 `FaultStatus`、故障注入状态或场景答案提供给 Agent；
- 通过文件名、topic 名、metadata、Trace 或 fixture 偷渡 benchmark answer；
- 调用真实模型 API，除非单独任务明确授权真实 E2E。

以上限制不禁止 Benchmark/Evaluator 在隔离通道使用 Ground Truth，也不要求把现有 `/health/*`、`/faulted/*` 或 `FaultStatus` 生产接口强行迁移。它只规定这些数据进入 Robot Agent 前必须遵守的边界。
