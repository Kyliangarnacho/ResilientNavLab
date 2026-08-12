# ResilientNavLab Codex 项目指令

本文件是 Codex 在本仓库工作的项目入口和强制规则索引。
项目事实、长期 Robot Agent 合同和外部基线分别以链接文档为准，避免在此重复展开。

## 项目定位

ResilientNavLab 是 ROS 2 / Gazebo 多传感器健康监测、容错导航和 Robot Diagnostic Agent 综合项目。

当前仓库状态、已验证能力和未实现边界见：

- [README.md](README.md)
- [项目范围](docs/PROJECT_SCOPE.md)
- [当前状态](docs/CURRENT_STATE.md)
- [环境与复核方式](docs/ENVIRONMENT.md)
- [学习与决策记录](docs/LEARNING_LOG.md)

Robot Agent 的长期开发合同与 baseline 决策见：

- [Robot Agent 开发规则](docs/ROBOT_AGENT_DEV_RULES.md)
- [开源项目基线](docs/OPEN_SOURCE_BASELINES.md)

## 开始工作前

1. 阅读 `README.md`、`docs/PROJECT_SCOPE.md`、`docs/ENVIRONMENT.md` 和
   `docs/LEARNING_LOG.md`，再按任务读取相关专题文档。
2. 执行 `git status --short`，检查最新 commit，保留用户已有且与任务无关的修改。
3. 核验当前阶段、授权边界和验收条件，不把规划能力写成已实现能力。
4. 涉及 ROS 2、Gazebo 或硬件时，先按 `docs/ENVIRONMENT.md` 复核实际环境。
5. 未经明确授权，不安装依赖、不修改系统配置、不创建新 ROS 包或未来阶段架构。

## Robot Agent 分层原则

- Robot Agent 是 ResilientNavLab 的上层诊断子系统，不替代既有监测与控制链。
- Detection 由确定性的 Health Monitor 负责。
- Agent 负责 Diagnosis；只有后续阶段经单独授权后，才逐步进入 Decision。
- LLM 永远不得进入实时控制闭环。
- 当前 RA-1A 只允许离线、只读诊断，不得实现 Planner、Recovery 或机器人状态变更。

## Agent Core 强制边界

- Robot Agent 必须依赖独立的 `Kyliangarnacho/agent-core`。
- 禁止把 `agent_core` 源码复制到本仓库。
- 通用运行时、模型抽象、Tool/Trace 等 Core 能力应留在 agent-core；机器人语义留在 Robot Domain。
- 若发现 Core 通用能力缺陷，必须先构造最小复现，再回 agent-core 修复。
- 禁止在 Robot Domain 内私自 fork、影子实现或长期 monkey-patch 一套 Core。

## Ground Truth 铁律

- 在线或离线 Agent 输入均不得包含 `FaultStatus`、`/fault_injection/status`、
  `scenario_id`、`scenario_seed`、fault model truth、`parameters_yaml` 或任何 benchmark answer。
- `/faulted/*` 等会通过内部命名泄露实验真值的标识，必须经过 Sanitizer 后才能进入 Agent。
- Sanitizer 应 fail closed；无法证明字段安全时，拒绝进入 Agent Input 通道。
- Ground Truth 只允许 Benchmark/Evaluator 使用，不得用于提示词、上下文、Tool 返回或诊断缓存。
- 现有 evaluator 使用真值是合法的 Benchmark 通道，不得据此放宽 Agent 输入边界。

## Safety 铁律

- RA-1A 禁止 Robot Agent 发布 `/cmd_vel`。
- 禁止任意 shell 或 code execution。
- 禁止任意 `ros2 param set`。
- 禁止重启或 kill ROS node。
- 禁止修改 EKF、控制器或其他运行参数。
- 禁止借助通用 Tool 绕过上述限制。
- 未来任何改变机器人状态的操作，都必须经过显式白名单 Tool 和独立 Safety Gate。

## 架构规则

- Robot Domain 与 ROS Adapter 必须分离；Domain 核心逻辑尽量保持纯 Python。
- `rclpy`、`sensor_msgs`、`nav_msgs` 等 ROS 依赖不得渗入 agent-core。
- ROS Adapter 只负责消息转换、订阅读取和边界适配，不承载通用 Agent Core 逻辑。
- 跨模块边界优先使用明确的 Pydantic Schema，禁止用随意 `dict` 形成隐式协议。
- Agent 优先消费结构化 Health / Incident / Evidence，不依赖自由文本日志猜测故障。
- 诊断必须保留证据来源、时间、置信度和不确定性，不得把推断伪装成检测真值。

## 测试与验证规则

- 默认使用 Fake/Mock，不调用真实模型 API。
- 每个小步骤先运行 targeted tests；一个完整 Step 收口时再运行相关回归。
- 真实 Qwen E2E 只有在任务明确要求时才允许运行。
- 不为“保险”反复运行没有新证据的大型测试。
- 测试范围应与改动风险相称；文档任务不默认触发 ROS/Gazebo 动态测试。
- 不伪造测试、构建、仿真、硬件或模型调用结果；不能运行时明确说明原因。

## Git 与变更规则

- 禁止 `git reset`、`git clean` 或其他会破坏用户未提交工作的命令。
- 未明确要求时，不 commit、不 push、不改写历史。
- 保持提交内容聚焦，不覆盖或清理无关文件。
- 新能力变更应同步更新相应范围、环境或学习记录；纯规则文档按任务要求更新。
- 最终汇报必须包含 modified files、tests、deviations 和 remaining issues。

## 外部 Baseline 原则

- 外部高质量开源项目只作为架构与设计参考，不作为默认依赖清单。
- 禁止为了模仿而强行增加依赖或迁移框架。
- 每个 Robot Agent 大阶段的 baseline 决策记录在
  `docs/OPEN_SOURCE_BASELINES.md`。
- Codex 只按已记录的 Adopt 项实现；Reject-or-Defer 项不得擅自引入。
- 大规模迁移第三方框架必须另行提案、授权和验收。

## 文档与实现纪律

- `docs/ROBOT_AGENT_DEV_RULES.md` 定义长期稳定合同，本文件只保留不可跳过的入口规则。
- `docs/OPEN_SOURCE_BASELINES.md` 记录外部参考的 Adopt / Reject-or-Defer / Reason。
- 临时 Step 细节写入对应任务文档，不污染长期合同。
- 计划、实验结果与已实现能力必须清楚区分。
- 当前任务无授权的生产代码、ROS 接口和消息定义一律不得修改。
