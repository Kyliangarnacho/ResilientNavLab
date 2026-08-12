# Robot Agent 开源项目基线

本文记录每个 Robot Agent 大阶段对外部高质量开源项目的取舍。外部项目只提供架构与设计参考；只有明确写入 **Adopt** 的思想才是当前实现依据，**Reject-or-Defer** 内容不得被视为隐含授权。

## RA-1A Step 1 Baseline Snapshot

记录日期：2026-08-13

阶段边界：RA-1A Step 1 只建立离线、只读 Robot Domain、数据合同、Sanitizer 和外部 agent-core Fake integration。本 snapshot 不授权真实模型调用、Live ROS、Planner、Recovery、Nav2 接入或机器人状态变更。

### pydantic/pydantic-ai

- **Reference：** [pydantic/pydantic-ai](https://github.com/pydantic/pydantic-ai)；类型与数据验证思想同时参考 [pydantic/pydantic](https://github.com/pydantic/pydantic)。
- **Adopt：** Schema-first、strict typed boundary、依赖注入和可测试性；跨边界数据使用明确模型，测试时替换外部依赖。
- **Reject-or-Defer：** RA-1A 不引入 Pydantic AI 框架依赖，也不把现有 agent-core 改写为其运行时。
- **Reason：** 这些设计原则能提升诊断输入、Evidence 和输出协议的可验证性，但引入完整框架会扩大依赖面并与独立 agent-core 的职责重叠。

### openai/openai-agents-python

- **Reference：** [openai/openai-agents-python](https://github.com/openai/openai-agents-python)。
- **Adopt：** Tool 与 Agent Runtime 明确分层、Guardrail/validation boundary、Trace-first debugging，以及 Agent 不得绕过受控接口。
- **Reject-or-Defer：** 不以该框架替换现有 agent-core，不在 RA-1A 引入新的 Agent runtime。
- **Reason：** 其概念边界适合作为安全和审计参考，但本项目已要求 Robot Agent 依赖独立 agent-core，替换 runtime 会造成双重抽象与迁移成本。

### langchain-ai/langgraph

- **Reference：** [langchain-ai/langgraph](https://github.com/langchain-ai/langgraph)。
- **Adopt：** typed state、有限状态迁移和可观测执行；诊断流程应有显式状态与审计点。
- **Reject-or-Defer：** RA-1A 不引入 LangGraph 依赖，不为了工作流外观提前搭建复杂图执行器。
- **Reason：** 当前离线诊断可以先用小型、明确的领域状态模型验证边界；框架依赖应在流程复杂度和持久化需求得到证据后再评估。

### ros-navigation/navigation2

- **Reference：** [ros-navigation/navigation2](https://github.com/ros-navigation/navigation2)。
- **Adopt：** 仅作为后续 Planner / Recovery 的行为编排、恢复行为和生命周期边界参考。
- **Reject-or-Defer：** RA-1A 不接入 Nav2，不实现 Behavior Tree、恢复插件或控制接口。
- **Reason：** Nav2 面向确定性导航执行，适合未来研究 Agent 建议如何映射到受控行为；当前阶段没有 Decision 或执行授权。

### autowarefoundation/autoware_universe

- **Reference：** [autowarefoundation/autoware_universe](https://github.com/autowarefoundation/autoware_universe)。
- **Adopt：** 结构化 diagnostics、component/function health 分层，以及诊断信息机器可读而非只存在日志中的思想。
- **Reject-or-Defer：** RA-1A 暂缓 diagnostic graph / DAG 和 fault propagation graph，也不迁移 Autoware 的大型模块或依赖体系。
- **Reason：** RA-1A Step 1 只有单 Incident Offline Diagnosis；功能依赖图有长期价值，但当前 Incident/Evidence 合同应先在更小范围稳定。

### ros/diagnostics

- **Reference：** [ros/diagnostics](https://github.com/ros/diagnostics)，定位为 ROS 官方标准参考，而不是高 Star 创新 baseline。
- **Adopt：** 关注 `DiagnosticArray`、`diagnostic_updater`、`diagnostic_aggregator` 和 self-test 的消息组织、聚合与自检思想。
- **Reject-or-Defer：** 当前不要求把现有 `/health/*` 强行迁移到 `/diagnostics`，也不借 RA-1A 改写既有 `SensorHealth` 接口。
- **Reason：** 与 ROS 标准诊断生态保持概念兼容有长期收益，但现有健康链已经过验证；没有迁移需求和验收证据时，强制改名或换接口只会增加回归风险。

## 使用规则

- 新增依赖、替换 agent-core、接入 Nav2 或引入诊断 DAG，都必须在后续阶段更新本文件并获得明确授权。
- 设计相似不等于复制实现；应结合 ResilientNavLab 的 Ground Truth 隔离、Safety Gate 和测试合同做最小采用。
- 若外部项目后续变化，本 snapshot 仍表示 RA-1A 当时的决策；新的采用理由应以新阶段记录补充，不静默改写历史。
