# 开源项目基线

本文集中记录 ResilientNavLab 对外部开源项目的取舍。外部项目只提供架构与设计参考；只有明确列为
“采用”的内容才是当前实现依据，“暂缓”内容不构成后续实现授权。

## 阶段 8：健康感知融合

- **参考：** `robot_localization` 与 ROS diagnostics 生态。
- **采用：** 保留 fixed EKF 与 adaptive EKF 两条独立、可对照的链路；由确定性的健康状态控制
  measurement adapter，不在 EKF 内部隐藏故障策略。
- **暂缓：** 运行中随意修改 EKF 内部参数、引入大型诊断图框架。
- **原因：** 独立链路更容易做因果对照、回归和安全回退。

## 阶段 9：SLAM

- **参考并采用：** Slam Toolbox 的 ROS 2 Jazzy 接口、pose graph 保存和 localization mode。
- **暂缓：** Cartographer、RTAB-Map 迁移，以及未获得场景证据的大规模 SLAM 框架替换。
- **原因：** 当前二维 LiDAR 场景已由 Slam Toolbox 完成建图、持久化和重定位闭环。

## 阶段 10：Nav2

- **参考并采用：** Nav2 Jazzy 官方生命周期、Navfn、Regulated Pure Pursuit、BT Navigator、Recovery
  和 Costmap 分层接口。
- **暂缓：** vendoring Nav2、定制 controller plugin 或为实验表现重写标准导航栈。
- **原因：** 官方组件已覆盖 saved-map 导航基线，项目代码只负责场景、配置和评价边界。

## BRNE V1

- **参考并采用：** `MurpheyLab/brne` pinned commit
  `633a5cdcb39ab27f18b596cb8cb1968644f82391` 的数学核心与官方 ROS runtime profile。
- **采用边界：** BRNE core 保持 pinned；项目侧只实现 ROS-free wrapper、传感器 dynamic-agent 输入、
  proposal/interaction 策略、time-aligned safety mask 和控制发布边界。
- **暂缓：** 修改 BRNE fixed-point/core、重新设计 initial weights 或引入新的学习式感知依赖。
- **原因：** 保持上游算法可追溯，同时把机器人、场景和跨周期工程语义留在项目层。

## RA-1A：Robot Diagnostic Agent

记录日期：2026-08-13。RA-1A 只建立离线、只读 Robot Domain、数据合同、Sanitizer 和外部
`agent-core` 集成，不授权 Live ROS、Planner、Recovery 或机器人状态变更。

### Pydantic / Pydantic AI

- **参考：** [pydantic/pydantic](https://github.com/pydantic/pydantic) 与
  [pydantic/pydantic-ai](https://github.com/pydantic/pydantic-ai)。
- **采用：** schema-first、strict typed boundary、依赖注入和可测试性。
- **暂缓：** 引入 Pydantic AI runtime；它会与独立 `agent-core` 的职责重叠。

### OpenAI Agents SDK

- **参考：** [openai/openai-agents-python](https://github.com/openai/openai-agents-python)。
- **采用：** Tool 与 runtime 分层、guardrail、validation boundary 和 trace-first debugging。
- **暂缓：** 用该框架替换现有 `agent-core`。

### LangGraph

- **参考：** [langchain-ai/langgraph](https://github.com/langchain-ai/langgraph)。
- **采用：** typed state、有限状态迁移和可观测执行的设计思想。
- **暂缓：** 引入 LangGraph 依赖或提前搭建复杂图执行器。

### Autoware 与 ROS diagnostics

- **参考：** [autowarefoundation/autoware_universe](https://github.com/autowarefoundation/autoware_universe)
  和 [ros/diagnostics](https://github.com/ros/diagnostics)。
- **采用：** 结构化 diagnostics、component/function health 分层、聚合和 self-test 思想。
- **暂缓：** diagnostic DAG、Autoware 大型依赖，以及把现有 `/health/*` 强制迁移到 `/diagnostics`。

## 使用规则

- 新增依赖、替换 `agent-core`、迁移框架或扩大 Agent 权限，必须获得单独授权。
- 设计相似不等于复制实现；采用项必须符合 Ground Truth 隔离、Safety Gate 和测试合同。
- 外部项目后续变化不静默改写历史决策；必要时在新阶段补充一条简洁记录。
