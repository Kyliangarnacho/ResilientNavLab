# ResilientNavLab

ResilientNavLab 是一个面向移动机器人的 ROS 2 实验与学习项目，目标是构建多传感器故障注入、健康评估、自适应融合和容错导航平台。

## 当前状态

项目当前处于**阶段 1：基础设施基线**。

- 已完成项目目标、范围和环境基线文档。
- 已安装 ROS 2 Jazzy，并完成基础命令和官方 talker/listener 通信验证。
- 已创建 `ros2_ws` 工作空间，并完成空工作空间构建。
- 已在 `resilient_nav_monitor` 中实现 `system_heartbeat` 节点，构建、测试和运行验证通过。
- 尚未安装 Gazebo。
- 尚未实现其他项目 ROS 2 节点。
- 尚未开始故障注入、健康评估、融合或导航功能开发。

当前环境可以运行 ROS 2 官方演示节点和项目 `system_heartbeat` 节点；仿真、健康评估和导航任务仍不可用。

## 核心方向

项目后续计划围绕以下能力逐步展开：

1. 多传感器数据接入与统一管理。
2. 可控、可复现的传感器故障注入。
3. 在线健康状态评估与异常检测。
4. 基于健康状态的自适应多传感器融合。
5. 面向定位与导航任务的降级运行和容错决策。
6. 仿真与实验指标记录、对比和复现。

详细边界见 [docs/PROJECT_SCOPE.md](docs/PROJECT_SCOPE.md)。

## 当前环境摘要

| 项目 | 当前状态 |
| --- | --- |
| 操作系统 | Ubuntu 24.04.4 LTS（Noble Numbat） |
| CPU 架构 | x86_64（Debian 架构名：amd64） |
| Git | 2.43.0，可用 |
| Python | Python 3.12.3（`python3` 可用） |
| Node.js | v24.18.0，可用 |
| npm | 11.16.0，可用 |
| Codex CLI | 0.145.0，可用 |
| ROS 2 | Jazzy；`ROS_DISTRO=jazzy`，`ROS_VERSION=2` |
| ROS 2 工具 | `ros2`、`colcon`、`rosdep` 可用 |
| ROS 2 基础通信 | 官方 C++ talker 与 Python listener 通信验证通过 |
| ROS 2 工作空间 | `ros2_ws` 已创建；空构建和 `--symlink-install` 包构建均通过 |
| 项目 ROS 2 包 | `resilient_nav_monitor` 可构建、测试并由 ROS 2 发现 |
| 项目 ROS 2 节点 | `system_heartbeat` 以 1 Hz 在 `/system_heartbeat` 发布递增存活消息 |
| Gazebo | 未安装 |

完整核验结果和复核命令见 [docs/ENVIRONMENT.md](docs/ENVIRONMENT.md)。

## 文档索引

- [项目范围](docs/PROJECT_SCOPE.md)
- [开发环境基线](docs/ENVIRONMENT.md)
- [学习与决策记录](docs/LEARNING_LOG.md)

## 近期里程碑

当前已完成 ROS 2 Jazzy 安装、官方演示节点基础通信验证、`ros2_ws` 工作空间基线和 `system_heartbeat` 节点。Gazebo、Nav2 和 SLAM 仍未安装；故障注入、健康评估、融合和导航功能仍未实现。
