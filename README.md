# ResilientNavLab

ResilientNavLab 是一个面向移动机器人的 ROS 2 实验与学习项目，目标是构建多传感器故障注入、健康评估、自适应融合和容错导航平台。

## 当前状态

项目当前处于**初始化与基础环境准备阶段**。

- 已完成项目目标、范围和环境基线文档。
- 已安装 ROS 2 Jazzy，并完成基础命令和官方 talker/listener 通信验证。
- 尚未安装 Gazebo。
- 尚未创建 ROS 2 工作空间。
- 尚未创建 ROS 2 包。
- 尚未开始机器人功能开发。

当前环境可以运行 ROS 2 官方演示节点，但仓库尚不能构建或运行项目机器人节点、仿真或导航任务。

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
| Gazebo | 未安装 |

完整核验结果和复核命令见 [docs/ENVIRONMENT.md](docs/ENVIRONMENT.md)。

## 文档索引

- [项目范围](docs/PROJECT_SCOPE.md)
- [开发环境基线](docs/ENVIRONMENT.md)
- [学习与决策记录](docs/LEARNING_LOG.md)

## 近期里程碑

当前已完成 ROS 2 Jazzy 安装和官方演示节点基础通信验证。Gazebo、Nav2 和 SLAM 仍未安装；ROS 2 工作空间和项目 ROS 2 包仍未创建，后续初始化需在单独任务中明确实施。
