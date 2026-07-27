# ResilientNavLab

ResilientNavLab 是一个面向移动机器人的 ROS 2 实验与学习项目，目标是构建多传感器故障注入、健康评估、自适应融合和容错导航平台。

## 当前状态

项目已完成**阶段 2：Gazebo 基础仿真与 ROS 2 时钟链路**的收尾验收。

- ROS 2 Jazzy、`ros2_ws`、`resilient_nav_monitor` 和 `system_heartbeat` 的阶段 1 基线保持可用。
- 已通过 `ros-jazzy-ros-gz` 安装 Gazebo Harmonic；Gazebo Sim 版本为 8.11.0。
- 已创建 `resilient_nav_simulation` 包，包含自定义 SDF 世界、`/clock` 桥接配置、Python Launch 文件和静态资源测试。
- 自定义世界已在 Gazebo 中人工验证，可见 `ground_plane`、`box_obstacle` 和 `cylinder_checkpoint`。
- Launch 启动后，Gazebo 的仿真时钟经 `ros_gz_bridge` 出现在 ROS 2 `/clock`，`system_heartbeat` 使用 `use_sim_time=true`。
- 暂停和恢复 Gazebo 时，ROS 2 `/clock` 与心跳会同步停止和继续，说明节点确实由仿真时间驱动。
- 修改 `box_obstacle` 的 SDF 位姿并重新启动后，Gazebo 中的坐标变化已生效。

阶段 2 的完成边界仅是静态世界、Gazebo—ROS 2 时钟桥和已有心跳节点的仿真时间验证。机器人、URDF、传感器、运动与控制、Nav2、SLAM、故障注入、健康评估、自适应融合和容错导航均尚未开始。

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
| 项目 ROS 2 包 | `resilient_nav_monitor`、`resilient_nav_simulation` 可构建、测试并由 ROS 2 发现 |
| 项目 ROS 2 节点 | `system_heartbeat` 以 1 Hz 在 `/system_heartbeat` 发布递增存活消息 |
| Gazebo | Gazebo Harmonic / Gazebo Sim 8.11.0，可用 |
| ROS 2—Gazebo 集成 | `ros-jazzy-ros-gz` 已安装；`/clock` 单向桥接和仿真时间联动已验证 |
| 阶段 2 仿真资源 | 自定义静态世界、桥接配置和 Launch 集成已验证 |

完整核验结果和复核命令见 [docs/ENVIRONMENT.md](docs/ENVIRONMENT.md)。

## 文档索引

- [项目范围](docs/PROJECT_SCOPE.md)
- [开发环境基线](docs/ENVIRONMENT.md)
- [学习与决策记录](docs/LEARNING_LOG.md)
- [阶段 2 收尾总结](docs/PHASE2_SUMMARY.md)

## 近期里程碑

阶段 2 已完成 Gazebo Harmonic 安装、基础世界加载、`/clock` 桥接和 `use_sim_time` 联动验证。下一阶段尚未开始；机器人、URDF、传感器、Nav2、SLAM、故障注入、健康评估、融合和导航功能仍未实现。
