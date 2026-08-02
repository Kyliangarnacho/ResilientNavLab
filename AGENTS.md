# AGENTS.md

本文件用于约束在 ResilientNavLab 仓库中工作的开发者和自动化代理。

## 项目定位

ResilientNavLab 的目标是构建基于 ROS 2 的移动机器人多传感器故障注入、健康评估、自适应融合和容错导航平台。

项目的阶段 0（初始化）、阶段 1（ROS 2 基础设施）、阶段 2（Gazebo 基础仿真与时钟链路）、阶段 3（虚拟差速机器人与基础运动）、阶段 4（多传感器与局部定位基线）和阶段 5（可复现故障注入实验闭环）已经完成。当前环境已建立 ROS 2 Jazzy、Gazebo Harmonic、`ros_gz` 和 `ros2_ws` 工作空间，工作空间内已有六个 ROS 2 软件包：

- `resilient_nav_monitor`：提供已验证的 `system_heartbeat` 和 `odom_tf_broadcaster` 节点。
- `resilient_nav_simulation`：提供自定义 SDF 世界、阶段 2/3 Launch、ROS—Gazebo bridge、RViz 配置、运动测试工具和相应测试。
- `resilient_nav_description`：提供基础两轮差速机器人 Xacro、Display Launch、RViz 配置，以及 Gazebo 材质、接触参数和原生差速/关节状态插件。
- `resilient_nav_localization`：提供阶段 4 wheel odometry + IMU EKF 配置和完整健康链入口。
- `resilient_nav_interfaces`：提供阶段 5 `FaultStatus` 消息接口。
- `resilient_nav_fault_injection`：提供阶段 5 故障模型、注入器、场景、统一 Launch、faulted EKF、probe、RViz 和 rosbag 工具。

阶段 2 已验证自定义世界加载、Gazebo `/clock` 到 ROS 2 `/clock` 的单向桥接、Launch 集成，以及 `system_heartbeat` 使用 `use_sim_time=true` 时随 Gazebo 暂停和恢复。阶段 3 已验证机器人描述、Gazebo 原生差速和关节状态、ROS 基础运动 bridge、运行时 TF、Gazebo/RViz 联合显示，以及短时直行、原地旋转、圆弧和自动停车。阶段 4 已验证 IMU、二维 Lidar、RGB-D 和健康 EKF。阶段 5 已验证 raw/faulted topic 并存、`FaultStatus` 状态窗口、faulted EKF 对照、IMU bias、wheel freeze、Lidar blindness、rosbag 记录与无 Gazebo 回放。

完整运动性能、PointCloud2、RGB-D 故障、`ros2_control`、Nav2、SLAM、健康评估、自适应融合和容错导航仍未实现；后续能力必须在单独任务中明确授权和验收。

## 开始工作前

1. 阅读 `README.md`、`docs/PROJECT_SCOPE.md`、`docs/ENVIRONMENT.md` 和 `docs/LEARNING_LOG.md`。
2. 检查工作树状态，保留用户已有且与当前任务无关的修改。
3. 核验当前阶段、任务边界和验收条件，不把未来规划写成已实现能力。
4. 涉及 ROS 2 或 Gazebo 时，先根据 `docs/ENVIRONMENT.md` 重新确认实际环境。

## 工作规则

- 只实施用户明确要求的当前阶段内容，避免提前创建架构、工作空间或功能包。
- 未经明确授权，不安装系统或项目依赖，不执行 `sudo apt install`，不修改系统配置。
- 未经明确授权，不创建 ROS 2 工作空间、ROS 2 包、仿真世界或机器人模型。
- 不伪造测试、构建、仿真或硬件实验结果；无法运行时应明确记录原因。
- 新增或变更开发能力时，同步更新环境文档、范围文档或学习日志中相关内容。
- 保持提交内容聚焦，不覆盖或清理与当前任务无关的用户文件。
- 优先采用可复现的命令、配置和实验参数，并记录关键版本与假设。

## 代码约定

以下约定适用于现有和后续 ROS 2 代码：

- ROS 2 包、节点、话题、参数和坐标系名称应语义清晰，并遵循 ROS 2 社区惯例。
- 将故障注入、健康评估、融合算法和导航策略保持为边界清晰、可独立测试的模块。
- 仿真输入、故障场景、随机种子、参数和评价指标应可配置、可追踪、可复现。
- 为关键算法编写单元测试，为跨节点流程编写集成或启动测试。
- 代码变更应附带与当前环境相匹配的验证说明。

## 文档与状态维护

- `README.md`：项目入口、当前状态和文档导航。
- `docs/PROJECT_SCOPE.md`：目标、范围、边界和阶段规划。
- `docs/ENVIRONMENT.md`：实际环境、工具状态和复核方法。
- `docs/LEARNING_LOG.md`：按日期记录事实、决策、经验和待办。

如果实际状态发生变化（例如安装 ROS 2、创建工作空间或完成首个节点），必须删除或更新所有过时的“尚未”描述。
