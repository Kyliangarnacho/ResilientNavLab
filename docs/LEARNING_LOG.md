# ResilientNavLab 学习与决策日志

本日志按日期记录项目中的事实、判断、经验和后续问题。尚未实施或验证的内容应标记为计划或待办。

## 2026-07-27 — 阶段 2 Gazebo 基础仿真与时钟链路收尾

### 当前事实

- 已通过现有 ROS 2 Jazzy 软件源安装 `ros-jazzy-ros-gz`，Gazebo Sim 版本为 8.11.0；安装和验证日志分别保存在 `docs/gazebo_install_20260727.log` 与 `docs/gazebo_verify_20260727.log`。
- 官方 `shapes.sdf` 已通过脚本完成限时、无 GUI 的服务端启动检查，用户另行确认其图形世界正常打开。
- 已创建 `resilient_nav_simulation` 包，包含 `phase2_world.sdf`、`bridge.yaml`、`phase2_world.launch.py` 和静态资源测试。
- 用户确认自定义世界正常打开，并可见 `ground_plane`、`box_obstacle` 和 `cylinder_checkpoint`。
- 未启动桥接时，Gazebo Transport 可观察 `/clock`，ROS 2 不可观察 `/clock`；启动项目 Launch 后，ROS 2 可以观察 `/clock`。
- Launch 将 `system_heartbeat` 的 `use_sim_time` 设置为 `true`。暂停 Gazebo 时 ROS 2 `/clock` 和心跳停止，恢复 Gazebo 后二者继续，验证了仿真时钟驱动关系。
- 用户将 `box_obstacle` 的 pose 从 `2 0 0.5 0 0 0` 修改为 `3 1 0.5 0 0 0`，重新启动后确认 Gazebo 中的新坐标生效。

### 学习要点

- Gazebo Transport 和 ROS 2 Topic 是两套独立通信机制。即使两侧都使用 `/clock` 这一名称，也必须由 `ros_gz_bridge` 显式转换消息后才能互通。
- ROS 2 负责节点、参数和 ROS 图，Gazebo 负责世界与仿真时间，`ros_gz` 负责启动集成和选定数据的桥接；明确边界有助于定位“Gazebo 有数据但 ROS 2 看不到”的问题。
- SDF 定义 Gazebo 世界，`bridge.yaml` 定义跨中间件消息映射，Python Launch 文件负责把 Gazebo、桥和 ROS 2 节点组织成一次可复现启动。
- `src/` 是源码，`build/` 是中间产物，`install/` 是运行时可发现前缀，`log/` 保存构建和测试日志。`colcon build --symlink-install` 建立便于迭代的安装布局，而 `source install/setup.bash` 只是把该布局加载到当前 shell。
- 通过 ROS vendor 包安装的 `gz` 位于 `/opt/ros/jazzy/opt/gz_tools_vendor/bin`。安装前已经加载过 ROS 环境的 shell 不会自动获得后来新增的路径，需要重新加载 `/opt/ros/jazzy/setup.bash` 或打开新的已配置 shell；因此当时的 `gz` 命令不可见属于环境未刷新，而非安装失败。

### 当前边界

- 阶段 2 只完成静态世界、`/clock` 桥接、Launch 编排和已有心跳节点的仿真时间联动。
- 没有开始机器人、URDF、传感器、运动控制、Nav2、SLAM、故障注入、健康评估、自适应融合或容错导航。
- 本次收尾只更新文档并执行非图形验证，不安装软件、不启动 Gazebo 图形界面，也不修改 `resilient_nav_monitor` 或 `resilient_nav_simulation` 源码。

## 2026-07-24 — 实现 system_heartbeat 节点

### 当前事实

- `resilient_nav_monitor` 新增 `system_heartbeat` console 入口和同名节点。
- 节点每 1 秒在 `/system_heartbeat` 发布一次 `std_msgs/msg/String`，消息为 `alive count=N`，计数从 1 开始递增。
- 聚焦测试覆盖话题、周期和递增消息格式；完整测试结果为 4 项通过、1 项按生成器默认配置跳过、0 项失败。
- `colcon build --symlink-install` 成功完成 1 个包。
- 自动运行验证确认节点、话题类型和消息均符合约定，`ros2 topic hz` 测得频率为 `1.000 Hz`。
- 节点由 timeout 和清理逻辑自动停止，验证后没有后台进程残留。

### 当前边界

- `system_heartbeat` 只提供基础存活信号，不代表健康评估、故障检测或容错决策已经实现。
- 本次没有创建其他包或节点，没有安装软件，也没有安装 Gazebo、Nav2 或 SLAM。

## 2026-07-24 — 创建 resilient_nav_monitor 包骨架

### 当前事实

- 在 `ros2_ws/src/` 中创建了 `resilient_nav_monitor`，构建类型为 `ament_python`，许可证为 Apache-2.0。
- 包清单声明 `rclpy` 和 `std_msgs` 依赖，Python 打包配置、ament 资源索引和标准测试目录已建立。
- `colcon build --symlink-install` 成功完成 1 个包。
- 包标准测试结果为 2 项通过、1 项按生成器默认配置跳过、0 项失败。
- 加载工作空间环境后，`ros2 pkg prefix resilient_nav_monitor` 成功返回工作空间安装前缀。

### 当前边界

- 当前只创建规范包骨架，没有实现 `system_heartbeat`、其他节点或任何机器人功能。
- 本次没有安装软件，没有安装 Gazebo、Nav2 或 SLAM。
- 后续节点接口、行为和测试应在单独任务中明确设计和实现。

## 2026-07-24 — 阶段 1 ROS 2 工作空间基线

### 当前事实

- 已创建 `ros2_ws/src/`，并以 `.gitkeep` 保留空源码目录。
- 加载 ROS 2 Jazzy 环境后，在 `ros2_ws` 中执行空 `colcon build` 成功，结果为 `0 packages finished`。
- `ros2_ws/build/`、`ros2_ws/install/` 和 `ros2_ws/log/` 均由现有 `.gitignore` 规则排除，没有进入 Git。
- 当前尚未创建任何 ROS 2 包、项目节点、消息、启动文件或参数。

### 当前边界

- 本次没有安装软件，没有创建 ROS 2 包，也没有启动机器人功能开发。
- Gazebo、Nav2 和 SLAM 仍未安装。
- 后续首个功能包及其接口、构建和测试需要在单独任务中明确实施。

## 2026-07-24 — ROS 2 Jazzy 安装与基础通信验证

### 当前事实

- ROS 2 Jazzy 已安装，环境变量为 `ROS_DISTRO=jazzy`、`ROS_VERSION=2`。
- `ros2`、`colcon` 和 `rosdep` 命令均可用。
- 官方 `demo_nodes_cpp talker` 和 `demo_nodes_py listener` 通信验证通过。
- 运行期间 `/talker` 和 `/listener` 节点均存在，`/chatter` 话题类型为 `std_msgs/msg/String`。
- Gazebo、Nav2 和 SLAM 仍未安装。
- ROS 2 工作空间和项目 ROS 2 包仍未创建。

### 问题与修复

- 验证脚本启用了 `set -euo pipefail`。首次直接加载官方 `/opt/ros/jazzy/setup.bash` 时，官方脚本引用未定义的 `AMENT_TRACE_SETUP_FILES`，触发 nounset 错误。
- 修复仅作用于环境加载过程：加载前执行 `set +u`，加载后立即恢复 `set -u`；脚本仍保留 `set -e`、`set -u` 和 `pipefail`。
- 修复后验证脚本、详细 `ros2 doctor --report` 和官方 talker/listener 通信实验均完成。通信实验使用超时和清理机制，未留下后台演示节点。

### 当前边界

- 本次没有重新安装 ROS 2，也没有执行 apt、dpkg 或 sudo 安装。
- 本次没有安装 Gazebo、Nav2、SLAM 或其他 ROS 发行版。
- 本次没有创建工作空间、ROS 2 包或项目节点。
- 后续机器人仿真、导航和容错功能仍为未实施计划。

## 2026-07-24 — 第 0 阶段文档收尾

### 当前事实

- 项目名称确定为 **ResilientNavLab**。
- 项目目标是构建基于 ROS 2 的移动机器人多传感器故障注入、健康评估、自适应融合和容错导航平台。
- 项目仍处于初始化阶段。
- 当前环境为 Ubuntu 24.04.4 LTS，CPU 架构为 x86_64/amd64。
- Git 2.43.0、Python 3.12.3、Node.js v24.18.0、npm 11.16.0 和 Codex CLI 0.145.0 可用。
- 当前只有 `python3` 命令，没有 `python` 命令。
- ROS 2 与 Gazebo 均未安装或不可用。
- 尚未创建 ROS 2 工作空间和功能包，机器人功能开发尚未开始。

### 本阶段决策

- 先建立清晰的项目范围、环境基线和协作规则，再进入软件安装与代码开发。
- 不在第 0 阶段提前选择 ROS 2 发行版或 Gazebo 版本；选择必须基于 Ubuntu 24.04 的官方兼容关系另行确认。
- 后续设计采用模块化边界：故障注入、健康评估、自适应融合和容错导航应能分别开发、测试和对比。
- 实验设计必须重视可复现性，至少记录场景、参数、随机种子、故障真值和评价指标。

### 本次完成

- 补充项目入口和当前状态说明。
- 定义项目目标、计划内能力、当前边界与建议阶段路线。
- 记录系统、工具、ROS 2 和 Gazebo 的实际状态。
- 建立仓库协作规则和面向未来构建产物的忽略规则。

### 学习要点

- “命令不存在”与“环境未加载”可能表现相同，因此 ROS 2 状态同时通过 `ros2`、`ROS_DISTRO` 和 `/opt/ros` 交叉核验。
- 在创建工作空间前记录系统与工具版本，可以为后续依赖选择、问题复现和迁移提供基准。
- 项目文档必须区分“目标”“计划”和“已实现”，避免初始化仓库给出错误的功能完成预期。

### 后续待办（未执行）

- 调研并选择适配 Ubuntu 24.04 的 ROS 2 发行版。
- 确认 ROS 2 与 Gazebo 的官方兼容组合。
- 经明确授权后安装依赖并创建 ROS 2 工作空间。
- 设计首个最小可运行的仿真和数据链路里程碑。

### 操作声明

本次仅执行只读环境检查并修改项目文档；未安装任何软件，未执行 `sudo apt install`，未创建 ROS 2 工作空间或包，也未修改系统配置。
