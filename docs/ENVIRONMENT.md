# 开发环境基线

## 核验信息

- 最近核验日期：2026-07-27
- 项目目录：`/home/kylian/projects/resilient_nav_lab`
- 当前阶段：阶段 2（Gazebo 基础仿真与 ROS 2 时钟链路）已完成

本页记录核验时的实际环境，不代表未来项目最终采用的依赖组合。

## 系统与工具状态

| 项目 | 检测结果 | 状态 |
| --- | --- | --- |
| Ubuntu | Ubuntu 24.04.4 LTS（Noble Numbat） | 可用 |
| CPU 架构 | `x86_64`；Debian 架构名为 `amd64` | 可用 |
| Git | `git version 2.43.0`，路径 `/usr/bin/git` | 可用 |
| Python 3 | `Python 3.12.3`，路径 `/usr/bin/python3` | 可用 |
| `python` 命令 | 未找到 | 不可用；当前使用 `python3` |
| Node.js | `v24.18.0`，路径 `/usr/bin/node` | 可用 |
| npm | `11.16.0`，路径 `/usr/bin/npm` | 可用 |
| Codex CLI | `codex-cli 0.145.0`，路径 `/home/kylian/.npm-global/bin/codex` | 可用 |
| ROS 2 CLI | ROS 2 Jazzy，路径 `/opt/ros/jazzy/bin/ros2` | 可用 |
| ROS 2 环境 | `ROS_DISTRO=jazzy`，`ROS_VERSION=2` | 已安装并在交互式 Bash 中配置 |
| ROS 2 开发工具 | `colcon` 路径 `/usr/bin/colcon`；`rosdep` 路径 `/usr/bin/rosdep` | 可用 |
| ROS 2 基础通信 | 官方 `demo_nodes_cpp talker` 与 `demo_nodes_py listener` | 通信验证通过 |
| ROS 2 工作空间 | `/home/kylian/projects/resilient_nav_lab/ros2_ws` | 已创建；空构建和 `colcon build --symlink-install` 均通过 |
| 项目 ROS 2 包 | `resilient_nav_monitor`（`ament_python`） | 构建、自动测试和 `ros2 pkg prefix` 发现验证通过 |
| 项目 ROS 2 节点 | `system_heartbeat` | 1 Hz 发布和 ROS 图端到端验证通过 |
| 仿真资源包 | `resilient_nav_simulation`（`ament_cmake`） | 构建、静态测试和资源安装验证通过 |
| Gazebo | Gazebo Harmonic；Gazebo Sim `8.11.0` | `gz` 可用，官方和项目世界均已验证 |
| ROS 2—Gazebo 集成 | `ros-jazzy-ros-gz` `1.0.22` | 已安装，核心 `ros_gz` 包可被 ROS 2 发现 |
| 仿真时钟链路 | Gazebo `/clock` → ROS 2 `/clock` | 单向桥接、暂停/恢复和 `use_sim_time` 联动验证通过 |

运行 `codex --version` 时，Codex 成功返回版本号，同时提示当前受限检查环境无法创建 PATH aliases。该提示不影响本次版本识别；如后续需要诊断 Codex PATH 行为，应在对应任务中单独复核。

## 软件职责边界

- **ROS 2 Jazzy** 负责项目节点、参数、ROS 图、ROS 2 Topic 和节点间通信。`system_heartbeat` 属于这一侧。
- **Gazebo Harmonic** 负责加载 SDF 世界、维护仿真状态、推进或暂停仿真时间，并通过 Gazebo Transport 发布仿真数据。
- **`ros_gz`** 是两套中间件之间的集成层。`ros_gz_sim` 负责从 ROS 2 Launch 启动 Gazebo，`ros_gz_bridge` 负责按配置转换消息；它不会把所有 Gazebo 话题自动变成 ROS 2 Topic。

Gazebo Transport 和 ROS 2 Topic 是彼此独立的通信域。`gz topic -l` 看到的 `/clock` 是 Gazebo Transport 话题，`ros2 topic list` 看到的是 ROS 2 Topic；名称相同并不代表数据天然互通。未启动桥接时，Gazebo 一侧可观察到 `/clock`，ROS 2 一侧看不到它。当前 `bridge.yaml` 显式把 `gz.msgs.Clock` 单向转换为 `rosgraph_msgs/msg/Clock`，桥接启动后 ROS 2 才能订阅 `/clock`。

## 阶段 2 资源与启动链路

- `worlds/phase2_world.sdf` 使用 SDF 描述世界、物理参数、光源以及 `ground_plane`、`box_obstacle`、`cylinder_checkpoint` 三个静态模型。SDF 是 Gazebo 的仿真输入，不是 ROS 2 节点配置。
- `config/bridge.yaml` 只声明 `/clock` 的 `GZ_TO_ROS` 单向桥接、两侧消息类型和时钟 QoS。
- `launch/phase2_world.launch.py` 解析已安装包的共享目录，启动自定义世界、`ros_gz_bridge` 和 `system_heartbeat`，并为心跳节点设置 `use_sim_time=true`。
- `resilient_nav_simulation` 只提供仿真资源和启动集成，不包含机器人、传感器、导航或故障注入实现。

## 工作空间目录与构建流程

`ros2_ws` 下四个目录的关系如下：

| 目录 | 作用 |
| --- | --- |
| `src/` | 受版本控制的包源码和资源，是开发时应修改的来源。 |
| `build/` | `colcon` 为每个包生成的中间构建目录。 |
| `install/` | 构建后的可发现安装前缀；ROS 2 运行时使用这里的包索引、可执行文件和共享资源。 |
| `log/` | `colcon build`、`colcon test` 和结果汇总产生的本地日志。 |

`build/`、`install/` 和 `log/` 都是可重新生成的本地产物，并由 `.gitignore` 排除；它们不是 `src/` 的替代品。

- `colcon build` 解析 `src/` 中的包并生成 `build/`、`install/` 和构建日志。
- `--symlink-install` 尽可能在 `install/` 中建立指向源码或构建产物的符号链接，便于 Python 和资源文件修改后的快速迭代；构建系统或安装规则变化后仍应重新构建。
- `source /opt/ros/jazzy/setup.bash` 加载 ROS 2 及其 vendor 环境；`source ros2_ws/install/setup.bash` 再把当前工作空间叠加到环境中。`source` 只改变当前 shell，不执行构建，也不会自动影响其他已打开的 shell。

当前工作空间包含两个包：

- `resilient_nav_monitor`：阶段 1 的心跳节点。
- `resilient_nav_simulation`：阶段 2 的 Gazebo 世界、桥接和 Launch 资源。

## 安装、桥接与人工验收

- `scripts/install_gazebo_harmonic.sh` 通过已有 ROS 2 软件源安装 `ros-jazzy-ros-gz`；安装日志为 `docs/gazebo_install_20260727.log`。
- `scripts/verify_gazebo_harmonic.sh` 验证 ROS 2 环境、`gz`、Gazebo Sim 版本、核心 `ros_gz` 包和官方 `shapes.sdf` 的无 GUI 服务端启动；日志为 `docs/gazebo_verify_20260727.log`。
- 安装与脚本日志确认 `ros-jazzy-ros-gz` 安装完成，Gazebo Sim 版本为 8.11.0，`ros_gz`、`ros_gz_sim`、`ros_gz_bridge`、`ros_gz_interfaces` 可发现。
- 用户人工确认官方 `shapes.sdf` 图形世界和项目自定义世界正常打开，自定义世界中的三个静态实体可见。
- 用户人工确认：不启动桥接时，Gazebo Transport 可见 `/clock` 而 ROS 2 不可见；启动项目 Launch 后，ROS 2 可见 `/clock`，且 `/system_heartbeat` 的 `use_sim_time` 为 `true`。
- 用户人工确认：暂停 Gazebo 后 ROS 2 `/clock` 和心跳停止，恢复后两者继续。
- 用户把 `box_obstacle` 的 pose 从 `2 0 0.5 0 0 0` 改为 `3 1 0.5 0 0 0`，重新启动后在 Gazebo 中确认新坐标生效。

完整人工验收清单见 `docs/PHASE2_SUMMARY.md`。

## 已知环境加载问题

本阶段安装完成后曾出现当前 shell 找不到 `gz` 的现象。通过 ROS 2 Jazzy 的 vendor 包安装时，`gz` 位于 `/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz`，不在系统默认的 `/usr/bin`。该目录由 `/opt/ros/jazzy/setup.bash` 的环境钩子加入 `PATH`；如果 shell 在安装前已经加载过 ROS 环境，或当前 shell 根本没有加载 ROS 环境，新路径不会自动出现。重新打开已配置的交互式 Bash，或在当前 shell 中重新执行：

```bash
source /opt/ros/jazzy/setup.bash
```

即可让 `gz` 可见。这是 shell 环境未刷新，不是 Gazebo 安装失败。脚本在 `set -u` 下加载官方 setup 文件时仍临时使用 `set +u`，以避免官方脚本引用未定义的 `AMENT_TRACE_SETUP_FILES`。

## 复核命令

以下均为只读检查命令：

```bash
sed -n '1,20p' /etc/os-release
lsb_release -a
uname -m
dpkg --print-architecture
git --version
python3 --version
node --version
npm --version
codex --version
command -v ros2
command -v colcon
command -v rosdep
printenv ROS_DISTRO
printenv ROS_VERSION
test -f /opt/ros/jazzy/setup.bash
ros2 pkg executables demo_nodes_cpp
ros2 pkg executables demo_nodes_py
command -v gz
gz sim --versions
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix ros_gz_bridge
```

项目工作空间复核：

```bash
cd /home/kylian/projects/resilient_nav_lab/ros2_ws
colcon list
colcon build --symlink-install
colcon test
colcon test-result --verbose
source install/setup.bash
ros2 pkg prefix resilient_nav_monitor
ros2 pkg prefix resilient_nav_simulation
```

SDF 语义检查：

```bash
gz sdf -k src/resilient_nav_simulation/worlds/phase2_world.sdf
```

命令未输出路径或返回非零状态时，应先确认 ROS 2 环境是否已在当前 shell 中加载，再判断软件是否未安装。

## 当前边界

阶段 2 已完成静态世界、Gazebo—ROS 2 `/clock` 桥和已有心跳节点的仿真时间联动。机器人模型、URDF、传感器、运动插件、Nav2、SLAM、故障注入、健康评估、自适应融合和容错导航均未开始，也不应从当前基础世界推断出这些能力已经存在。
