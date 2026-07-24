# 开发环境基线

## 核验信息

- 核验日期：2026-07-24
- 项目目录：`/home/kylian/projects/resilient_nav_lab`
- 当前阶段：阶段 1（基础设施基线）

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
| Gazebo CLI | 未找到 `gz` 或 `gazebo` 命令 | 未安装/不可用 |

运行 `codex --version` 时，Codex 成功返回版本号，同时提示当前受限检查环境无法创建 PATH aliases。该提示不影响本次版本识别；如后续需要诊断 Codex PATH 行为，应在对应任务中单独复核。

## ROS 2 项目状态

当前 ROS 2 基础环境状态如下：

- 已安装 ROS 2 Jazzy。
- `ros2`、`colcon` 和 `rosdep` 均可用。
- 官方 C++ talker 能通过 `/chatter` 发布 `std_msgs/msg/String`，Python listener 能正常接收。
- 已创建 `ros2_ws`；其 `src/` 当前包含 `resilient_nav_monitor` 包骨架。
- 在加载 ROS 2 Jazzy 环境后执行空 `colcon build` 成功，结果为 `0 packages finished`。
- `resilient_nav_monitor` 使用 `ament_python`、Apache-2.0 许可证，并声明 `rclpy` 和 `std_msgs` 依赖。
- `colcon build --symlink-install` 成功完成 1 个包；当前自动测试结果为 4 项通过、1 项按生成器默认配置跳过、0 项失败。
- 加载 `ros2_ws/install/setup.bash` 后，`ros2 pkg prefix resilient_nav_monitor` 返回工作空间内的安装前缀。
- `system_heartbeat` 节点以 1 Hz 在 `/system_heartbeat` 发布 `std_msgs/msg/String`，消息格式为 `alive count=N`，其中计数持续递增。
- 自动运行验证确认 `/system_heartbeat` 节点存在、话题类型正确、消息可接收，测得频率为 `1.000 Hz`；验证结束后没有节点进程残留。
- `ros2_ws/build/`、`ros2_ws/install/` 和 `ros2_ws/log/` 是本地构建产物，均由 `.gitignore` 排除。
- 尚未安装 Gazebo。
- 尚未安装 Nav2 和 SLAM。
- 尚未实现 `system_heartbeat` 以外的项目节点。
- 尚未开始故障注入、健康评估、融合或导航功能开发。

验证脚本最初在启用 `set -u` 时直接加载官方 `/opt/ros/jazzy/setup.bash`，因官方脚本引用未定义的 `AMENT_TRACE_SETUP_FILES` 而失败。现已将加载过程限定为临时执行 `set +u`，加载完成后立即恢复 `set -u`，并保留 `set -e` 和 `pipefail`。

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
command -v gazebo
```

命令未输出路径或返回非零状态时，应结合其他检查项判断工具是否未安装或未进入当前 shell 的 `PATH`。

## 后续环境决策

后续环境工作应单独完成并记录：

1. 确认 ROS 2 Jazzy 兼容的 Gazebo 版本和安装方式。
2. 约定 ROS 2 工作空间、依赖管理和构建测试流程。
3. 经明确授权后安装 Gazebo、Nav2 或 SLAM 等后续依赖。
4. 经明确授权后设计 `resilient_nav_monitor` 的后续健康监测接口和节点。

ROS 2 Jazzy 已安装，并已在 `~/.bashrc` 中幂等配置其环境加载；`ros2_ws`、`resilient_nav_monitor` 和 `system_heartbeat` 均已通过验证；Gazebo、Nav2 和 SLAM 未安装。
