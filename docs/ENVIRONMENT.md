# 开发环境基线

## 核验信息

- 核验日期：2026-07-24
- 项目目录：`/home/kylian/projects/resilient_nav_lab`
- 当前阶段：初始化阶段（第 0 阶段）

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
| ROS 2 CLI | 未找到 `ros2` 命令 | 未安装/不可用 |
| ROS 2 环境 | `ROS_DISTRO` 未设置，`/opt/ros` 不存在 | 未安装/未配置 |
| Gazebo CLI | 未找到 `gz` 或 `gazebo` 命令 | 未安装/不可用 |

运行 `codex --version` 时，Codex 成功返回版本号，同时提示当前受限检查环境无法创建 PATH aliases。该提示不影响本次版本识别；如后续需要诊断 Codex PATH 行为，应在对应任务中单独复核。

## ROS 2 项目状态

当前仓库中只有初始化文档和辅助目录：

- 尚未安装 ROS 2。
- 尚未安装 Gazebo。
- 尚未创建 ROS 2 工作空间。
- 尚未创建任何 ROS 2 包。
- 尚未开始机器人功能开发。
- 当前没有可执行的 ROS 2 构建、测试或启动命令。

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
printenv ROS_DISTRO
test -d /opt/ros
command -v gz
command -v gazebo
```

命令未输出路径或返回非零状态时，应结合其他检查项判断工具是否未安装或未进入当前 shell 的 `PATH`。

## 后续环境决策

进入下一阶段前应单独完成并记录：

1. 根据 Ubuntu 24.04 的官方支持情况选择 ROS 2 发行版。
2. 确认所选 ROS 2 发行版兼容的 Gazebo 版本和安装方式。
3. 约定 ROS 2 工作空间、依赖管理和构建测试流程。
4. 安装完成后更新本页，不保留过时的“未安装”状态。

本次环境核验没有执行安装，也没有修改系统配置。
