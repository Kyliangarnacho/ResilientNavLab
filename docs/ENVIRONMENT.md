# 开发环境

更新日期：2026-09-06。

## 已验证环境

| 项目 | 当前基线 |
| --- | --- |
| 操作系统 | Ubuntu 24.04.4 LTS，x86_64 |
| Python | 3.12.3 |
| ROS 2 | Jazzy |
| Gazebo | Harmonic / Gazebo Sim 8.11.0 |
| `ros_gz` | Jazzy 1.0.22 |
| Nav2 | 1.3.12 |
| Slam Toolbox | 2.8.5 |
| `robot_localization` | 3.8.3 |
| 工作空间 | `/home/kylian/projects/resilient_nav_lab/ros2_ws` |
| ROS 2 package | 13 个 |
| Python 环境 | 仓库根 `.venv`，启用 system site packages |
| 离线 RF | scikit-learn 1.9.0、joblib 1.6.0（仓库根 `.venv`） |

Gazebo Transport 与 ROS 2 Topic 是独立通信域；只有 launch/config 明确建立的 `ros_gz_bridge` 才会
转换消息。TF 也必须保持唯一 owner，不能因为 Gazebo 中存在 pose/TF 就无条件桥接到 ROS。

## 环境加载

仓库根 `.venv/bin/activate` 会加载 Jazzy 和当前 workspace overlay。正常运行前可以使用：

```bash
cd /home/kylian/projects/resilient_nav_lab
source .venv/bin/activate
```

若手工加载：

```bash
set +u
source /opt/ros/jazzy/setup.bash
source /home/kylian/projects/resilient_nav_lab/ros2_ws/install/setup.bash
set -u
```

`set +u` 用于避免某些 ament setup hook 在启用 Bash nounset 时读取未定义变量。

## BRNE Python/Numba 永久构建合同

`resilient_nav_brne` 导入仓库 `.venv` 中的 `numba 0.61.2`。ament Python console script 的 shebang
由运行 colcon 的 Python 决定；仅激活 `.venv` 后调用 `/usr/bin/colcon` 仍会生成系统 Python shebang。

因此，只要构建集合包含 `resilient_nav_brne`，必须从 `ros2_ws/` 使用：

```bash
cd /home/kylian/projects/resilient_nav_lab/ros2_ws
set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u
../.venv/bin/python -m colcon build --symlink-install \
  --packages-select resilient_nav_brne
../.venv/bin/python -m colcon test --packages-select resilient_nav_brne
```

全工作空间构建也必须由同一解释器驱动：

```bash
../.venv/bin/python -m colcon build --symlink-install
```

构建后检查：

```bash
head -1 install/resilient_nav_brne/lib/resilient_nav_brne/brne_shadow_node
```

预期为：

```text
#!/home/kylian/projects/resilient_nav_lab/.venv/bin/python
```

禁止用系统 Python、临时 `PYTHONPATH` 或手改 shebang 绕过此合同。

## 测试约定

- 小改动先运行对应 package 的 targeted pytest。
- 阶段收口再运行相关 package 的 colcon test/build。
- ROS 日志目录在受限环境中使用 `/tmp`：`ROS_LOG_DIR=/tmp/<task-name>`。
- 默认使用 Fake/Mock，不调用真实模型 API。
- Gazebo、DDS 和 GUI 在受限 sandbox 可能因网络接口或显示权限失败；这种失败必须标为
  infrastructure，不得写成算法失败或 PASS。

## 重要运行入口

```bash
# 基础 Gazebo 机器人
ros2 launch resilient_nav_simulation phase3_demo.launch.py

# BRNE V1
ros2 launch resilient_nav_brne brne_sensor_scene1_demo.launch.py arm_brne:=true use_rviz:=true
ros2 launch resilient_nav_brne brne_scene2_demo.launch.py arm_brne:=true use_rviz:=true
ros2 launch resilient_nav_brne brne_scene3_head_on_demo.launch.py arm_brne:=true use_rviz:=true
```

## 当前环境限制

- C920 依赖 WSL/USBIP，仍可能出现闪帧、FPS 波动和偏暗。
- PointCloud2 未桥接。
- 真实机器人、长期运动性能和 fault-aware autonomous navigation 尚未验收。
- 不得为当前任务擅自安装依赖或修改系统配置。
