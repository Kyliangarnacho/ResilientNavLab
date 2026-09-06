# ResilientNavLab

ResilientNavLab 是一个基于 ROS 2 Jazzy 与 Gazebo Harmonic 的移动机器人实验平台，覆盖传感器接入、
故障注入、健康评估、自适应融合、SLAM、Nav2、动态行人交互和离线 Robot Diagnostic Agent。

## 当前状态

| 阶段 | 状态 | 已完成成果 |
| --- | --- | --- |
| 0 | 完成 | 项目范围、环境基线和协作规则 |
| 1 | 完成 | ROS 2 Jazzy、工作空间、基础通信与 `system_heartbeat` |
| 2 | 完成 | Gazebo Harmonic、自定义世界、`/clock` bridge 与仿真时间 |
| 3 | 完成 | 差速机器人 Xacro、Gazebo 插件、`/cmd_vel`、里程计、TF 和基础运动 |
| 4 | 完成 | IMU、二维 LiDAR、RGB-D 与 wheel+IMU EKF 定位基线 |
| 5 | 完成 | IMU/wheel/LiDAR 故障注入、真值接口、faulted EKF、probe 与 rosbag |
| 6 | 完成 | 传感器健康监测、分类、恢复和 evaluator |
| 7 | 完成 | C920 接入、CameraInfo、去畸变、回放与 camera health v1 |
| 8 | 完成 | 健康感知 `FusionPolicy`、measurement adapter、adaptive EKF 与隔离评价 |
| 9 | 完成 | Slam Toolbox 建图、地图持久化、重定位与 loop closure |
| 10 | 已收口 | saved-map Nav2、Costmap、Navfn、RPP、BT、动态障碍、Recovery 与 Goal Cancel |
| BRNE V1 | 已收口 | LiDAR dynamic-agent、统一 interaction/crossing/head-on policy 和 Scene 1/2/3 闭环 |
| RA-1A | 完成 | 离线只读诊断、Sanitizer、Evidence/Incident、Tools、strict output 与 benchmark |

Phase 10 的工程验收为 8/8 valid navigation PASS，另有一次发 goal 前的 infrastructure-invalid trial；
不将其伪装为严格自动 9/9。BRNE V1 是人工场景 baseline，不是统计 benchmark。更精确的当前边界见
[当前状态](docs/CURRENT_STATE.md)。

## 软件包

当前 `ros2_ws/src` 包含 13 个 ROS 2 package：

| 软件包 | 职责 |
| --- | --- |
| `resilient_nav_monitor` | 心跳与 odom TF |
| `resilient_nav_simulation` | Gazebo 世界、spawn、bridge 与运动工具 |
| `resilient_nav_description` | 机器人 Xacro、collision、sensor 与插件 |
| `resilient_nav_localization` | wheel+IMU EKF |
| `resilient_nav_interfaces` | Fault、Health、Fusion、Pedestrian 等消息接口 |
| `resilient_nav_fault_injection` | 故障模型、场景、probe 与 bag 工具 |
| `resilient_nav_health_assessment` | IMU/wheel/scan/camera 健康监测与评价 |
| `resilient_nav_camera` | C920、CameraInfo 与图像处理入口 |
| `resilient_nav_fusion` | 健康感知融合策略、adapter 与 adaptive EKF |
| `resilient_nav_slam` | Slam Toolbox、地图资产与评价工具 |
| `resilient_nav_navigation` | Nav2 localization、Costmap、Planner、Controller、BT 与 benchmark |
| `resilient_nav_brne` | BRNE、LiDAR dynamic-agent、交互策略、控制 gate 与场景 Demo |
| `resilient_nav_agent` | RA-1A 离线 Robot Diagnostic Agent |

## 环境与构建

当前基线为 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic 8.11.0、Nav2 1.3.12、
Slam Toolbox 2.8.5 和 Python 3.12。完整约束见 [开发环境](docs/ENVIRONMENT.md)。

完整工作空间构建统一使用仓库根 `.venv` 的 Python，避免重建 BRNE 时生成错误的
console-script shebang：

```bash
cd /home/kylian/projects/resilient_nav_lab/ros2_ws
source /opt/ros/jazzy/setup.bash
../.venv/bin/python -m colcon build --symlink-install
source install/setup.bash
```

只重建 `resilient_nav_brne` 时使用：

```bash
../.venv/bin/python -m colcon build --symlink-install \
  --packages-select resilient_nav_brne
```

`resilient_nav_brne` 依赖仓库根 `.venv` 中的 Numba；禁止用裸 `colcon` 或
`/usr/bin/colcon` 重建该包。构建后应检查其可执行文件首行指向 `.venv/bin/python`。

## 关键入口

```bash
# BRNE Scene 1 / 2 / 3
ros2 launch resilient_nav_brne brne_sensor_scene1_demo.launch.py arm_brne:=true use_rviz:=true
ros2 launch resilient_nav_brne brne_scene2_demo.launch.py arm_brne:=true use_rviz:=true
ros2 launch resilient_nav_brne brne_scene3_head_on_demo.launch.py arm_brne:=true use_rviz:=true

# 同场景 RPP 对照
ros2 launch resilient_nav_brne rpp_scene1_comparison_demo.launch.py use_rviz:=true
```

## 文档

### 长期入口与合同

- [项目范围](docs/PROJECT_SCOPE.md)
- [当前状态](docs/CURRENT_STATE.md)
- [开发环境](docs/ENVIRONMENT.md)
- [学习与决策日志](docs/LEARNING_LOG.md)
- [Robot Agent 开发规则](docs/ROBOT_AGENT_DEV_RULES.md)
- [开源项目基线](docs/OPEN_SOURCE_BASELINES.md)

### 阶段总结

- [阶段 2](docs/PHASE2_SUMMARY.md) · [阶段 3](docs/PHASE3_SUMMARY.md) ·
  [阶段 4](docs/PHASE4_SUMMARY.md) · [阶段 5](docs/PHASE5_SUMMARY.md) ·
  [阶段 6](docs/PHASE6_SUMMARY.md) · [阶段 7](docs/PHASE7_SUMMARY.md)
- [阶段 8](docs/PHASE8_SUMMARY.md) · [阶段 9](docs/PHASE9_SUMMARY.md) ·
  [阶段 10](docs/PHASE10_SUMMARY.md) · [BRNE V1](docs/BRNE_V1_SUMMARY.md) ·
  [RA-1A](docs/RA1A_SUMMARY.md)

需要长期保留的原始实验数据不放在 `docs/`，统一按阶段存入 [data](data/README.md)。

## 当前未完成

- fault-aware navigation 与 Health/Fusion 到 Nav2 的正式自适应闭环；
- Live ROS Robot Agent、Planner/Recovery 权限和真实机器人执行；
- RGB-D/PointCloud2 完整处理与视觉故障模型；
- BRNE 的人类分类、遮挡续接、统计 benchmark 和完整 footprint 安全证明。
