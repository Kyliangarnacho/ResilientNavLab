# 阶段 3 收尾总结

## 1. 阶段结论

- 收尾日期：2026-07-29
- 阶段名称：虚拟差速机器人与基础运动
- 状态：已完成

阶段 3 已建立一个可复现的 ROS 2 Jazzy / Gazebo Harmonic 差速机器人基线。机器人能够从 Xacro 生成到阶段 2 世界，通过 Gazebo 原生 DiffDrive 和 JointStatePublisher 插件运动，并通过定向 bridge 使用 ROS 2 `/cmd_vel`、`/odom` 和 `/joint_states`。ROS 侧以 `/odom` 为唯一来源发布 `odom -> base_footprint`，RViz 使用同一 TF 链显示机器人状态。

本阶段完成的是低速、短时、平地基础运动链路，不代表完整运动性能、定位或导航能力已经验收。

## 2. 已完成内容

- 新建 `resilient_nav_description` 包，提供基础两轮差速机器人 Xacro、独立 Display Launch 和 RViz 配置。
- 模型包含 `base_footprint`、`base_link`、左右驱动轮和球形支撑轮，以及必要的 visual、collision、inertial、材质和摩擦参数。
- Xacro 集成 Gazebo Harmonic `DiffDrive` 和 `JointStatePublisher` 系统插件，轮距为 `0.39 m`，轮半径为 `0.10 m`。
- `phase3_spawn.launch.py` 复用阶段 2 世界，生成机器人并建立 `/cmd_vel`、`/odom`、`/joint_states` 的定向 ROS—Gazebo bridge。
- `odom_tf_broadcaster` 从 `/odom` 发布 `odom -> base_footprint`，避免桥接 Gazebo 原生 TF 后产生重复来源。
- `phase3_demo.launch.py` 复用完整 Spawn 链，并可选启动以 `odom` 为 Fixed Frame 的 RViz。
- `motion_test` 支持直行、原地旋转和圆弧，正常结束、异常和 Ctrl-C 均重复发送零速度。
- 调整纵向支撑、重心和轮轴参考点，使机器人稳定落地，并让 Gazebo、轮式里程计与 TF 使用一致的 `base_footprint` 语义。

## 3. 最终验收结果

2026-07-29 收尾时执行快速全工作空间复核：

```text
colcon build --symlink-install
Summary: 3 packages finished

colcon test
colcon test-result --verbose
Summary: 43 tests, 0 errors, 0 failures, 1 skipped
```

跳过项是 `resilient_nav_monitor` 包骨架沿用的版权检查配置，不是功能失败。测试覆盖机器人描述、惯性与接触参数、Gazebo 插件、Launch/bridge/RViz 静态契约、odom TF 转换、运动参数校验和自动停车路径。

阶段内已完成并在收尾时保留的运行验收结果：

| 验收项 | 结果 |
| --- | --- |
| 机器人生成 | `ros_gz_sim create` 报告实体创建成功；稳定落地 Z 约为 `-0.000001 m` |
| ROS 2 接口 | `/cmd_vel` 为 `geometry_msgs/msg/Twist`，`/odom` 为 `nav_msgs/msg/Odometry`，`/joint_states` 为 `sensor_msgs/msg/JointState` |
| 关节状态 | 同时包含 `left_wheel_joint` 和 `right_wheel_joint`，停车后速度接近零 |
| TF | `/odom` 经 `odom_tf_broadcaster` 生成 `odom -> base_footprint`，`robot_state_publisher` 继续发布机器人 link TF |
| 直行 | `0.2 m/s`、`1.5 s` 后 `/odom` 约为 `(0.2768, 0, 0)` |
| 原地旋转 | `0.6 rad/s`、`1.5 s` 后 `/odom` yaw 约为 `0.832 rad` |
| 圆弧 | `0.2 m/s`、`0.4 rad/s`、`1.5 s` 后 `/odom` 约为 `(0.2613, 0.0737, 0.550)` |
| 自动停车 | 三种模式结束后的 `/odom` 线速度和角速度均为零；真实 Ctrl-C 路径也验证为零 |
| Gazebo/RViz 同步 | RViz 使用 `/robot_description`、`/tf`、`/tf_static` 和 `odom` Fixed Frame；三模式最大 Gazebo—里程计位置差约 `8 mm`、航向差约 `0.052 rad` |
| 进程清理 | 收尾停止 Launch 后未发现 Gazebo、RViz、bridge、TF 或运动测试进程残留 |

根据收尾指令，没有重复运行完整 Gazebo/RViz 和三组运动流程；上述动态数值沿用阶段内已经完成并记录在环境文档和学习日志中的验收结果。收尾只再次执行快速构建与自动测试。

## 4. 启动与运动测试命令

每个新终端先加载环境：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/kylian/projects/resilient_nav_lab/ros2_ws
source install/setup.bash
```

完整 Gazebo/RViz 演示：

```bash
ros2 launch resilient_nav_simulation phase3_demo.launch.py
```

只启动 Gazebo、机器人和 ROS 链路：

```bash
ros2 launch resilient_nav_simulation phase3_spawn.launch.py
```

在 Launch 运行期间，于另一个已加载环境的终端执行三种短时运动：

```bash
ros2 run resilient_nav_simulation motion_test straight \
  --linear-speed 0.2 --duration 1.5
ros2 run resilient_nav_simulation motion_test spin \
  --angular-speed 0.6 --duration 1.5
ros2 run resilient_nav_simulation motion_test arc \
  --linear-speed 0.2 --angular-speed 0.4 --duration 1.5
```

快速构建与自动测试：

```bash
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

## 5. 主要文件

- `ros2_ws/src/resilient_nav_description/urdf/resilient_nav_robot.urdf.xacro`
- `ros2_ws/src/resilient_nav_description/launch/display.launch.py`
- `ros2_ws/src/resilient_nav_simulation/launch/phase3_spawn.launch.py`
- `ros2_ws/src/resilient_nav_simulation/launch/phase3_demo.launch.py`
- `ros2_ws/src/resilient_nav_simulation/rviz/phase3_demo.rviz`
- `ros2_ws/src/resilient_nav_simulation/scripts/motion_test.py`
- `ros2_ws/src/resilient_nav_monitor/resilient_nav_monitor/odom_tf_broadcaster.py`

## 6. 已知边界与交接

- 当前只验收给定低速、短时、平地运动，不包含速度精度标定、长距离累计误差、轨迹跟踪、高速急停、坡面或控制鲁棒性。
- Gazebo 物理位姿与基于轮转角积分的里程计存在毫米级位置和小角度航向差，当前如实保留，不引入额外定位来源修正。
- Gazebo 原生 TF 没有桥接；`odom -> base_footprint` 的唯一动态来源是 `odom_tf_broadcaster`。
- `system_heartbeat` 在整套 Launch 收到 Ctrl-C 时仍可能报告一次 `rcl_shutdown already called`；该既有退出警告不影响仿真和运动结果，且没有留下后台进程。
- 传感器、`ros2_control`、Nav2、SLAM、定位、故障注入、健康评估、自适应融合和容错导航均未实现，必须在后续阶段单独授权和验收。
