# 阶段 4 轮式里程计与 IMU EKF 基线

## 1. 范围与结论

- 核验日期：2026-08-01
- ROS 2：Jazzy
- `robot_localization`：3.8.3
- 完整启动入口：`phase4_ekf_demo.launch.py`
- EKF 配置：`resilient_nav_localization/config/ekf.yaml`
- RViz 配置：`phase4_sensors.rviz`

本基线在既有 IMU、二维 Lidar 和 RGB-D 数据链之上，新增轮式里程计与
IMU 的最小二维 EKF。阶段 4 完整 Launch 中，Gazebo DiffDrive 的原始里程计
只映射为 `/wheel/odometry`，`robot_localization` 输出
`/odometry/filtered` 并独占 `odom -> base_footprint` 动态 TF。阶段 3 的
`phase3_demo.launch.py` 默认行为保持为 `/odom` 加既有
`odom_tf_broadcaster`。

本任务没有启动 SLAM、Nav2，没有加入 PointCloud2、目标识别、故障注入、
健康评估、自适应融合或机械臂功能。

## 2. 数据链与接口语义

```text
Gazebo DiffDrive /model/resilient_nav_robot/odometry
  -> ros_gz_bridge remap -> /wheel/odometry ─┐
                                             ├-> robot_localization ekf_node
Gazebo IMU /imu/data -> ros_gz_bridge -------┘       ├-> /odometry/filtered
                                                     └-> odom -> base_footprint TF
```

`/wheel/odometry` 是 DiffDrive 根据同一组左右轮运动积分得到的原始观测，保留
`nav_msgs/msg/Odometry` 类型、`header.frame_id=odom` 和
`child_frame_id=base_footprint`。它是 EKF 输入，不再占用阶段 4 完整链的
`/odom` 名称。

`/odometry/filtered` 是 EKF 状态估计输出。把两者分开能让数据来源和处理层级
清晰可见，便于以后比较、故障注入和健康评估，也避免把原始消息误认为融合
结果。阶段 4 完整 Launch 中没有第二条原始 `/odom`。

## 3. Launch 复用与阶段 3 兼容性

`resilient_nav_localization/launch/phase4_ekf_demo.launch.py` Include 既有
`resilient_nav_simulation/launch/phase4_rgbd_demo.launch.py`，只覆盖两个贯穿
现有 Launch 链的参数：

- `odom_ros_topic=/wheel/odometry`；
- `start_odom_tf_broadcaster=false`。

随后新增一个 `robot_localization/ekf_node`。世界、机器人生成、时钟、
`/cmd_vel`、`/joint_states`、`/imu/data`、`/scan`、四个 RGB-D 接口和 RViz
仍由已验收的 Launch 链负责，没有复制第三阶段启动结构。

两个新参数在阶段 3 至 RGB-D Launch 链中的默认值仍分别是 `/odom` 和
`true`。单独动态启动 `phase3_demo.launch.py use_rviz:=false` 时，实际 ROS 图
仍有 `/odom` 和 `/odom_tf_broadcaster`，没有 `/wheel/odometry`、
`/odometry/filtered` 或 EKF，且原始消息 frame 保持 `odom` /
`base_footprint`。

## 4. EKF frame、频率与融合字段

核心参数如下：

| 参数 | 值 | 原因 |
| --- | --- | --- |
| `use_sim_time` | `true` | 与 Gazebo `/clock` 同步 |
| `frequency` | `20.0 Hz` | 在当前 RGB-D/GPU Lidar 图形负载下可按仿真时间稳定达到 |
| `two_d_mode` | `true` | 当前机器人只验收平面差速运动 |
| `world_frame` / `odom_frame` | `odom` | 当前没有全局绝对定位源 |
| `base_link_frame` | `base_footprint` | 与 DiffDrive、阶段 3 TF 和地面运动参考点一致 |
| `publish_tf` | `true` | EKF 是阶段 4 的 odom TF 唯一发布者 |
| `odom0` | `/wheel/odometry` | 原始轮式输入 |
| `imu0` | `/imu/data` | Gazebo IMU 输入 |
| 输出 | `/odometry/filtered` | 明确区分原始与融合结果 |

15 项配置向量顺序为位置 xyz、姿态 roll/pitch/yaw、线速度 xyz、角速度
roll/pitch/yaw、线加速度 xyz。本基线只启用：

- `odom0_config[6]`：轮式前向速度 `vx`。DiffDrive 的 pose 与 twist 来自同一
  对轮编码来源，不同时融合 pose 和 twist，避免无理由重复计入同一信息。
- `imu0_config[11]`：IMU 偏航角速度 `vyaw`。该字段直接约束平面转动，并且
  动态消息具有非零角速度协方差。

实际 IMU 消息的 orientation covariance 为全零；这不能解释为经过标定的
高精度姿态，因此本基线不融合绝对 yaw。线加速度包含约 `9.8 m/s²` 的重力，
其方向、去重力和真实协方差尚未形成独立验收，因此三个加速度字段均关闭，
`imu0_remove_gravitational_acceleration=false`。项目没有伪造 process、initial
或输入 covariance。

最初按建议试用 30 Hz，但并发 RGB-D、GPU Lidar、RViz 和检查订阅时诊断持续
报告更新率低于阈值。改为 20 Hz 后，80 个连续输出样本的仿真时间戳频率为
`20.000 Hz`；墙钟到达率约 `12.470 Hz`，同轮 Gazebo
`real_time_factor≈0.6745`。因此 20 Hz 是仿真时间配置与验收值，不把低实时
因子下的墙钟频率伪写为 20 Hz。

## 5. TF 发布权切换

阶段 4 EKF Launch 显式关闭既有 `odom_tf_broadcaster`，由 EKF 发布
`odom -> base_footprint`。动态检查结果：

- 节点列表有 `/ekf_filter_node`，没有 `/odom_tf_broadcaster`；
- `/tf` publisher endpoint 只有 `robot_state_publisher` 和
  `ekf_filter_node`；前者负责关节/机器人树，后者负责 odom 动态 TF；
- EKF 节点订阅 `/wheel/odometry` 和 `/imu/data`，发布
  `/odometry/filtered` 与 `/tf`；
- 连续 TF 可读取，没有第二个 odom TF 发布源或明显跳变。

此切换仅由 Launch 参数控制，没有删除或修改 `resilient_nav_monitor` 节点，
所以阶段 3 独立运行仍由旧 broadcaster 发布 TF。

## 6. RViz 配置与运行证据

`phase4_sensors.rviz` 保持 Fixed Frame `odom`，并保留启用的 RobotModel、TF、
Best Effort `/scan` LaserScan 和 `/camera/color/image_raw` 彩色 Image。新增：

- 启用的 `Filtered Odometry`，topic 为 `/odometry/filtered`；
- 默认关闭的 `Raw Wheel Odometry`，topic 为 `/wheel/odometry`。

完整 Launch 默认加载此配置。运行时 RViz 完成 OpenGL 初始化，并实际订阅
`/robot_description`、`/scan`、`/camera/color/image_raw` 和
`/odometry/filtered`，同时创建 TF listener。自动化环境没有保存窗口截图，
因此结论限于配置加载、进程状态、订阅关系和日志无持续显示错误；画面内容的
最终人工视觉检查仍列为待办。

## 7. 构建、自动测试和动态验收

### 7.1 静态与自动验证

- Xacro 展开与 `check_urdf`：通过；
- 四个工作空间包 `colcon build --symlink-install`：通过；
- 完整工作空间测试：64 tests，0 errors，0 failures，1 skipped；
- 新包配置/Launch 及受影响 Launch 的 Python、YAML、XML 语法：通过；
- 相关 Python 文件 `ament_flake8`：7 files checked，no problems found；
- `resilient_nav_localization` 可由 ROS 2 发现，安装区存在 Launch 和 YAML；
- `git diff --check`：通过。

自动测试覆盖 EKF YAML 解析、frame、二维模式、仿真时间、TF 开关、输入话题、
融合向量、filtered 输出、wheel odometry remap、旧 broadcaster 条件关闭、
阶段 3 默认行为、IMU/Lidar/RGB-D/RViz 资源保留及资源安装规则。

### 7.2 接口和消息

运行真实入口：

```bash
ros2 launch resilient_nav_localization phase4_ekf_demo.launch.py
```

ROS 图同时存在 `/wheel/odometry`、`/imu/data`、`/scan`、四个 RGB-D 接口、
`/odometry/filtered`、`/cmd_vel` 和 `/joint_states`，且没有原始 `/odom`。
原始与 filtered 消息均为 `frame_id=odom`、
`child_frame_id=base_footprint`。

### 7.3 直行、旋转和连续性

使用既有 `motion_test` 完成 0.2 m/s、2 s 直行和 0.6 rad/s、2 s 原地旋转，
工具在每段结束后自动发布零速度。

| 检查点 | `/wheel/odometry` | `/odometry/filtered` | 判断 |
| --- | ---: | ---: | --- |
| 直行后 x | 0.254200 m | 0.253207 m | 同向合理增长，相差约 0.993 mm |
| 直行后 yaw | 约 0 rad | 约 -0.000036 rad | 无异常漂移 |
| 旋转后 x | 0.254200 m | 0.253207 m | 原地旋转时位置保持 |
| 旋转后 yaw | 约 0.851 rad | 约 0.799 rad | 同向合理增长，相差约 0.052 rad |

停止后抽取 80 个 filtered 样本，所有状态均为有限数；最大相邻位置步长约
`2.4e-19 m`，最大相邻 yaw 步长约 `1.03e-5 rad`，没有 NaN 或突然的大幅
跳变。

### 7.4 暂停、恢复和停止

Gazebo pause 服务返回 `data: true`。暂停后的 5 秒窗口内，
`/wheel/odometry`、`/imu/data` 和 `/odometry/filtered` 都没有新消息；`/clock`
仍重复发布冻结的同一 stamp，但数值不再推进。恢复服务返回 `data: true` 后，
四者时间戳继续推进并恢复输出。

Ctrl-C 后检查未发现 Gazebo、RViz、bridge、EKF 或项目 ROS 节点残留。

## 8. 错误现象、判断、定位、修改与验证

### 8.1 30 Hz EKF 在当前负载下持续低频

- 现象：配置 30 Hz 时 filtered 墙钟约 21.7 Hz，诊断报告低于最小阈值。
- 判断：完整图形和 RGB-D 链的实时因子低于 1，30 Hz 不是当前主机上稳健的
  基线。
- 定位：对照 EKF 诊断、输出时间戳和 Gazebo stats，而不是只看墙钟
  `ros2 topic hz`。
- 修改：将配置频率调整为 20 Hz。
- 验证：80 样本仿真时间戳频率精确为 20.000 Hz；墙钟约 12.470 Hz，
  `real_time_factor≈0.6745`，状态连续且消息 frame 正确。

### 8.2 高并发检查时一次 update-rate 警告

- 现象：Gazebo GUI、RViz、RGB-D 和多个命令行订阅并行时，EKF 曾报告一次
  update cycle 用时约 0.227 s。
- 判断：这是验收观测本身增加 CPU/消息复制负载造成的瞬时墙钟超时，不是
  输入断链或数值发散。
- 定位：同轮 ROS 图和输入持续存在，后续双时钟采样仍得到 20.000 Hz 仿真
  stamp，连续性检查无 NaN/跳变。
- 修改：没有隐藏警告或篡改 covariance；减少并发检查后分别采集频率和状态。
- 验证：直行、旋转、暂停恢复及连续性检查均通过。

### 8.3 参数批量查询的可选输入提示

- 现象：一次广泛参数查询触发 `imu1` 未初始化提示，`ros2 param dump` 还曾
  返回空映射。
- 判断：`imu1` 是未配置的可选第二 IMU，不是本基线必需输入丢失。
- 定位：`ros2 param list/get` 和节点信息逐项确认实际 `imu0`、`odom0`、frame
  及布尔参数均已加载。
- 修改：动态验收改用显式参数逐项查询，不设置无用的 `imu1`。
- 验证：EKF 实际订阅 `/imu/data` 与 `/wheel/odometry` 并持续输出。

### 8.4 RViz 启动首帧与 Ctrl-C 日志

- RViz 启动时丢弃过一条时间戳 `0.002 s`、早于 TF cache 的 LaserScan；后续
  `/scan` 订阅和 TF 均持续正常，判断为启动竞态。
- Ctrl-C 时既有 `system_heartbeat` 仍有重复 shutdown 异常；部分 bridge 或
  Gazebo 在不同轮次收到 SIGTERM，属于已有整套 Launch 停止路径问题。本任务
  按禁止项未修改 `resilient_nav_monitor`。最终进程和 ROS 节点残留检查为空。

## 9. 仍未完成或需人工确认

- PointCloud2 bridge 和点云处理仍未实现；
- 没有 `/wheel/odometry` 的真实编码器噪声模型、硬件标定或长期累计误差验收；
- 没有融合 IMU 绝对 yaw 或线加速度，需先验证硬件方向、重力处理和 covariance；
- 没有 map frame、GNSS、SLAM、Nav2 或全局定位；
- EKF 是固定字段基线，不是健康感知或自适应融合实现；
- 未生成阶段末交接文档、Word 学习总结或下一阶段测试题；
- RViz 的配置加载和运行时订阅已验证，但最终画面内容仍需用户在桌面环境人工
  观察确认。
