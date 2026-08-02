# 阶段 5 收尾总结：可复现实验闭环

记录日期：2026-08-03

## 完成内容与设计原因

阶段 5 已完成一条可复现的故障注入实验闭环：统一 Launch 启动阶段 4 健康链、按场景启动 IMU/wheel/scan 注入器、并行启动 faulted EKF、用 `fault_probe` 输出 JSON 指标、用 rosbag 记录和无 Gazebo 回放。设计上保留原始健康 topic，不覆盖阶段 4 基线；故障数据统一输出到 `/faulted/*`，定位对照输出到 `/odometry/faulted`。

## 完整数据链

```text
Gazebo sensors and DiffDrive
  -> /imu/data, /wheel/odometry, /scan
  -> healthy EKF: /odometry/filtered, publish_tf=true

/imu/data, /wheel/odometry, /scan
  -> phase 5 injectors
  -> /faulted/imu/data, /faulted/wheel/odometry, /faulted/scan
  -> faulted EKF: /odometry/faulted, publish_tf=false

injectors -> /fault_injection/status
fault_probe -> compact JSON metrics
phase5_record_bag -> reproducible bag under scenario/time directory
phase5_replay_bag -> playback without Gazebo
```

健康 EKF 仍是 `odom -> base_footprint` 的实际 TF owner。`faulted_ekf_filter_node` 参数验证为 `publish_tf=false`，输入为 `/faulted/wheel/odometry` 与 `/faulted/imu/data`。运行时 `ros2 topic info /tf -v` 会显示 `faulted_ekf_filter_node` 创建了 `/tf` endpoint，这是 `robot_localization` 节点内部行为；本阶段以参数和消息链路保证 faulted EKF 不驱动主 TF。

## 各包和主要文件职责

- `resilient_nav_interfaces/msg/FaultStatus.msg`：故障真值标签，包含场景、事件、源 topic、faulted topic、模型、时间窗和状态枚举。
- `resilient_nav_fault_injection/launch/phase5_fault_injection.launch.py`：阶段 5 统一入口，参数包括 `scenario_file`、`use_rviz`、`record_bag`、`bag_output`。
- `resilient_nav_fault_injection/config/faulted_ekf.yaml`：faulted EKF 配置，订阅 faulted wheel/IMU，输出 `/odometry/faulted`，`publish_tf=false`。
- `resilient_nav_fault_injection/config/scenarios/*.yaml`：IMU bias、noise、dropout、delay、wheel freeze、Lidar sector blindness 和 EKF 对照场景。
- `resilient_nav_fault_injection/fault_probe.py`：有限时长 JSON 指标采集工具。
- `resilient_nav_fault_injection/phase5_record_bag.py`、`phase5_replay_bag.py`：记录和无 Gazebo 回放辅助入口。
- `resilient_nav_fault_injection/rviz/phase5_fault_injection.rviz`：RobotModel、TF、健康/faulted scan、健康/faulted odometry 对照显示。

## 故障模型及参数

- IMU `bias`：窗口 5.0 到 15.0 s，`angular_velocity.z += 0.15 rad/s`。
- IMU `gaussian_noise`：窗口 5.0 到 15.0 s，`noise_sigma_rad_s=0.05`，固定 seed `20260803`。
- IMU `dropout`：窗口 5.0 到 15.0 s，丢包概率 `0.30`。
- IMU `fixed_delay`：窗口 5.0 到 15.0 s，延迟 `0.50 s`，保留原始 stamp。
- wheel `freeze`：窗口 5.0 到 15.0 s，冻结窗口首个 odometry pose/twist 快照。
- Lidar `sector_blindness`：窗口 5.0 到 15.0 s，中心 `0.0 rad`、宽度 `1.0 rad`，范围束置为 NaN。

## 错误定位与修复

- 错误现象：首次动态启动在沙箱内失败，日志含 `getifaddrs: Operation not permitted` 和 `TRANSPORT_UDP Error`；初步判断：ROS 2 DDS/Gazebo 需要访问网络接口，受限沙箱禁止；定位过程：同一 launch 在申请沙箱外权限后成功创建实体、bridge 和 EKF；修改内容：无代码修改，动态实验使用 `/tmp/resilient_nav_phase5_ros_logs` 和沙箱外执行；修复后验证：统一 launch 可稳定启动并输出全部关键 topic。

- 错误现象：`fault_probe` 首版把非活动窗口样本计入 IMU bias 平均值，且延迟统计因未默认仿真时间出现异常大值；初步判断：指标聚合没有按 `FaultStatus` 时间窗过滤，probe 时钟未使用 `/clock`；定位过程：IMU bias 首轮 JSON 中平均差约 `0.052`，低于配置 `0.15`；修改内容：probe 默认注入 `use_sim_time=true`，按非 CANCELLED 的 `FaultStatus` 窗口计算 active 指标；修复后验证：IMU bias active 样本 1000 对，平均差 `0.15 rad/s`。

- 错误现象：Ctrl-C 停止时三个阶段 5 注入器因重复 `rclpy.shutdown()` 退出码为 1；初步判断：rclpy 信号处理已关闭 context，finally 中再次 shutdown；定位过程：launch 停止日志显示 `rcl_shutdown already called` 来自注入器；修改内容：注入器捕获 `KeyboardInterrupt`，并仅在 `rclpy.ok()` 时 shutdown；修复后验证：后续停止中 IMU/wheel/scan 注入器均 cleanly 退出。

- 错误现象：`phase5_replay_bag` 用 `subprocess.call` 包装回放时 Ctrl-C 会打印 Python traceback；初步判断：wrapper 没有直接交还进程控制权；修改内容：改用 `os.execvp()` 执行 `ros2 bag play`；修复后验证：回放入口由 rosbag 原生命令接管。

保留问题：`system_heartbeat` 在 Ctrl-C 时仍有阶段 3/4 已知的重复 shutdown 异常；本次授权范围未修改 `resilient_nav_monitor`。一次 wheel+bag 停止中 `clock_bridge` 也出现 shutdown-time `std::system_error`，未留下进程。

## 构建、测试、动态实验、TF、指标和 rosbag 证据

- 构建：`colcon build --symlink-install`，6 个包全部成功。
- 测试：`colcon test && colcon test-result --verbose`，193 tests，0 errors，0 failures，1 skipped。
- 话题并存：运行时可见 `/imu/data`、`/faulted/imu/data`、`/wheel/odometry`、`/faulted/wheel/odometry`、`/scan`、`/faulted/scan`、`/odometry/filtered`、`/odometry/faulted`。
- faulted EKF 参数：`publish_tf=false`，`odom0=/faulted/wheel/odometry`，`imu0=/faulted/imu/data`。
- IMU bias：active 1000 对样本，`angular_velocity_z_diff_mean=0.15`，stddev 约 `1.24e-18`，EKF yaw mean diff 约 `0.818 rad`。
- wheel freeze：active raw 位移约 `0.50886 m`，faulted 位移 `0.0 m`，`freeze_detected=true`，健康/faulted EKF 均 559 条输出。
- Lidar blindness：333 帧 faulted scan，总 NaN 束数 20672，最大单帧 136，NaN 比例约 `0.096997`。
- rosbag：`/tmp/phase5_bags/wheel_freeze_ekf_comparison_20260803_013910`，13.3 MiB，47962 条消息，12 个要求话题全部记录。
- 回放：不启动 Gazebo，仅 `phase5_replay_bag ... --clock`，回放期间可见 `/clock`、raw/faulted sensor、`/fault_injection/status`、健康/faulted EKF、`/tf`、`/tf_static`。
- 残留：最终 `ps` 检查未发现 Gazebo、RViz、bridge、EKF、注入器或 bag 进程残留。

## 未完成内容

- 没有实现健康评估、异常检测、自适应融合、Nav2、SLAM、真实硬件实验或容错导航策略。
- RGB-D 图像故障和 PointCloud2 不在阶段 5 首批闭环内。
- `system_heartbeat` shutdown 异常仍待后续监控包维护任务处理。
- RViz 配置已提供并随 launch 可选启动；本次动态验收主要使用 headless 运行，未保存截图证据。
