# 阶段 8 总结：健康感知融合

## 架构

```text
/faulted/scan -> reciprocal + median/MAD Robust ICP -> LiDAR odometry -> local admission
/faulted/wheel + /faulted/imu + LiDAR velocity + wheel/IMU SensorHealth
  -> schema-v2 RF reliability + unified Measurement Adapter/Fusion Supervisor
  -> /fusion/input/*
  -> adaptive EKF (/odometry/adaptive, publish_tf=false)

Gazebo Ground Truth + fixed/adaptive odometry
  -> evaluator-only Localization Evaluator
```

`FusionPolicy` 是纯 Python 确定性策略，根据 wheel/IMU 健康状态决定测量接纳、协方差倍率、wheel yaw
fallback、wheel fault 时的 LiDAR 平移 fallback 和恢复滞回。measurement adapter 不读取故障真值，
Ground Truth 只存在于 `/evaluation/*`。

启动阶段不再把尚未积满 Health 窗口等同于故障恢复：adapter 先检查消息时间戳、frame、有限值、
四元数和实际使用的协方差；合法首帧作为 `PROVISIONAL` 以原协方差接入，首次确认 `HEALTHY` 也不走
recovery confirmation。后续恢复确认由 Health Monitor 单层负责，Fusion 不再重复增加等待。delay/bias 的首个
异常证据立即进入 `DEGRADED`；wheel freeze 使用约 `0.4 s` 无进展短窗预降级，约 `1.0 s` 完整窗确认
`FAULT`，并分别用位置和 yaw 进展避免把正常平移/转向当作 freeze。

## 代表性结果

| 场景 | 实验完整性 | 主要结论 |
| --- | --- | --- |
| Healthy startup parity | PASS | adaptive/fixed 的位置与 yaw RMSE 差异约 `+1.50%/+1.26%`，处于同一量级 |
| IMU bias | PASS | adaptive 的位置与 yaw RMSE 均改善 |
| IMU fixed delay | PASS | 结果存在 run-to-run 敏感性，不宣称稳定改善 |
| IMU dropout | SKIPPED | 既有概率配置不能可靠越过 stale 门限 |
| Wheel freeze + LiDAR fallback V3 | PASS | 异常在故障后 `0.402 s` 首报，`1.0 s` 时拒绝 wheel 并接纳 LiDAR velocity + IMU yaw-rate；adaptive 位置/yaw RMSE 均优于 fixed |

## 边界

adaptive EKF 不拥有 TF，也不在运行中修改其他 EKF 参数。LiDAR scan matching 独立于 wheel odometry，
在 FusionPolicy 之前用 reciprocal correspondence、median/MAD residual gate 和 trimmed inliers 排除突然
出现或移动的少数点，再产生带质量门限和协方差的平移速度；是否接纳仍由统一 FusionPolicy/measurement
adapter 决定。adapter 不订阅 `/health/scan`；它只对 Robust ICP 已发布的速度做时间戳、frame、有限值和
协方差本地准入。该动态点逻辑不进入 Health Monitor，也不把环境变化误报为传感器故障。
benchmark truth 只供 evaluator 使用，不进入 Health、Fusion 或 Agent。

启动/预降级补丁后的 Phase 9 富几何世界 wheel-freeze V3 回归为 PASS。故障从 `5.0 s` 开始，Health
和 Fusion 在 `5.402 s` 首次非健康，检测延迟为 `0.402 s`；Fusion 在 `6.0 s` 拒绝 wheel velocity，
`6.006 s` 观察到实际 LiDAR fallback，`15.4 s` 恢复 NOMINAL。相较旧 V2 记录，异常首报由
`7.0 s` 提前至 `5.402 s`，完整 wheel-fault response 由 `7.0 s` 提前至 `6.0 s`。

V3 主对齐窗口的 adaptive/fixed 位置 RMSE 为 `0.430/1.228 m`，yaw RMSE 为 `0.242/1.340 rad`，
最大位置误差为 `0.893/2.323 m`；运行中观察到 139 条 LiDAR fallback measurement。原始数据保存在
`data/phase8/wheel_freeze_startup_guard_v3_phase9_world_20260907/`；旧 V2 数据继续保留在
`data/phase8/wheel_freeze_lidar_fallback_v2_phase9_world_20260907/`，但不再代表最新补丁。

同世界 healthy startup parity 回归也为 PASS。adaptive/fixed 位置 RMSE 为
`0.003193/0.003146 m`，yaw RMSE 为 `0.004394/0.004339 rad`，Adaptive 分别高约 `1.50%/1.26%`；
三个对齐窗口内位置 RMSE 最大相对差约 `4.88%`，未出现启动拒绝造成的明显性能缺口。
`adaptive_improved=false` 只表示 Adaptive 没有同时严格优于 Fixed，不表示 parity 失败。Health evaluator
在 271 个已评价状态中记录了 2 个 false positive（wheel/IMU 各 1），应保留为后续观察项。
原始数据保存在 `data/phase8/healthy_startup_parity_v1_phase9_world_20260907/`。

两组结果均为单次场景回归，不外推为跨地图、动态障碍或复合故障鲁棒性结论。
