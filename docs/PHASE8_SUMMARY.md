# 阶段 8 总结：健康感知融合

## 架构

```text
/faulted/* + SensorHealth
  -> Measurement Adapter
  -> /fusion/input/*
  -> adaptive EKF (/odometry/adaptive, publish_tf=false)

Gazebo Ground Truth + fixed/adaptive odometry
  -> evaluator-only Localization Evaluator
```

`FusionPolicy` 是纯 Python 确定性策略，根据 wheel/IMU 健康状态决定测量接纳、协方差倍率、wheel yaw
fallback 和恢复滞回。measurement adapter 不读取故障真值，Ground Truth 只存在于 `/evaluation/*`。

## 代表性结果

| 场景 | 实验完整性 | 主要结论 |
| --- | --- | --- |
| Healthy | PASS | adaptive 不保证优于 fixed，保持为对照 |
| IMU bias | PASS | adaptive 的位置与 yaw RMSE 均改善 |
| IMU fixed delay | PASS | 结果存在 run-to-run 敏感性，不宣称稳定改善 |
| IMU dropout | SKIPPED | 既有概率配置不能可靠越过 stale 门限 |
| Wheel freeze | PASS | 能拒绝 wheel velocity，但没有独立平移冗余 |

## 边界

adaptive EKF 不拥有 TF，不在运行中修改其他 EKF 参数，也不构造缺失的平移信息。benchmark truth
只供 evaluator 使用，不进入 Health、Fusion 或 Agent。

