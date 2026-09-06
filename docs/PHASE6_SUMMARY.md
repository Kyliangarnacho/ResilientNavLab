# 阶段 6 总结：传感器健康评估

## 完成内容

- 建立 `resilient_nav_health_assessment`，覆盖 timing/stale/delay、wheel freeze、IMU bias 和
  LiDAR sector blindness。
- Health Monitor 发布结构化 `SensorHealth`；`health_evaluator` 仅在评价侧读取 `FaultStatus`。
- 统一 `phase6_health_evaluation.launch.py` 输出检测、分类、恢复和混淆矩阵 JSON。
- LiDAR 统一链曾得到 1 个事件、约 `0.6 s` 检测延迟和约 `0.96` F1。

## 边界

健康检测与真值评价严格分层；本阶段不执行融合策略、参数修改、导航降级或恢复动作。
