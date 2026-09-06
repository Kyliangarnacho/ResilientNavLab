# 阶段 5 总结：故障注入闭环

## 完成内容

- 建立 `resilient_nav_interfaces/FaultStatus` 和 `resilient_nav_fault_injection`。
- 支持 IMU bias/noise/dropout/fixed delay、wheel freeze 和 LiDAR sector blindness。
- 健康 topic 保持不变，故障数据统一发布到 `/faulted/*`。
- fixed 对照 EKF 发布 `/odometry/faulted` 且 `publish_tf=false`，不争夺主 TF。
- 提供统一 launch、JSON probe、RViz、rosbag 记录和无 Gazebo 回放入口。

## 代表性验证

- IMU bias 活动窗口平均差为 `0.15 rad/s`。
- wheel freeze 期间 faulted 位移为零。
- LiDAR blindness 能稳定产生预期扇区 NaN。

## 边界

`FaultStatus` 是实验真值，只允许注入器和 evaluator 使用，不得进入 Robot Agent 输入。
