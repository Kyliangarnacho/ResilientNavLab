# 阶段 9 总结：健康二维 LiDAR SLAM

## 完成内容

- 复用 Jazzy Slam Toolbox 2.8.5，healthy EKF 独占 `odom -> base_footprint`，Slam Toolbox 独占
  `map -> odom` 和 `/map`。
- 完成二维 mapping、M8 occupancy map 与 serialized pose graph 持久化。
- 在全新 localization process 中重载保存地图，并验证不同初始 pose。
- Ground Truth 仅经 evaluation overlay 进入只读 evaluator。
- automatic loop closure 保留直接证据链：`TryCloseLoop accepted -> LinkChainToScan -> CorrectPoses`。

## 评价结果

persisted-map localization 使用实验前声明的固定 `T_map_odom=(5.5,4.0,pi)`，不从本次轨迹拟合。
代表性 fixed-frame position/yaw RMSE：SLAM `0.3434 m / 0.1788 rad`，healthy EKF
`0.4594 m / 0.1905 rad`。SLAM 相对改善，但 endpoint error 仍约 `3.022 m / 1.612 rad`，因此不描述为
高精度定位。

## 边界

本阶段不包含 fault-aware SLAM、Adaptive EKF+SLAM 正式比较或 Nav2 控制。

