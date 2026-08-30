# Phase 10 Costmap Rotation Ghost：最终根因与最小修复

记录日期：2026-08-28

## 结论

rotation ghost 的直接来源不是 Costmap lifecycle、update frequency、inflation、AMCL、EKF
或 frozen map，而是 270° GPU LiDAR 在旧 FOV 最小边界产生的一个固定异常 ray。该短距
`/scan` hit 会被 ObstacleLayer 正常 marking，随后经 InflationLayer 扩张；因此表面上成为
Global/Local 中位置相近但不完全重合的 persistent ghost。

最小修复只排除这个 source ray：`resilient_nav_robot.urdf.xacro` 将 horizontal samples
从 `640` 改为 `639`，并将 `min_angle` 向内移动一个旧 angular increment `pi/426`。`max_angle`
不变，剩余 638 个间隔仍为 `pi/426`。未改 frozen map、Costmap algorithm、EKF、AMCL 或
Nav2 上游代码，也未加入 filter 或 temporal gate。

## 证据链

相同 fresh-process、`0.6 rad/s`、`10.8 s` 原地整圈转动中，诊断记录仅保留
`<=2.5 m` 的 marking-range 孤立短距候选：

| 方向 | 修复前 scan 数 | 候选数 | 共同特征 |
| --- | ---: | ---: | --- |
| CW | 663 | 28 | 全部 `beam_index=0` |
| CCW | 673 | 27 | 全部 `beam_index=0` |

旧消息为 640 beams，异常 beam 是 `angle_min=-2.356194496 rad`（`-135°`），range 约
`2.19–2.48 m`，后续五条 beam 为约 `4.07–9.26 m`。两个方向的 endpoint 都沿 map 中约
`y=2.3 m` 的场地边移动，符合固定 FOV 边界 ray 的观测。

修复后，CW 669 个 scan 和 CCW 645 个 scan 均为 639 beams，均无 marking-range 孤立短距
候选；用户进一步确认 Global/Local Costmap 不再留下 persistent ghost。

## 正式架构收口

早期 startup gate、`autostart=false`、15/5 Hz hardening 和 rotation diagnostic 只是排查
instrumentation，均不保留在正式运行链。Task 2 恢复标准 Lifecycle Manager `autostart=true`：

- Global Costmap：`update/publish=1/1 Hz`；
- Local Costmap：`update/publish=5/2 Hz`；
- 保留 `sensor_frame=lidar_link` 和 `expected_update_rate=0.2`；
- `observation_persistence=0.0` 为 Jazzy 默认行为，不显式配置；
- 保留 Global probe 按 `scan.header.stamp` 查询 TF 的时间一致性修正。

## 剩余边界

- 此证据足以隔离并消除项目配置中的异常 source ray，但尚未唯一归因于 GPU renderer、
  Gazebo collision 或 GZ-to-ROS bridge 的哪一层。
- standalone `nav2_costmap_2d` 的 Ctrl-C teardown 偶发 `exit -11` 与运行期数据链无关，
  仍应在将来独立最小复现。
- 本结论覆盖已验收的 world 和 270° LiDAR 配置，不自动推广至其他传感器/FOV/world。
