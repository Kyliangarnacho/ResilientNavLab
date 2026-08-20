# Phase 9 开源项目基线（2D SLAM 技术雷达）

记录日期：2026-08-20

## 1. 目的与当前授权边界

本文记录 Phase 9 Milestone 0 的技术雷达与依赖决策，遵循“避免手撕 sin17°”原则：优先以成熟项目作为问题边界、接口和验收基线，不从零手写已有 SLAM 子系统，也不复制其实现。它不是安装清单、实现授权或性能结论。

本 Milestone 不安装软件、不 clone 仓库、不新增 ROS 包、不实现 SLAM、不改生产代码、Robot Agent 或控制链。Phase 8 已完成的健康感知融合与 evaluator-only Ground Truth 边界保持不变；任何后续 SLAM 运行时仍不得读取 `FaultStatus`、场景真值或 `/evaluation/*`，也不得改变当前唯一 TF owner、发布控制命令或绕过 Safety Gate。

`Borrow` 表示在后续获授权的运行时直接采用外部项目的已发布能力作为 baseline，而不复制或派生其源码；`Reference-only` 表示只独立借鉴问题分解，不作为运行时候选；`Adapt` 只指 ResilientNavLab 自己的 Launch、TF ownership、实验与评价合同。`Reject` 表示本阶段明确不采用的第三方代码、功能或依赖。

## 2. 决策总表

| 项目 | 决策 | Phase 9 用途 | 依赖决策 | 许可证与仓库 |
| --- | --- | --- | --- | --- |
| [Slam Toolbox](https://github.com/SteveMacenski/slam_toolbox) | **Borrow** | 实际 2D SLAM runtime baseline：后续获授权的二维 LaserScan 建图/定位实验直接使用其 ROS 2 能力和明确运行模式，不手写等价 2D SLAM。 | Milestone 0 不安装、不 clone、不新增依赖。后续只可在明确的兼容性、TF ownership、数据接口与回放验收任务中评估 ROS 包依赖；不复制源码。 | LGPL-2.1；[LICENSE](https://github.com/SteveMacenski/slam_toolbox/blob/master/LICENSE)；[GitHub](https://github.com/SteveMacenski/slam_toolbox) |
| [Cartographer](https://github.com/cartographer-project/cartographer) | **Reference-only** | 只参考 local SLAM / global SLAM 的职责划分，以及 pose-graph 全局一致性与局部实时链解耦。 | Reject：不引入 Cartographer、Cartographer ROS、Lua 配置体系或其构建/优化依赖。 | Apache-2.0；[LICENSE](https://github.com/cartographer-project/cartographer/blob/master/LICENSE)；[GitHub](https://github.com/cartographer-project/cartographer) |
| [RTAB-Map](https://github.com/introlab/rtabmap) | **Reference-only** | 只参考地图持久化、重载与重定位的产品/实验问题分解。 | Reject：不引入 RTAB-Map、`rtabmap_ros`、RGB-D/视觉/3D 建图链、数据库格式或其依赖。 | BSD-3-Clause；[LICENSE](https://github.com/introlab/rtabmap/blob/master/LICENSE)；[GitHub](https://github.com/introlab/rtabmap) |
| [KISS-ICP](https://github.com/PRBonn/kiss-icp) | **Reference-only** | 只参考 local odometry 与低频 global correction 分层，避免把局部连续估计和全局一致性校正混成一个实时职责。 | Reject：不引入 KISS-ICP、其 ROS wrapper、ICP 实现、PointCloud2 或 3D LiDAR 依赖。 | MIT；[LICENSE](https://github.com/PRBonn/kiss-icp/blob/main/LICENSE)；[GitHub](https://github.com/PRBonn/kiss-icp) |

许可证信息只用于工程风险识别，不构成法律意见；若未来复制、再分发或引入任何项目/其传递依赖，必须在实现任务中重新核验版本、完整许可证与归因义务。

## 3. 项目级边界

### Slam Toolbox — 实际 2D SLAM runtime baseline

- **Borrow：** 以后续获授权的二维 `LaserScan` 建图与定位实验为实际运行时 baseline；比较应限定在已记录的传感器、时间、坐标系、地图和评价条件内。
- **不采用：** 本 Milestone 不安装或运行 package，不复制源码、参数文件、launch、地图文件或测试数据；不把其能力等同于 Nav2、容错导航、控制、真实硬件定位或已验证的 Phase 9 性能。
- **接口纪律：** 未来集成前须单独确定 map/odom/base TF 的唯一 owner、与 Phase 8 adaptive 输出的关系、健康数据的合法输入边界，以及 evaluator 与运行链的真值隔离。

### Cartographer — reference-only 的 local/global 与 pose-graph 架构参考

- **Reference-only：** 把局部实时匹配和较慢的全局 pose-graph 一致性处理为可分别验证、不同频率的职责。
- **不采用：** 不接入任何 Cartographer 代码、ROS wrapper、2D/3D 算法管线、传感器模型、配置值或优化器；不以其 global SLAM 结构暗示当前已有 map-frame 全局定位。
- **依赖决定：** 当前保持零新增依赖。其上游仓库标注为不再积极维护，因此它只作为架构参考，不是 Phase 9 的运行时候选。

### RTAB-Map — reference-only 的持久地图与重定位问题参考

- **Reference-only：** 将地图的保存、版本/兼容性、重载和重定位成功/失败条件作为未来可验收的问题，而不是把“已有地图文件”写成已完成重定位。
- **不采用：** 不引入其代码、`rtabmap_ros`、数据库或地图格式、视觉词袋、回环检测、RGB-D/双目/3D LiDAR 流程、GUI、配置或数据集。
- **依赖决定：** 当前保持零新增依赖；任何未来实际采用都需要单独的依赖、数据格式和许可证复核，不能由本技术雷达自动推出。

### KISS-ICP — reference-only 的 local odometry / global correction 分层参考

- **Reference-only：** 保持本地连续里程计与全局校正两个职责的显式边界；后者不能直接掩盖前者的质量、健康或时序问题。
- **不采用：** 不将当前二维 `LaserScan` 伪装为点云，不引入 ICP、点云预处理、KISS-ICP ROS 节点、3D LiDAR、PointCloud2、其参数或数据集。
- **依赖决定：** 当前保持零新增依赖；仅在未来 3D LiDAR/PointCloud2 获单独授权且接口、标定和回放证据充分时，才可重新评估。

## 4. Phase 9 后续门槛（非实现授权）

任何 Phase 9 实现提案至少应先说明：

1. 为什么 Slam Toolbox 足以作为实际 2D baseline，以及精确的 ROS 2/Jazzy 版本与依赖兼容性；本文件不替代该核验。
2. `map`、`odom` 与 `base_footprint` 的唯一 TF owner 和切换/停止策略；不得与当前健康参考或 Phase 8 adaptive 输出重复发布。
3. 运行时只使用合法观测与结构化健康信息；`FaultStatus`、场景/参数真值和 `/evaluation/*` 继续只属于 benchmark/evaluator。
4. 至少覆盖健康、退化、恢复、无信息和错误重定位/回环候选的可回放评价，并把局部 odometry、全局校正、地图持久化与重定位分别报告。
5. 任何 Nav2、规划、恢复、状态变更或控制能力都须另行授权；本技术雷达不构成该授权。
