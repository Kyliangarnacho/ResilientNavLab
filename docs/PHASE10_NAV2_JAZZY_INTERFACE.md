# Phase 10 Task 1.1–1.2：Nav2 Jazzy 接口与定位骨架

记录日期：2026-08-27

## 1. 范围和完成状态

本页保留 Task 1.1–1.2 的安装/接口冻结快照：Task 1.1 已完成官方 binary 安装核验；
Task 1.2 已完成 `resilient_nav_navigation` 的最小 `ament_python` 工程骨架、静态资源
安装和接口核验。后续 Task 1 动态定位结果见
`docs/PHASE10_TASK1_LOCALIZATION_SMOKE.md`，Task 2 Global Costmap 结果见
`docs/PHASE10_TASK2_COSTMAP_SMOKE.md`；二者均不提供自主导航或运动命令。

## 2. 实际 Jazzy 安装基线

用户于 2026-08-27 使用官方 APT binary 安装了 `ros-jazzy-navigation2` 与
`ros-jazzy-nav2-bringup`。本机核验到的已安装包及版本如下：

| 包 | 版本 | 前缀 |
| --- | --- | --- |
| `navigation2` | `1.3.12-1noble.20260615.181551` | `/opt/ros/jazzy` |
| `nav2_bringup` | `1.3.12-1noble.20260616.082701` | `/opt/ros/jazzy` |
| `nav2_map_server` | `1.3.12-1noble.20260615.153120` | `/opt/ros/jazzy` |
| `nav2_amcl` | `1.3.12-1noble.20260615.152740` | `/opt/ros/jazzy` |
| `nav2_lifecycle_manager` | `1.3.12-1noble.20260615.152740` | `/opt/ros/jazzy` |

已发现的运行入口为：

- `nav2_map_server`：`map_server`、`map_saver_server`、`map_saver_cli`、`costmap_filter_info_server`；
- `nav2_amcl`：`amcl`；
- `nav2_lifecycle_manager`：`lifecycle_manager`。

本机 installed `package.xml` 的声明许可分别为：`nav2_bringup` Apache-2.0，
`nav2_map_server` Apache-2.0 与 BSD-3-Clause，`nav2_amcl` LGPL-2.1-or-later，
`nav2_lifecycle_manager` Apache-2.0。它们是 binary/package 元数据核验结果，
不替代对未来复制文件或新增依赖的逐项许可审查。

## 3. 冻结的上游接口

实际 `/opt/ros/jazzy/share/nav2_bringup/launch/localization_launch.py` 的参数为：
`namespace`、`map`、`use_sim_time`、`params_file`、`autostart`、
`use_composition`、`container_name`、`use_respawn`、`log_level`。

非 composition 分支由上游 launch 创建 Map Server、AMCL 和
`lifecycle_manager_localization`；其 lifecycle node 顺序为 `map_server`、`amcl`。
composition 分支则使用上游 plugin 名称 `nav2_map_server::MapServer`、
`nav2_amcl::AmclNode` 和 `nav2_lifecycle_manager::LifecycleManager`。本项目不复制
该实现。

Phase 10 最小参数合同仅覆盖：

- Map Server：`frame_id=map`、`topic_name=/map`；`map` launch 参数由上游注入
  `yaml_filename`；
- AMCL：`map` / `odom` / `base_footprint` frames，`/map`、`/scan`，
  `nav2_amcl::DifferentialMotionModel`、`likelihood_field`、`tf_broadcast=true`；
- 初始位姿：`set_initial_pose=false`、`always_reset_initial_pose=true`，因此在没有
  launch 参数预置初始位姿时，操作者必须显式发布 `/initialpose`；
- 不配置 costmap、planner、controller、behavior tree、recovery、导航目标或
  `/cmd_vel`。

`ros2 launch nav2_bringup localization_launch.py --show-args` 已成功显示上述接口。
受限执行环境的默认 `~/.ros/log` 不可写，因此核验使用
`ROS_LOG_DIR=/tmp/resilient_nav_phase10_ros_logs`；这不是 Nav2 或项目运行错误。

## 4. Phase 9 资产身份冻结

Task 1 后续定位只消费 `resilient_nav_slam` 已安装资源，不复制、不重建、也不修改
Phase 9 地图。源资产的 SHA-256 为：

| 资源 | SHA-256 |
| --- | --- |
| `maps/phase9/occupancy/phase9_map.pgm` | `548a37d56084ca7a804c96341ef2784f45d22ec4772f5819ea4cd981fa8c3161` |
| `maps/phase9/occupancy/phase9_map.yaml` | `21499e0fcd079f11a276832ec4622bb1c69a0889dfa9cf7820c08b8c93e45161` |
| `maps/phase9/posegraph/phase9_posegraph.data` | `586c28fa47cc5def621eeaecd66e564861e10e6faad1b69c5368b1d60e018872` |
| `maps/phase9/posegraph/phase9_posegraph.posegraph` | `ee7152d9973ed87c26df2dd767fea495173941a9787a70c2cbb70dc6d74b6f20` |

Occupancy YAML 的身份元数据为 `227 × 226`、resolution
`0.05000000074505806`、origin `[-2.22269738693, -2.16105234952, 0]`。
Phase 9 仍是 `/map` 与 `map -> odom` 的唯一 owner，healthy EKF 仍是
`odom -> base_footprint` 的唯一 owner；当前 skeleton 只读入 saved map，并不启动
Slam Toolbox。

Phase 9 最终总结是回环结论的权威记录：无 manual service 的最终 run 保留
`TryCloseLoop accepted → LinkChainToScan → CorrectPoses`，故 automatic loop closure
为 PASS。仓库中的早期 raw evidence JSON 和 2026-08-21 学习记录仍有
UNCONFIRMED 描述，不能覆盖该最终总结；该历史不一致应在后续 Phase 9 档案整理时
单独消解。

## 5. 新包及复用关系

`ros2_ws/src/resilient_nav_navigation` 是第 12 个项目 ROS package，采用
`ament_python`，并安装：

- `launch/phase10_localization.launch.py`：只用 `IncludeLaunchDescription` 复用
  Nav2 官方 `nav2_bringup/launch/localization_launch.py`，透传全部九个上游参数；
- `config/nav2_localization.yaml`：上述最小 Map Server / AMCL 合同；
- `rviz/phase10_localization.rviz`：仅 map、scan、robot、TF、AMCL pose、particles
  和 Set Initial Pose；
- package metadata、resource marker 与静态资源测试。

wrapper 默认 map 是已安装 `resilient_nav_slam` 的
`maps/phase9/occupancy/phase9_map.yaml`；默认 `use_sim_time=true`、
`autostart=true`、`use_composition=false`、`use_respawn=false`、`use_rviz=false`。
它不复制 Map Server、AMCL 或 Lifecycle Manager，不启动 Slam Toolbox/Gazebo，不包含
控制或安全绕行接口。

## 6. 已执行验证与未完成门槛

- `python3 -m pytest src/resilient_nav_navigation/test -q`：6 passed；
- `colcon build --symlink-install --packages-up-to resilient_nav_navigation`：10 个
  相关包构建成功；
- `colcon test --packages-select resilient_nav_navigation`：6 tests、0 errors、0
  failures、0 skipped；
- sourced install 下 `ros2 pkg prefix resilient_nav_navigation` 指向 workspace install，
  `ros2 launch resilient_nav_navigation phase10_localization.launch.py --show-args`
  成功，且默认 map/params 指向安装后的静态资源。

后续 Task 1.3–1.5 已在单独受控 run 中完成 Map Server/AMCL lifecycle、TF、`/scan`
QoS、显式 `/initialpose` 和定位验收；详见对应动态记录。该历史接口冻结不改变边界：
尚无 local costmap、planner、controller 或 `/cmd_vel` 自主控制能力。
