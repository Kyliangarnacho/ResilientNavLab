# Phase 9 健康重定位基线

Jazzy `localization_slam_toolbox_node` 在启动期通过 `map_file_name` 加载 M8 Serialized Pose Graph。M10 的同起点 dock-start 路径保留；M11 增加可选 `map_start_pose`，默认关闭，不改变同起点 baseline。

Run C1（全新进程、同起点）结果：startup/first valid `map -> odom` `0.28 s`，position RMSE `9.15e-16 m`，yaw RMSE `5.59e-05 rad`，地图仍为 `227 x 226`。

Run C2 使用安全的 Gazebo spawn `(2.0, 0.5, pi)`；人为已知测试条件相对 M8 spawn 转换为 map-frame 初值 `(5.5, 4.0, pi)`，不是 Ground Truth topic 输入。结果：startup/first valid TF `0.38 s`，position RMSE `5.59e-16 m`，yaw RMSE `2.81e-05 rad`，地图保持 `227 x 226`。初值只给出匹配起点；后续细化来自 Slam Toolbox scan matching。

不同起点无初值 global localization、kidnapped robot、Nav2 与真实硬件重定位均不属于本 baseline。
