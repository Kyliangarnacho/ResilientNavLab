# BRNE V1 总结：动态行人交互闭环

## 收口状态

Scene 1 单行人横穿、Scene 2 顺序双横穿和 Scene 3 迎面行人使用同一冻结配置：
`ros2_ws/src/resilient_nav_brne/config/brne_v1_runtime.yaml`。正式 pedestrian state 来自机器人 LiDAR；
Gazebo odometry 输入只保留为隔离感知问题的验证入口。

## 数据链

```text
/scan + scan timestamp TF
  -> cluster / common-drift correction / tracking / velocity EMA
  -> /brne/pedestrians (odom, id, x, y, vx, vy)
  -> interaction lifecycle + crossing/head-on event
  -> pinned BRNE core + project-side weights
  -> weighted control sequence
  -> armed control gate -> /cmd_vel
```

Navfn 只消费 `/brne/static_scan`。新 cluster 在确认静态前被隔离；已确认 dynamic agent 不再以静态障碍
身份重复影响 global path。RViz 显示 Global Costmap、dynamic-agent marker、global path 和 BRNE prediction。

## 冻结参数

| 类别 | 参数 |
| --- | --- |
| Pinned BRNE | agents 5，samples 196，`dt=0.1`，steps 25，kernel `0.2/0.2`，cost `15/3/20`，ped scale `0.1` |
| 机器人 | 5 Hz，linear/angular max `0.30/0.80`，nominal linear `0.20` |
| Interaction | entry `3.20 m`；最近距离增加 `0.20 m` 且连续 4 个输出分离后释放 |
| Crossing | lateral `0.08 m/s`，alignment `0.50`，`t_cross<=4 s`，forward `[0.20,2.00] m`，side bias `10×`，初始方向窗口 5 |
| Head-on | approach `0.12 m/s`，方向锥 `1.05 rad`，deadband `0.17 rad`，CV line clearance `0.58 m` |
| Proposal | opposite threshold `0.35 rad/s`，scale `0.25`，窗口 5 |
| Safety | time-aligned CV threshold `0.20 m`；crossing preferred separating factor `0.10` |
| LiDAR | agent range `4.0 m`，6 帧拟合，EMA `0.25`，3 帧方向稳定，方向变化/拒绝 `0.35/0.70 rad` |

一个 owner 同时最多进入一个 event。横穿选择行人横向速度反侧；迎面事件在可靠斜向分量存在时选择
反侧，否则由首个明显 BRNE 输出确定 passing side。proposal protection 只恢复 sampler support，不直接
强制最终 command。最终权重在每个时间步沿 sample axis 归一化；即时命令取 weighted sequence 第 0 步。

没有 dynamic agent 时，wrapper 使用 Navfn local waypoint 做确定性低速跟随；输入 stale、全部 candidate
无效或 control gate 未 armed 时 fail closed。

## 入口

```bash
ros2 launch resilient_nav_brne brne_sensor_scene1_demo.launch.py arm_brne:=true use_rviz:=true
ros2 launch resilient_nav_brne brne_scene2_demo.launch.py arm_brne:=true use_rviz:=true
ros2 launch resilient_nav_brne brne_scene3_head_on_demo.launch.py arm_brne:=true use_rviz:=true
```

## 验证与边界

- 包测试：`123 passed, 1 xfailed`；xfail 是 pinned scalar `traj_sim()` 缺 `dt` 的已知合同。
- `196×25` warm planning mean/P95/max：`11.315/12.706/15.292 ms`，低于 200 ms 周期。
- `close_stop_threshold=0.20 m` 是 point-agent 实验阈值，不是完整 footprint 几何安全证明。
- LiDAR V1 不做人类分类、遮挡续接或统计 benchmark。
- 旧 passing-side commitment、global-path freeze、nominal freeze 和 output low-pass 已从生产逻辑删除；
  相关实验经验只保留在学习日志。

