# 阶段 10 总结：健康 Nav2 导航

## 收口结论

状态：**已收口，工程验收通过并保留已知限制**。

```text
Saved Map -> Map Server + AMCL
  -> Global/Local Costmap
  -> Navfn -> RPP
  -> BT NavigateToPose -> /cmd_vel
```

## 完成内容

- saved-map Map Server、AMCL 与唯一 `map -> odom -> base_footprint` TF 链。
- Planner-owned Global Costmap、Controller-owned Local Costmap、polygon footprint 与
  Static/Obstacle/Inflation layers。
- Navfn、Regulated Pure Pursuit、官方无 Recovery BT 和 1 Hz replanning。
- healthy benchmark：simple 3/3、detour 3/3、multi-turn 2/2 valid，合计 8/8 valid PASS；另一次
  multi-turn 在发 goal 前因 TF readiness 判为 infrastructure-invalid，因此严格自动 9/9 未达到。
- 动态绕障 engineering PASS。
- fully blocked 无 Recovery 场景以 Planner-first `NO_VALID_PATH/208 -> BT ABORT -> stop` 安全失败。
- 官方 Recovery 场景在临时墙删除后恢复并 SUCCESS。
- 原生 Goal Cancel 得到 `CANCELED`，Controller、odom 与 GT 均停止。

## 已知限制

- AMCL 相对 GT 可有约 `0.3 m` endpoint deviation。
- scan 与物理墙存在小角度偏差；长墙清除受有限量程和遮挡影响。
- Task 4 的一个无效 trial 不计入导航成功率，也不伪装为算法失败。
- Ground Truth 始终只在 evaluator 通道；Runner 只发送 `NavigateToPose`。

原始批量运行和动态障碍证据已迁至 [`data/phase10`](../data/phase10/)，不再与总结文档混放。
