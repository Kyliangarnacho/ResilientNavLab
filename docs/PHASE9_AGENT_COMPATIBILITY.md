# Phase 9 Agent compatibility boundary

未来经单独授权的 Agent 可观察经过 sanitizer 的健康运行 metadata：localization mode、map loaded、map id、`map -> odom` 可用性、scan 可用性、localization state 与 map/session metadata。它们只能是只读诊断输入，不能改变 Slam Toolbox、TF、EKF 或 `/cmd_vel`。

RED channel：`/evaluation/ground_truth_pose`、任何 GT error、benchmark answer、Gazebo exact pose 及由它们导出的 evaluator result 均禁止进入 Agent Input、Tool、cache、prompt 或诊断输出。当前不新增 `LocalizationStatus.msg`，不修改 `resilient_nav_agent` 或 agent-core。
