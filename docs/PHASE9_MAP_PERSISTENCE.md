# Phase 9 地图持久化

M8 保存的正式资产位于 `ros2_ws/src/resilient_nav_slam/maps/phase9/`：Occupancy Map 为 `occupancy/phase9_map.pgm` 与 YAML，Serialized Pose Graph 为 `posegraph/phase9_posegraph.posegraph` 与 `.data`。metadata.yaml 保留的是生成当时的 HEAD 和 dirty 状态，属于历史 provenance，不事后改写。

当前 source/install share 校验的 SHA-256 分别为 PGM `548a37d56084ca7a804c96341ef2784f45d22ec4772f5819ea4cd981fa8c3161`、YAML `21499e0fcd079f11a276832ec4622bb1c69a0889dfa9cf7820c08b8c93e45161`、data `586c28fa47cc5def621eeaecd66e564861e10e6faad1b69c5368b1d60e018872`、posegraph `ee7152d9973ed87c26df2dd767fea495173941a9787a70c2cbb70dc6d74b6f20`。

Localization 使用 package share 构造 `phase9_posegraph` 基路径并由 Jazzy node 加载 `.posegraph`，不以 PGM/YAML 代替 pose graph。保存文件未因 M9--M13 验证重新生成或修改。
