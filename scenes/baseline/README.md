# 基础测试场景 baseline

标签：**基础测试场景（2026-11-08 已验收）**。

这是原来的 8×6 m 三房间场景，共四把 Fuel 椅子，厨房两把、客厅和卧室各一把。用于基础链路与算法回归，区别于新增的 AWS 家庭展示场景。

- 世界：`course_house`，构建入口保留在 `fortress.launch.py` 的基础场景函数。
- 地图：`maps/baseline/course_house.yaml` 和 `course_house.pgm`。
- 原地图路径 `maps/course_house.*` 也保留。
- 房间/观察点：`src/semantic_search/config/rooms_baseline.json`。
- 历史验收：`results/validation/summary.json`、`docs/archive/validation/VALIDATION.md`。
- 原始地图与配置的 SHA-256：本目录 `preservation.json`。

后续家庭场景的建图、结果和调整使用独立路径，不能覆盖这些基础场景验收材料。
