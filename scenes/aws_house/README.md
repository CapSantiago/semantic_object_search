# AWS 家庭展示场景 aws_house

默认演示场景，基于固定版本 AWS Small House，保留 87 个家具和建筑实例。原测试场景独立保留为 `baseline`。

- 操作与验收：[docs/AWS_HOUSE.md](../../docs/AWS_HOUSE.md)。
- 来源、提交、许可和适配范围：`source.json`；上游原件 `third_party/aws_small_house/`。
- 实例世界坐标：`ground_truth.json`，仅用于评估，搜索节点不读取。
- Nav2 建图巡游：`mapping_route.json`。
- 房间和搜索视点：`src/semantic_search/config/rooms_aws_house.json`。
- 真实激光地图：`maps/aws_house/house.yaml`。
- 新任务结果：`results/aws_house/tasks/`；验证证据：`results/aws_house/validation/`。

世界共 8 把椅子：厨房 4、卧室 1、阳台/客厅 2、杂物间 1。这里的数字来自上游摆放清单，实际感知结果另按任务输出验收。语音、杯子/瓶子和随机摆放仍需专项验证。
