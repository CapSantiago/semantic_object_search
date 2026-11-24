# AWS家庭场景

基于固定版本AWS Small House，保留87个原始家具和建筑实例，原测试场景独立保留为`baseline`。当前一键入口使用五区v4与白黄杯布局，默认GPU n/640；多类与宽视场是显式启用的候选。运行方法见[运行指南](../../docs/maintained/RUNNING.md)，验收状态见[交接摘要](../../docs/maintained/HANDOFF.md)。

- 来源、提交、许可及原场景适配：[AWS_HOUSE.md](../../docs/maintained/AWS_HOUSE.md)、`source.json`；上游原件为`third_party/aws_small_house/`。
- 房间与观察点：`src/semantic_search/config/rooms_aws_house_v4.json`；五区坐标见`spatial_v4.json`。旧`rooms_aws_house.json`用于原场景回归。
- 当前默认布局：`layout_r0_yellow_v1.json`；多位置与多类布局独立保留，逐类进度见[物品清单](../../docs/maintained/TARGET_CATALOG.md)。
- 默认导航实际载入：`scenes/aws_house/r0_v1/maps/house.yaml`。`maps/aws_house/`、`scenes/aws_house/maps/`保留同版兼容副本；逻辑上共用一份底图，统一载入与视图导出按[总计划](../../docs/maintained/DEPLOYMENT_PLAN.md#地图数据与输出视图)推进。
- 原场景建图巡游：`mapping_route.json`。一键运行会话记录在`results/sessions/`，任务数据及历史验证位置以运行指南和收据为准。
- `ground_truth.json`记录上游实例世界坐标；布局文件同样只供场景生成和事后评估，不能进入感知、搜索或计数决策。

原住宅有8把椅子；具体区域归属按实际map位置与五区配置判定，实际数量由传感器识别后验收。杯、瓶、椅的基线和合成语音已有实测，16类常用区域变化布局、清扫覆盖、最终避障与真人麦克风仍按总计划完成。
