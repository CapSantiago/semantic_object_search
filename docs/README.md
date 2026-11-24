# 项目文档阅读入口

日常阅读从这里开始。**现行文档集中在 `maintained/`，唯一总计划为 [DEPLOYMENT_PLAN.md](maintained/DEPLOYMENT_PLAN.md)。**

## 建议阅读顺序

| 顺序 | 文档 | 用途 |
|---|---|---|
| 1 | [总计划](maintained/DEPLOYMENT_PLAN.md) | 总目标、框图、里程碑、统一底图/图层、UI与创新安排 |
| 2 | [当前交接](maintained/HANDOFF.md) | 现在做到哪里、下一步、运行配置和关键证据 |
| 3 | [运行指南](maintained/RUNNING.md) | 当前候选的构建、各终端启动、检查和任务命令 |
| 4 | [要求书对照](maintained/REQUIREMENTS_STATUS.md) | 任务一至四的功能完成情况 |
| 5 | [接口说明](maintained/INTERFACES.md) | 输入、结果、状态及坐标约定 |
| 6 | [验证概览](maintained/DEVELOPMENT_VALIDATION.md) | 答辩素材入口、主要通过/失败结果及证据 |

## 专项维护文档

| 文档 | 用途 |
|---|---|
| [家庭场景](maintained/AWS_HOUSE.md) | 原AWS场景运行与背景；最新候选命令看运行指南 |
| [新机器部署](maintained/GITHUB_DEPLOYMENT.md) | 获取源码、依赖与资源恢复 |
| [资源准备](maintained/RESOURCE_PREPARATION.md) | 模型、场景和语音资源 |
| [物品清单](maintained/TARGET_CATALOG.md) | 正式/候选类别、常用验收区域及变化布局 |
| [来源与许可](maintained/SOURCES.md) | 上游项目、教材、参考资料与版本 |

## 目录说明

- [course_materials/](course_materials/README.md)：老师提供的任务书、报告模板和三份教程，原文件保留。
- [archive/](archive/README.md)：旧验收、调研、场景子计划和交接快照，按需追溯。
- [evidence/](evidence/README.md)：精选实验收据、图像与源码快照，保留原路径和原始内容。
- 根目录 [archive/](../archive/README.md)：历史专用工具、压缩后的旧运行数据和日志。

总计划维护目标、里程碑和地图视图方案，交接维护当前状态，接口维护数据与输出契约，验证概览维护证据入口。当前重点为清扫/日常避障及语义地图；导航比较暂缓。文档内按日期保留的旧配置与测试结论用于追溯，最新规则看总计划和交接摘要。

仓库部署不需要复制开发机results。现行docs中的results路径用于本地完整证据追溯；公开精选收据集中在[2026-11-24证据索引](evidence/development_20261124/manifest.json)。部署入口见[新机器部署](maintained/GITHUB_DEPLOYMENT.md)。
