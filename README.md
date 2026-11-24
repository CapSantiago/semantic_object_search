# 家庭服务机器人：建图、清扫、寻物与盘点

当前以答辩素材为导向：展示统一四功能系统和有证据的亮点，16类全面验收与全屋清扫不作为前提。清扫重点卧室/杂物间，按≥80%并完成原可达规划条带；散落球障碍另组验证，保留真实partial。厨房/卧室两处支撑面重点寻物计数，厨房杯移动后的查找/计数/状态更新单轮通过，完整提效与重复稳定性仍待完善。详见唯一总计划与交接。

项目面向Ubuntu 22.04、ROS 2 Humble和Gazebo Fortress，完整目标是任务一至四：语音建图、房间清扫、寻找物品、统计与地图标注。共用家庭场景、地图、导航、视觉和统一UI。

当前继续开发与优化。可公开核查的本轮收据见 [精选证据](docs/evidence/development_20261124/manifest.json)，完整传感器数据、运行会话及原有PPT/报告保留在开发机，不随源码仓库分发。

当前三类目标的P1–P3基线已验收，网页已接通建图、清扫、寻物、盘点和离线语音。语义记忆与主动观察已接入，实际收益仍需正确对照；本轮卧室84.85%、杂物间92.74%均全原条带/回充单次通过，球障碍另测。杯/瓶/椅为正式默认，碗/书仍为显式候选，单次与正式稳定验收分别记录。避障按4项日常代表场景验证，导航比较暂缓；预检后由用户核验真人麦克风。[当前状态与证据](docs/maintained/HANDOFF.md)

地图采用同一场景版本的栅格底图叠加房间、物品、轨迹和覆盖等图层，按需导出视图。文件底图已有公共选择/校验入口，兼容副本和历史地图保留，建图另存新版本；7类空间视图及关系图可从UI统一导出PNG/SVG/原始JSON，局部代价尚未接入，详见[地图方案](docs/maintained/DEPLOYMENT_PLAN.md#地图数据与输出视图)。

## 一键运行

在项目根目录运行。首次部署先按系统依赖清单安装，再执行 `bash scripts/deploy.sh`。[新电脑部署说明](docs/maintained/GITHUB_DEPLOYMENT.md)

```bash
bash scripts/start_robot.sh --device cpu --headless
```

网页默认地址为 `http://127.0.0.1:8765`。带桌面的机器去掉 `--headless` 可同时打开 Gazebo 和 RViz。已准备 CUDA 环境的机器可用 `--device cuda:0`。等待网页就绪后选择任务，终端按 `Ctrl+C` 退出。

当前默认v6房间配置和service_surfaces_v2布局：厨房巡视餐桌与L形操作台，卧室巡视书桌与床头柜；旧v4/v5及布局保留。开发导航线速度上限0.35米/秒；显式展示候选已有卧室84.85%、杂物间92.74%且原条带完整的单次结果，不将其当默认配方全部稳定性通过。默认出生于客厅西侧固定充电处。寻物、盘点保存结果后自动回充，清扫及建图结束切回导航后也回到同一位置；可单独下达“返回充电处”。停止/取消会停车，已保存结果保留。最新集成验证与完整覆盖状态见交接，不代表16类或整机已验收。

## 从这里开始

| 需要做什么 | 文档 |
|---|---|
| 查看全部文档 | **[阅读索引](docs/README.md)** |
| 看总目标、框图和里程碑完成状态 | [总计划](docs/maintained/DEPLOYMENT_PLAN.md) |
| 接着开发 | [最新交接](docs/maintained/HANDOFF.md) |
| 在当前机器启动项目 | [运行指南](docs/maintained/RUNNING.md) |
| 在新机器部署 | [GitHub部署说明](docs/maintained/GITHUB_DEPLOYMENT.md) |
| 核查任务完成和实验依据 | [要求对照](docs/maintained/REQUIREMENTS_STATUS.md)、[验证概览](docs/maintained/DEVELOPMENT_VALIDATION.md) |
| 阅读老师资料 | [课程原件](docs/course_materials/README.md) |

## 工程目录

| 目录 | 用途 |
|---|---|
| `src/`、`tests/` | 运行源码及回归测试 |
| `ui/` | 本地控制台静态界面，无前端构建依赖 |
| `scripts/` | 当前构建、启动、抓帧、导航和验证工具 |
| `docs/maintained/` | 集中维护的计划、交接、接口、运行及来源文档 |
| `docs/course_materials/` | 老师任务书、教程和报告模板 |
| `docs/archive/`、`docs/evidence/` | 历史文档与精选实验证据 |
| `maps/`、`scenes/` | 原场景与候选的地图、布局和房间配置 |
| `resources/` | 资源版本锁定、校验清单和维护索引 |
| `results/` | 当前输出及保留的原场景验收基准 |
| `archive/` | 历史专项工具、已校验的旧运行/日志压缩包 |
| `third_party/`、`skills/` | 上游资源及项目技能 |

原baseline和AWS场景、地图及失败证据保留。大型依赖、资产、虚拟环境和本机归档包不随Git上传，部署时按说明恢复。源码来源与许可见 [SOURCES](docs/maintained/SOURCES.md)。

根目录 `HANDOFF.md`、`SOURCES.md`只保留简短阅读入口；协作规则见 [AGENTS.md](AGENTS.md)。
