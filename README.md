# 家庭服务机器人：建图、清扫、寻物与盘点

项目面向Ubuntu 22.04、ROS 2 Humble和Gazebo Fortress，完整目标是任务一至四：语音建图、房间清扫、寻找物品、统计与地图标注。共用家庭场景、地图、导航、视觉和统一UI。

当前已实测仿真、雷达/RGB-D/TF、定位导航、寻物和去重计数，P1–P3 基线已验收。网页支持离线语音转写、建图和房间清扫，能显示实际覆盖和轨迹。真人麦克风试用与清扫整机验收继续按总计划推进。四项功能验收后再拓展持久语义地图、主动观察与清扫观察协同。

## 一键运行

在项目根目录运行。首次部署先按系统依赖清单安装，再执行 `bash scripts/deploy.sh`。[新电脑部署说明](docs/maintained/GITHUB_DEPLOYMENT.md)

```bash
bash scripts/start_robot.sh --device cpu --headless
```

网页默认地址为 `http://127.0.0.1:8765`。带桌面的机器去掉 `--headless` 可同时打开 Gazebo 和 RViz。已准备 CUDA 环境的机器可用 `--device cuda:0`。等待网页就绪后选择任务，终端按 `Ctrl+C` 退出。

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

根目录 `HANDOFF.md`、`SOURCES.md`只保留兼容链接；协作规则见 [AGENTS.md](AGENTS.md)。
