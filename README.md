# 家庭服务机器人：建图、清扫、寻物与盘点

项目面向Ubuntu 22.04、ROS 2 Humble和Gazebo Fortress，完整目标是任务一至四：语音建图、房间清扫、寻找物品、统计与地图标注。共用家庭场景、地图、导航、视觉和统一UI。

当前已实测仿真、雷达/RGB-D/TF、SLAM、定位导航、文本寻物和去重计数。主线处于P3稳定性与覆盖收尾；已接入本地UI与一键启动，完整语音建图和清扫功能仍按计划开发。后续拓展持久语义地图、主动观察及清扫观察协同。

## 一键运行

```bash
cd "/home/willmatic/桌面/semantic_object_search"
bash scripts/start_robot.sh
```

自动打开网页、Gazebo和RViz；等待就绪后在网页选择寻找或统计。终端按 `Ctrl+C` 退出。依赖未准备的新机器先看部署说明；可选参数及分终端调试见运行指南。

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
