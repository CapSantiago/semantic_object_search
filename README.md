# 室内语音寻物与语义地图

《自主移动机器人》课程综合设计，按任务四选题，先验收任务三。老师已确认允许使用 TurtleBot3。项目面向 Ubuntu 22.04.5、ROS 2 Humble、Gazebo Fortress 6.x。已在 Ubuntu 实测仿真、雷达、RGB-D、SLAM、Nav2、文本寻物、去重计数和地图标注；详细结果见 [验收记录](docs/VALIDATION.md)。

- **任务三**：例如“去厨房找一把椅子”。到达配置的观察点，定位并确认第一个实例，返回地图坐标。
- **任务四**：例如“去厨房统计椅子”。逐个观察点搜索，同一帧执行一对一匹配，跨帧按地图坐标合并重复观测，输出实例编号、数量及带标记的地图。
- **输入和输出**：当前使用文本/结构化 JSON，输出 RViz2 标记、结果话题、JSON 和 SVG。语言理解目前为有限类别和房间的规则解析；语音入口与离线依赖已准备，尚未端到端验收。

## 从哪里开始

后续开发的优先级入口是 [部署与开发总计划](docs/DEPLOYMENT_PLAN.md)：**先沿用当前配置完成任务书及语音/人工完整验收，再做导航比较和视觉探索**。已提前准备的目标模型、相机参考、独立语音环境和离线权重见 [资源部署记录](docs/RESOURCE_PREPARATION.md)，计划扩展的 8/12 类目标及准入条件见 [目标清单](docs/TARGET_CATALOG.md)。资源已下载不代表类别已开放或功能已验收。

新增 AWS 家庭展示场景，原基础测试场景保留并标注。两套场景使用独立地图、房间配置和结果目录，通过 `scene:=aws_house` / `scene:=baseline` 切换。当前部署状态及各终端启动指令见 [家庭场景运行文档](docs/AWS_HOUSE.md)。

2026-11-09 已完成 AWS 场景的实际激光建图、AMCL/Nav2、厨房单物体搜索和四把椅子计数/编号验收，15 项回归测试通过。[汇总结果](results/aws_house/validation/summary.json) 和 [编号地图](results/aws_house/validation/final_run/semantic_map.png) 已保存。语音和其他目标类别仍待专项验证。

后续按“先核心功能、再模型接入、最后人工联调”推进。当前要求书差距与创新规划见 [REQUIREMENTS_STATUS.md](docs/REQUIREMENTS_STATUS.md)，新增结构化任务输入和结果坐标约定见 [INTERFACES.md](docs/INTERFACES.md)。接口补充后共 18 项回归通过；旧场景实测记录保留。

1. 从 GitHub 获取项目后，按 [仓库部署说明](docs/GITHUB_DEPLOYMENT.md) 安装系统依赖，运行 `python3 scripts/prepare_runtime.py` 下载固定版本的上游代码、场景与视觉权重。仓库包含两套已验收地图；虚拟环境、上游 Git 克隆和大模型数据通过脚本重建。不要只复制 `src`。
2. 在 VS Code 打开这个目录，让 Codex 先读 [HANDOFF.md](HANDOFF.md)。其中包含构建、启动、验收顺序和已知风险。
3. 源码来源、许可证、提交版本和引用方式见 [SOURCES.md](SOURCES.md)。

在项目根目录运行 `bash scripts/build.sh`。每个 ROS 终端先运行 `source scripts/env.sh`，它为中文目录建立 ASCII 别名，并隔离主机遗留的远程 DDS 配置。仿真使用 `bash scripts/simulation.sh`，已保存地图的导航使用 `bash scripts/navigation.sh`。完整启动顺序及视觉环境安装见交接和验收文档。

RViz 使用 `bash scripts/rviz.sh`，已处理 Snap VS Code 的桌面运行库冲突并预设地图、雷达、物体编号和 RGB 显示。默认使用 AWS 家庭场景，原 8×6 m 基础场景通过 `scene:=baseline` 运行。后续物品、语音与遮挡实验见 [场景优化计划](docs/SCENE_PLAN.md)。

## 目录

| 路径 | 用途 |
|---|---|
| `src/semantic_search/launch/` | Fortress 场景与 ROS 桥接、搜索应用启动 |
| `src/semantic_search/config/` | 房间范围和观察点、建图参数、机器人固定坐标系 |
| `src/semantic_search/semantic_search/` | 任务调度、视觉检测、深度定位、计数融合、语音输入 |
| `third_party/` | 固定版本的上游原始源码、住宅和家具模型资产 |
| `requirements-vision.txt`、`requirements-voice.txt` | 可选感知和语音依赖 |
| `scripts/`、`tests/` | Ubuntu 构建/启动、实际传感器及任务验收、关联与深度回归测试 |
| `maps/`、`results/` | 在 Ubuntu 中生成的地图和任务结果 |
