# 上游源码与课程资料

保留的第三方 Git 克隆位于 `third_party/`，可在相应目录中运行 `git log -1`、`git remote -v` 核对来源。上游文件在各自许可证下分发，本项目新增代码位于 `src/semantic_search`。

GitHub 仓库通过 `resources/runtime.lock.json` 与 `scripts/prepare_runtime.py` 恢复这些固定提交，不重复上传嵌套 Git 仓库及大体积资产。Fuel Chair 的原始来源元数据、归档 SHA-256 与署名也保存在该清单中，下载后恢复 `SOURCE.json` 和 `ATTRIBUTION.md`。语音和候选物品分别使用已有锁定清单与准备脚本。克隆部署流程见 [docs/GITHUB_DEPLOYMENT.md](docs/GITHUB_DEPLOYMENT.md)。

2026-11-09 提前资源准备：`third_party/target_assets/` 暂存八个 Fuel 杯/瓶/碗/书/手机模型，逐项作者、版本、许可、URL 和 SHA-256 见 `resources/assets.lock.json` 及各模型 `SOURCE.json`，未加入运行场景。`third_party/realsense_ros_reference` 为 [官方 RealSense ROS 仓库](https://github.com/IntelRealSense/realsense-ros) `ros2-master` commit `7a87097c499e3b0b69518d54a8e7a9ac1924e87c` 的相机描述参考，保留 LICENSE/NOTICE，未构建驱动或替换相机。语音数据来自 [Systran/faster-whisper-small](https://huggingface.co/Systran/faster-whisper-small)，固定 revision 与文件校验值见 `resources/voice_model.lock.json`；在独立语音环境准备，未做录音/推理。详细状态见 `docs/RESOURCE_PREPARATION.md`。

新增 `third_party/aws_small_house`：[AWS Robotics / Small House World](https://github.com/aws-robotics/aws-robomaker-small-house-world)，`ros2` 分支固定提交 `ff9631ca6d1db9c1ba656498151464b5ab74aafe`。保留其全部原始资产和 LICENSE；根 LICENSE 为 Amazon 的宽松授权文本，而 package.xml 声明 Apache 2.0，两个原文件均保留供核对。不编译其 Classic 包；本工程读取 87 个静态实例，在临时文件中修复 Collada 照片路径、移除 Classic layer 元数据和不完整静态惯性，地板碰撞使用 Fortress 地平面，家具摆放和尺寸保留。来源清单及世界文件校验值见 `scenes/aws_house/source.json`，实例真值仅作评估。

| 本地目录 | 上游 / 分支 / 克隆时提交 | 许可证与项目内用途 |
|---|---|---|
| `third_party/tb3_fortress` | [Onicc/navigation2_ignition_gazebo_turtlebot3](https://github.com/Onicc/navigation2_ignition_gazebo_turtlebot3) · `main` · `115bcc30b0f5dc6bdfb918df64c94b88cf953657` | Apache-2.0。直接读取 Waffle SDF、网格、基础世界与 Nav2 参数。在本项目启动文件中生成派生场景，加 RGB-D、房间和搜索目标，采用当前系统的 `ros_gz_bridge`。原始上游文件保留原样。 |
| `third_party/turtlebot3_simulations` | [ROBOTIS-GIT/turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) · `humble` · `a35a56c8b04877dc89772b598084d8ce648a9023` | Apache-2.0。课程 Gazebo Classic 路线和原模型的对照源码。当前 Fortress 工程不编译这份仓库。 |
| `third_party/yolo_ros` | [mgonzs13/yolo_ros](https://github.com/mgonzs13/yolo_ros) · `main` · `94d2dd500cb26027ee9c019f4d3a02029ac615cc` | 仓库根目录为 GPL-3.0，`yolo_msgs/` 有单独的 MIT 许可证。当前只构建 `yolo_msgs`，复用 DetectionArray 接口。检测节点调用 Ultralytics Python API，避免将上游其余检测组件和当前 ONNX 工具链引入基础环境。 |

新增 `third_party/fuel_chair`：[OpenRobotics / Chair](https://fuel.gazebosim.org/1.0/OpenRobotics/models/Chair)，Fuel 版本 3，CC BY 4.0，作者 Cole Biesemeyer / Open Robotics。用于替换实测漏检的几何椅子。保留原资产；启动时修改实例名称、位置、静态属性和网格路径。下载元数据、SHA-256 和署名见目录内 `SOURCE.json`、`ATTRIBUTION.md`。

`src/semantic_search/config/nav2.yaml` 派生自固定版本 Onicc `waffle.yaml`（Apache-2.0），将目标检查器和 DWB 的 XY 到达容差统一为 0.12 m、朝向容差设为 0.10 rad，以提高相机观察点的可复现性。原上游配置保留。

AWS 场景继续使用原 `yolov8n-seg.pt` CPU 检测，按真实图像校准四个厨房视点，并在项目启动参数中将 NMS IoU 设为 0.30；基础场景保留 0.50。检测置信度均保持 0.35。此处只调整本工程配置，未修改上游检测库或模型文件。

Nav2、SLAM Toolbox、ros_gz 使用 Ubuntu 中已安装的 Humble 系统组件。应用通过 Nav2 的 `NavigateToPose` 发出目标，现有 Nav2 配置负责规划和控制。Ultralytics 运行库及权重的许可请以对应发行版本为准，首次下载的权重不在本目录。

课程任务书《设计任务及要求2026》、配套《设计教程》《设计教程2》《设计教程3》位于 `docs/`。课程教程以 Humble、TurtleBot3、Gazebo Classic 为例，第三份教程的任务顺序、YOLO 消息和 RGB-D 定位流程是本项目的设计参考。Fortress 与 Classic 的插件和启动方式不同，所以本项目另写了仿真入口。

研究背景可参考 [WoMAP](https://proceedings.mlr.press/v305/yin25b.html) 和 [OpenIN](https://ieeexplore.ieee.org/document/11091457/)。本项目借鉴语言指定目标和语义地图的任务设定，采用静态场景、固定类别、预设视点及简单跨帧融合；不复现论文算法。
