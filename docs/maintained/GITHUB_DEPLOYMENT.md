# GitHub 获取与部署

更新于2026-11-24。本轮发布为持续开发快照，不是项目完工或所有功能正式验收。首次部署先走CPU路径，再按硬件选配GPU；默认杯/瓶/椅与显式实验候选分别使用。

## 仓库内容

保存项目源码、配置、脚本、测试、课程资料、开发计划、来源说明、baseline/AWS基准地图及资源锁定清单。保留原 `baseline` 标记，与 `aws_house` 分开。`resources/github_upload_manifest.json`是11月9日上传整理的历史清单；当前目录迁移见 `resources/maintenance/reorganization_20261117.json`，不改写历史文件散列。现行文档在 `docs/maintained/`，老师原件在 `docs/course_materials/`，阅读入口是 [docs/README.md](../README.md)。

C035文件底图统一由`src/semantic_search/config/map_resources.json`选择，应恢复注册主图，仍存在的兼容副本须保持一致；不要分别编辑副本。会话冻结主YAML/PGM与SHA收据，充电绑定还需匹配所选版本。`save_map.sh`默认另存新版本，拒绝覆盖注册图及已有输出；新地图需重新核对房间、定位和充电标定后才能用于服务。[当前地图用法](RUNNING.md#地图与结果查看)

不上传虚拟环境、build/install/log、上游嵌套 Git 仓库、下载缓存、模型权重、本地音频库、原始深度数组及凭据目录。上游源码和资产保留固定下载来源、提交号、许可证说明与 SHA-256，恢复后位于原路径。既有已跟踪基线证据及docs/evidence精选收据保留；新results默认忽略，文档中的results链接是开发机证据入口；完整运行日志和原始深度数据仍保留本机。

本机历史运行压缩包位于 `archive/runs/`并被Git忽略，包清单及精选证据保留在源码中。新机器如需完整旧运行记录，应另行复制压缩包；当前候选启动方式见 [RUNNING.md](RUNNING.md)。

## 首次部署

运行环境为 x86-64 的 Ubuntu 22.04、系统 Python 3.10、ROS 2 Humble 和 Gazebo Fortress。首次在新机器部署，先按官方说明配置 ROS 和 Gazebo 的软件源。Gazebo 的 RGB-D 渲染需要可用的 OpenGL 环境。[ROS 2 Humble 安装](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/Ubuntu-Install-Debs.rst) · [Gazebo Fortress 安装](https://gazebosim.org/docs/fortress/install_ubuntu/)

安装系统包并准备项目。

```bash
sudo apt update
sudo apt install git build-essential cmake python3-pip python3-venv \
  python3-colcon-common-extensions python3-numpy python3-scipy python3-pytest python3-opencv python3-pil fonts-noto-cjk \
  ros-humble-ros-base ros-humble-rosidl-default-generators \
  ignition-fortress libassimp5 ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ros-gz-interfaces \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-collision-monitor ros-humble-nav2-rotation-shim-controller ros-humble-slam-toolbox \
  ros-humble-robot-state-publisher ros-humble-rviz2 ros-humble-cv-bridge ros-humble-tf2-geometry-msgs \
  ros-humble-message-filters ros-humble-teleop-twist-keyboard
git clone https://github.com/CapSantiago/semantic_object_search.git
cd semantic_object_search
bash scripts/deploy.sh
```

`deploy.sh` 下载并校验固定版本的资产和权重，准备项目内 CPU 视觉环境及独立 Spark-DSG 语义环境，构建 `semantic_search`、`yolo_msgs` 和原生导航适配包 `service_navfn`（构建必需，部分插件用于显式候选）。脚本可重新执行，已通过校验的资源会复用。CPU PyTorch权重轮较大；脚本默认pip读取超时120秒、重试5次，可用环境变量PIP_DEFAULT_TIMEOUT/PIP_RETRIES调整。下载失败不会继续启动或伪报完成，恢复网络后重跑同一命令。旧检出可单独运行 `bash scripts/prepare_semantic.sh` 准备场景图库。每个检出目录使用独立的构建路径，可与另一份检出并存。首次需要联网，准备完成后以 CPU 启动。

```bash
bash scripts/start_robot.sh --device cpu --headless
```

在同一台电脑打开 [http://127.0.0.1:8765](http://127.0.0.1:8765)。查看“系统就绪”、雷达和 RGB-D 数据，点击传感器检查，再选择“去厨房找杯子”测试导航与识别。带桌面的机器去掉 `--headless` 可同时打开 Gazebo、RViz 和网页。用启动终端的 `Ctrl+C` 退出。

## GPU 与语音

有匹配 CUDA 13.0 的 NVIDIA GPU 和驱动时，可选配 GPU 推理。

```bash
bash scripts/prepare_gpu.sh
bash scripts/start_robot.sh --device cuda:0
```

GPU 依赖锁在 `requirements-vision-ubuntu-gpu.txt`，安装到 `.venv-gpu`。驱动或 GPU 不满足要求时，继续使用 `--device cpu`。[PyTorch 安装说明](https://pytorch.org/get-started/locally/) · [CUDA 13.0 驱动要求](https://docs.nvidia.com/cuda/archive/13.0.0/cuda-toolkit-release-notes/)

需要网页录音和转写时准备离线语音环境，然后重启控制台。

```bash
bash scripts/prepare_voice.sh
```

语音资源在独立 `.venv-voice` 中准备。安装后允许浏览器访问麦克风，核对转写再执行指令。可按 `resources/voice_wheels.lock.json` 准备离线 wheel 和模型缓存，再用 `bash scripts/prepare_voice.sh --offline` 恢复。

## 排查与可选资源

检查核心资源散列。

```bash
python3 scripts/prepare_runtime.py --check
```

复现其他布局和设计参考时再下载对应资源。

```bash
python3 scripts/prepare_runtime.py --references
python3 scripts/prepare_target_assets.py
```

`--references` 恢复 RealSense 描述和 TurtleBot3 Classic 对照源码。可选启动参数、键盘建图及原场景复现见运行指南。现行功能与验收进度以总计划和验证概览为准。[运行指南](RUNNING.md) · [总计划](DEPLOYMENT_PLAN.md) · [验证概览](DEVELOPMENT_VALIDATION.md)

## 干净副本实测（2026-11-19基线）

这次记录对应11月19日当时的基线，未覆盖11月21日新增的语义环境、GLB适配和导航保护。本轮干净副本核验另见下方，不把该历史记录当新版本验收。

2026-11-19在开发机用仅含仓库文件的临时副本验证。重新获取了场景、默认布局物品、权重和 CPU 依赖，构建两个 ROS 包并启动网页。10 秒传感器检查通过，厨房找到 1 个杯子，结果已保存。按语音安装步骤重新下载 Whisper，使用合成音频转写成功。补齐旧布局测试资源后，167 项工程测试通过，7 项独立 ROS 用例按开关跳过。目标电脑还需验证自身的图形驱动和 Gazebo 渲染。临时副本及本次验证日志已清理。


## 本轮发布前核验（2026-11-24）

仅含待发布文件的隔离源码副本，全新build/install三包构建9.59秒通过；正式测试536通过、38条件跳过，原生导航3项通过。CPU/headless启动达到网页/导航/感知就绪，10秒传感器检查通过，未发送移动任务；测试根关闭收据stopped/所属残留为空。

本次复用已锁定资源缓存及现有CPU/语义依赖环境。另一独立副本实际尝试重装依赖，官方196.2MB CPU PyTorch下载读取超时，重试吞吐过低，按快速收尾要求停止。因此**未完成全新依赖联网重装，也未做另一台机器验收**。首次部署仍须可用网络、上方系统依赖和OpenGL驱动；失败后按日志重跑，不必清理项目成果。

[本轮原始核验收据与限制](../evidence/deployment_20261124/audit.json)。运行完整工程测试前，先用`python3 scripts/prepare_target_assets.py`恢复测试涉及的可选资产；默认启动只需deploy.sh恢复的基础布局资产。开发机results与个人语义记忆不属于部署必需文件。
