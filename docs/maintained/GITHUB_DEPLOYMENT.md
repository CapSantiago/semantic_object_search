# GitHub 获取与部署

更新于 2026-11-19。项目位于 GitHub 的公开仓库 `CapSantiago/semantic_object_search`。首次部署先走 CPU 路径，确认仿真和视觉工作后可选配 NVIDIA GPU。

## 仓库内容

保存项目源码、配置、脚本、测试、课程资料、开发计划、来源说明、两套已验收地图及资源锁定清单。保留原 `baseline` 标记，与 `aws_house` 分开。`resources/github_upload_manifest.json`是11月9日上传整理的历史清单；当前目录迁移见 `resources/maintenance/reorganization_20261117.json`，不改写历史文件散列。现行文档在 `docs/maintained/`，老师原件在 `docs/course_materials/`，阅读入口是 [docs/README.md](../README.md)。

不上传虚拟环境、build/install/log、上游嵌套 Git 仓库、下载缓存、模型权重、本地音频库、原始深度数组及凭据目录。上游源码和资产保留固定下载来源、提交号、许可证说明与 SHA-256，恢复后位于原路径。既有任务 JSON、SVG、地图 PNG、传感器摘要和回归测试日志作为证据收录；完整运行日志和原始深度数据仍保留本机。

本机历史运行压缩包位于 `archive/runs/`并被Git忽略，包清单及精选证据保留在源码中。新机器如需完整旧运行记录，应另行复制压缩包；当前候选启动方式见 [RUNNING.md](RUNNING.md)。

## 首次部署

运行环境为 x86-64 的 Ubuntu 22.04、系统 Python 3.10、ROS 2 Humble 和 Gazebo Fortress。首次在新机器部署，先按官方说明配置 ROS 和 Gazebo 的软件源。Gazebo 的 RGB-D 渲染需要可用的 OpenGL 环境。[ROS 2 Humble 安装](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/Ubuntu-Install-Debs.rst) · [Gazebo Fortress 安装](https://gazebosim.org/docs/fortress/install_ubuntu/)

安装系统包并准备项目。

```bash
sudo apt update
sudo apt install git build-essential cmake python3-pip python3-venv \
  python3-colcon-common-extensions python3-numpy python3-scipy python3-opencv \
  ros-humble-ros-base ros-humble-rosidl-default-generators \
  ignition-fortress ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-robot-state-publisher ros-humble-rviz2 ros-humble-cv-bridge ros-humble-tf2-geometry-msgs \
  ros-humble-message-filters ros-humble-teleop-twist-keyboard
git clone https://github.com/CapSantiago/semantic_object_search.git
cd semantic_object_search
bash scripts/deploy.sh
```

`deploy.sh` 下载并校验固定版本的资产和权重，准备项目内 CPU 环境，构建 `semantic_search` 和 `yolo_msgs`。脚本可重新执行，已通过校验的资源会复用。每个检出目录使用独立的构建路径，可与另一份检出并存。首次需要联网，准备完成后以 CPU 启动。

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

## 干净副本实测

2026-11-19 在开发机用仅含仓库文件的临时副本验证。重新获取了场景、默认布局物品、权重和 CPU 依赖，构建两个 ROS 包并启动网页。10 秒传感器检查通过，厨房找到 1 个杯子，结果已保存。按语音安装步骤重新下载 Whisper，使用合成音频转写成功。补齐旧布局测试资源后，167 项工程测试通过，7 项独立 ROS 用例按开关跳过。目标电脑还需验证自身的图形驱动和 Gazebo 渲染。临时副本及本次验证日志已清理。
