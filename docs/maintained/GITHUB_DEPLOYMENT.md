# GitHub 获取与部署

更新：2026-11-17。目标仓库为 `CapSantiago/semantic_object_search`，按用户选择使用私有可见性。上传是否完成以 GitHub 上实际可见的提交为准。本说明中的“已验收”均指现有开发机记录；新机器仍需复核环境和传感器。

## 仓库内容

保存项目源码、配置、脚本、测试、课程资料、开发计划、来源说明、两套已验收地图及资源锁定清单。保留原 `baseline` 标记，与 `aws_house` 分开。`resources/github_upload_manifest.json`是11月9日上传整理的历史清单；当前目录迁移见 `resources/maintenance/reorganization_20261117.json`，不改写历史文件散列。现行文档在 `docs/maintained/`，老师原件在 `docs/course_materials/`，阅读入口是 [docs/README.md](../README.md)。

不上传虚拟环境、build/install/log、上游嵌套 Git 仓库、下载缓存、模型权重、本地音频库、原始深度数组及凭据目录。上游源码和资产保留固定下载来源、提交号、许可证说明与 SHA-256，恢复后位于原路径。既有任务 JSON、SVG、地图 PNG、传感器摘要和回归测试日志作为证据收录；完整运行日志和原始深度数据仍保留本机。

本机历史运行压缩包位于 `archive/runs/`并被Git忽略，包清单及精选证据保留在源码中。新机器如需完整旧运行记录，应另行复制压缩包；当前候选启动方式见 [RUNNING.md](RUNNING.md)。

## 首次部署

使用 Ubuntu 22.04、Python 3.10、ROS 2 Humble、Gazebo Fortress。已安装这些系统组件的本机不需要重装。新机器先按 ROS/Gazebo 官方安装文档配置软件源，再核对以下依赖：

```bash
sudo apt update
sudo apt install git python3-pip python3-venv python3-colcon-common-extensions \
  python3-rosdep ignition-fortress ros-humble-ros-gz \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-rviz2 ros-humble-cv-bridge ros-humble-tf2-geometry-msgs \
  ros-humble-message-filters ros-humble-teleop-twist-keyboard
git clone https://github.com/CapSantiago/semantic_object_search.git
cd semantic_object_search
python3 scripts/prepare_runtime.py
source scripts/env.sh
rosdep check --from-paths src third_party/yolo_ros/yolo_msgs --ignore-src --rosdistro humble
bash scripts/build.sh
python3 -m venv --without-pip --system-site-packages .venv
.venv/bin/python -m pip install -r requirements-vision-ubuntu-cpu.txt
```

私有仓库克隆需要自己的 GitHub Git 认证。`prepare_runtime.py` 只下载和校验数据，不启动仿真或推理；若本地存在其他版本的资源会报错，要求先检查，不覆盖它。当前锁定 PyTorch 为 CPU 版本。下载入口可用性与系统包安装在新机器上需要实际核对；不能以本机准备记录替代新机器验收。

已验收运行资源的文件检查：

```bash
python3 scripts/prepare_runtime.py --check
```

## 启动现有任务

完成部署后，从项目根目录运行：

```bash
bash scripts/start_robot.sh
```

自动打开本地UI、Gazebo和RViz；当前使用五区v4、白黄杯布局及可靠传感器订阅。网页选择寻找/统计，终端Ctrl+C退出。可选参数、分终端调试见 [RUNNING.md](RUNNING.md)；原baseline/AWS四区复现见 [AWS_HOUSE.md](AWS_HOUSE.md)。

## 恢复后续准备资源

这些命令仅为资源准备，不表示语音、相机组件或扩展类别已验收：

```bash
python3 scripts/prepare_runtime.py --references
python3 scripts/prepare_target_assets.py
bash scripts/prepare_voice.sh
```

`--references` 额外取回固定版本的 RealSense 描述与 TurtleBot3 Classic 对照源码，不构建驱动。语音准备在独立 `.venv-voice` 安装软件并恢复 Whisper small 和本地 PortAudio，不录音或推理。它不会自动补齐 `downloads/voice_wheels` 离线缓存；如要核对 `verify_resources.py` 的全部检查项，应先按 `resources/voice_wheels.lock.json` 准备对应 wheel 文件。原机缓存可复制后使用 `bash scripts/prepare_voice.sh --offline`。

开发按唯一 [总计划](DEPLOYMENT_PLAN.md) 推进：先在当前导航/视觉配置下完成任务三、四/G0，再完成任务一/G1、任务二及集成验收/G2，之后开展导航比较和视觉探索。任务一、二尚待开发，前面的启动命令仍用于当前寻物/计数功能。一键UI和五区配置已有本机验证，具体结论见 [验证概览](DEVELOPMENT_VALIDATION.md)。
