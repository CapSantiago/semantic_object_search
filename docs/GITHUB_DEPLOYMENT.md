# GitHub 获取与部署

更新：2026-11-09。目标仓库为 `CapSantiago/semantic_object_search`，按用户选择使用私有可见性。上传是否完成以 GitHub 上实际可见的提交为准。本说明中的“已验收”均指现有开发机记录；新机器仍需复核环境和传感器。

## 仓库内容

保存项目源码、配置、脚本、测试、课程资料、开发计划、来源说明、两套已验收地图及资源锁定清单。保留原 `baseline` 标记，与 `aws_house` 分开。发布清单 `resources/github_upload_manifest.json` 列出本次收录文件和精选历史证据；证据中的路径、版本及时间为原测试环境记录。

不上传虚拟环境、build/install/log、上游嵌套 Git 仓库、下载缓存、模型权重、本地音频库、原始深度数组及凭据目录。上游源码和资产保留固定下载来源、提交号、许可证说明与 SHA-256，恢复后位于原路径。既有任务 JSON、SVG、地图 PNG、传感器摘要和回归测试日志作为证据收录；完整运行日志和原始深度数据仍保留本机。

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

每个终端进入项目根目录。按下面顺序启动并保持各终端运行，详细等待条件和 AMCL 初始化见 [AWS_HOUSE.md](AWS_HOUSE.md)。

```bash
# 终端 1：仿真
bash scripts/simulation.sh scene:=aws_house
# 终端 2：导航
bash scripts/navigation.sh scene:=aws_house
# 终端 3：地图与编号可视化
bash scripts/rviz.sh scene:=aws_house
# 终端 4：视觉与搜索
bash scripts/search.sh scene:=aws_house
```

在额外终端中，等待导航和感知就绪后发送任务：

```bash
source scripts/env.sh
# 本项目存档地图与仿真坐标对齐，可用此脚本初始化 AMCL 并等待导航激活。
# 自己重建且坐标未对齐的地图应改用 RViz 的 2D Pose Estimate。
python3 scripts/initialize_sim_amcl.py
ros2 topic pub --once /search/command std_msgs/msg/String "{data: '去厨房找椅子'}"
# 上一任务结束后再执行计数
ros2 topic pub --once /search/command std_msgs/msg/String "{data: '去厨房统计椅子'}"
```

四个启动终端统一改成 `scene:=baseline` 可复现原测试场景。任务输出仍写入独立 `results/` 目录，话题、JSON 输入及坐标约定见 [INTERFACES.md](INTERFACES.md)。

## 恢复后续准备资源

这些命令仅为资源准备，不表示语音、相机组件或扩展类别已验收：

```bash
python3 scripts/prepare_runtime.py --references
python3 scripts/prepare_target_assets.py
bash scripts/prepare_voice.sh
```

`--references` 额外取回固定版本的 RealSense 描述与 TurtleBot3 Classic 对照源码，不构建驱动。语音准备在独立 `.venv-voice` 安装软件并恢复 Whisper small 和本地 PortAudio，不录音或推理。它不会自动补齐 `downloads/voice_wheels` 离线缓存；如要核对 `verify_resources.py` 的全部检查项，应先按 `resources/voice_wheels.lock.json` 准备对应 wheel 文件。原机缓存可复制后使用 `bash scripts/prepare_voice.sh --offline`。

开发仍按 [DEPLOYMENT_PLAN.md](DEPLOYMENT_PLAN.md) 推进：先在当前导航/视觉配置下完成任务书 G0，再开展导航方法比较和视觉探索。上传整理没有执行新的仿真、推理或算法实验。
