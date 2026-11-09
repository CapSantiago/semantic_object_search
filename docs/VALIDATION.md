# Ubuntu 实测与复现

本文为保留的 **基础测试场景 baseline** 验收记录。新增 AWS 家庭场景使用独立地图与结果目录，当前部署与实测状态见 [AWS_HOUSE.md](AWS_HOUSE.md)。复现本文时，各启动入口统一使用 `scene:=baseline`。

日期：2026-11-08（Asia/Shanghai）。本次先验证基础链路，再执行任务三文本寻物与任务四。没有使用假检测、导航替身或 Gazebo 物体真值驱动搜索。物体真值仅用于验收比较。

## 环境与构建

- Ubuntu 22.04.5、ROS 2 Humble、Gazebo Fortress 6.18.0，系统已有 ros_gz、SLAM Toolbox、Nav2。
- 主机 RTX 4060 Laptop、驱动 580.178.04 可正常读取。视觉验证使用 CPU PyTorch；未验证 CUDA 推理。
- `bash scripts/build.sh` 构建 `yolo_msgs`、`semantic_search` 成功。系统包有一条 `pkg_resources` 版本警告，不影响构建。
- `rosdep check` 的运行依赖满足，但本机 rosdep 数据库不能解析 `ament_python` 键。实际 `ament_python` 构建通过，不需要为这个提示安装来源不明的软件。
- 中文“桌面”路径触发了 rosidl/CMake 和 Nav2 YAML 的两个路径问题。`scripts/env.sh` 建立 `/tmp/semantic_object_search_$UID` 别名；物理构建目录在 `~/.cache/semantic_object_search/build`。源码和安装文件仍在本工程。
- 主机终端原先配置了 TurtleBot4 远程发现服务器。项目脚本默认清除该配置，使用 `ROS_DOMAIN_ID=42`、`ROS_LOCALHOST_ONLY=1`。仅影响当前 shell。如确需连接真实机器人，可在加载脚本前显式设置 `SEMANTIC_USE_SYSTEM_DDS=1`。

每个 ROS 终端必须先在项目根目录运行：

```bash
source scripts/env.sh
```

重新构建：

```bash
bash scripts/build.sh
```

视觉环境实际版本：NumPy 1.26.4、SciPy 1.15.3、OpenCV 4.11.0.86、PyTorch 2.14.1+cpu、torchvision 0.29.2+cpu、Ultralytics 8.4.207。锁定文件是 `requirements-vision-ubuntu-cpu.txt`；完整系统/虚拟环境快照保存在 `results/validation/`。

```bash
# 本机缺少 ensurepip；利用系统已有 pip，无需 sudo 安装 venv 包。
python3 -m venv --without-pip --system-site-packages .venv
.venv/bin/python -m pip install -r requirements-vision-ubuntu-cpu.txt
```

## 基础链路

### RViz 桌面启动修复

Snap 安装的 VS Code 会把 GTK/GIO 模块路径带进集成终端。本机直接 `ros2 run rviz2 rviz2` 已复现 `/snap/core20/.../libpthread.so.0` 的 `__libc_pthread_init / GLIBC_PRIVATE` 错误；这与项目的 Python 视觉虚拟环境无关。

现在使用：

```bash
cd "/home/willmatic/桌面/semantic_object_search"
bash scripts/rviz.sh
```

启动器在自己的进程环境中清理 GTK/GIO/Qt 注入路径，移除动态库路径中的 Snap 项并保留 ROS/Ogre 路径，不修改系统库或 VS Code 配置。实测窗口与 OpenGL 4.6 初始化正常。预设已加入 `/map`、`/scan`、`/search/markers`、TF 和 `/rgbd/image`；Fixed Frame 为 `map`。地图和相机需要相应的仿真/导航节点运行后才有数据。

`scripts/simulation.sh` 同样使用该桌面环境清理。修复日志位于 `results/validation/rviz_fix/`。家具与目标扩展计划见 [SCENE_PLAN.md](SCENE_PLAN.md)。

本次补测已实际打开并保留 RViz 窗口，图形进程未加载 Snap 的 libpthread，地图、雷达、编号三个订阅已核对。此前的仿真和任务验收节点已停止；显示实时地图/相机仍需按启动顺序运行对应节点。

`results/validation/sensors.json` 记录实际订阅结果：

| 项目 | 实测 |
|---|---|
| `/clock` | 仿真时间持续递增 |
| `/odom` | 50 Hz，来自 Gazebo OdometryPublisher |
| `/scan` | 360 点、5 Hz、`base_scan` |
| RGB | 640×480、`rgb8`、`camera_optical_frame` |
| 深度 | 640×480、`32FC1`，单位 m；初始样本有效像素 100% |
| 相机内参 | fx≈fy≈565.6009，cx=320，cy=240 |
| TF | odom→base_footprint、base_footprint→base_scan、base_link→camera_optical_frame 均可查 |
| 底盘 | 发送 0.1 m/s 短时指令后，实际里程计位移 0.2978 m，随后停车 |

Gazebo 使用真实 Ogre2 传感器渲染。`gui:=false` 仅关闭界面，相机和雷达继续工作。日志中出现过 EGL 警告，但实际 RGB、深度和雷达均有有效数据。

复验：

```bash
# 终端 A，保持运行；需要界面时省略 gui:=false。
bash scripts/simulation.sh scene:=baseline gui:=false
# 另一已加载 env.sh 的终端。
python3 scripts/validate_sensors.py --motion
```

`--motion` 会真实驱动仿真底盘；仅在无其他运动控制节点运行时使用。

## 地图和导航

首轮使用 20 个已知安全路点，通过实际 cmd_vel、里程计反馈和雷达防碰撞完成三个房间巡游，耗时 134.4 s。保存 0.05 m 分辨率地图后停止 SLAM，再启动 AMCL/Nav2。房间坐标与地图基本对齐；首轮 SLAM map→odom 偏差约 1.2 cm。

```bash
# 建图时使用，不能与 AMCL 同时运行。
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true \
  slam_params_file:="$SEMANTIC_WS/src/semantic_search/config/slam.yaml"
# 无其他运动控制节点时，可用固定场景巡游脚本。
python3 scripts/drive_waypoints.py
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f "$SEMANTIC_WS/maps/course_house" \
  --ros-args -p use_sim_time:=true
```

停止 SLAM 和巡游程序后，导航终端：

```bash
bash scripts/navigation.sh scene:=baseline
```

该脚本使用项目内 `config/nav2.yaml`，从上游参数派生。目标检查器和 DWB 的 XY 容差统一为 0.12 m，朝向容差为 0.10 rad，避免默认 0.25 m 停点误差造成目标裁切。两个组件的 XY 容差必须一致，否则控制器可能提前进入旋转阶段而无法达到目标检查器要求。

另一已加载 env.sh 的终端：

```bash
python3 scripts/initialize_sim_amcl.py
```

初始化脚本仅适用于当前地图与仿真世界基本对齐的固定场景。更换地图或世界后应重新核对坐标，或在 RViz 用 2D Pose Estimate 初始化。此处使用仿真全局里程计，不代表验证了真实轮式里程计性能。

## 搜索、计数与调试证据

原几何椅子在三个观察点完成导航、取得 37 帧同步数据后仍无有效 YOLO 检出（`task3_geometric.json`）。随后换用有署名的 OpenRobotics Fuel Chair v3，原始资产与许可证记录保存在 `third_party/fuel_chair/`。

换用真实资产后的首个任务三结果为 `found`，访问 2 个观察点、耗时 29.67 s，确认地图点约 (2.337, 1.929, 0.306) m，TF 失败为 0。首个任务四基线访问全部三个观察点，确认 2 个实例，耗时 83.62 s。上述是修复前检测框中心定位的历史数据，保留供比较。

后续跨视角测试发现并保留了失败证据：

1. 框中心落在椅子镂空处，把后墙当作目标深度。
2. 同一个实例的前后表面位置差异足以造成轨迹分裂。
3. 默认 NMS IoU=0.7 时，同一椅子可产生两个重叠部件框。相同真实图像在 IoU=0.5 时保留两把实际椅子。
4. 最近优先的贪心关联在有可行完整匹配时仍可多建轨迹，已由回归测试复现。

最终处理：默认 `yolov8n-seg.pt`，NMS IoU=0.5；通过 `yolo_msgs/Mask` 传递实例轮廓，对实例深度点云排除远背景和地面，再用截尾三维范围估计中心。没有掩膜的外部检测节点仍可使用框内前景深度回退。实例关联使用带距离门限和未匹配选项的全局一对一分配；门限仍为 0.30 m、至少两帧确认。任务之间重建实例表。

在最终地图/精确停点条件下，原入口和后方视点仍发生裁切、漏检，完整巡游曾只返回 1 个（`task4_before_view_calibration.json`）。因此厨房视点重新校正为 `(2.8, 0.65, 2.4)`、`(2.8, 0.85, 1.9)`、`(3.1, 1.25, 2.15)`，第三个值为 yaw/rad。第一个视点实测取得两把椅子的检测和有效深度（`first_chair_view.json`），后两点用于侧视与重复关联。不能把“到达全部观察点”直接等同于“覆盖全部目标”。

每次任务保存 `result.json`、`observations.json` 和收到地图时的 `semantic_map.svg`。`observations.json` 记录每个观察点的检测数、投影点和已确认 ID，便于核对去重，而不是只看最终总数。结果有唯一 `task_id`，验收客户端不会误收 transient-local 缓存中的旧任务结果。

已完成的专项实测：跨视角配置 37 帧、46 个有效投影保持 2 个实例；中心相距 0.75 m 的两把椅子在 27 帧中保持 2 个实例，每个实例各 27 次命中；厨房仅保留一把椅子的重复观察试验取得 40 次有效投影，仍为 1 个实例。对应 `task4_cross_view_final.json`、`task4_close_pair_final.json`、`task4_stationary_final.json`。这些试验完成后，椅子恢复为默认场景位置。

最终资产巡游 21 个路点、实际耗时 146.6 s 后，已重新保存 `maps/course_house.yaml` 并重新启动 AMCL/Nav2。最终结果：

| 验收 | 结果 | 同步帧 | 有效投影 | 仿真耗时 |
|---|---|---|---|---|
| 从起点执行任务三 | 第一个校准视点找到 1 个目标 | 2 | 4 | 30.527 s |
| 默认三个视点执行任务四 | 2 个实例，ID 1、2，各 28 次命中 | 46 | 56 | 25.167 s |
| 同场景单视点对照 | 校准视点即检出 2 个 | 12 | 24 | 13.320 s |

最终多视点结果中，第一个、第二个不同观察位姿分别有 16、12 帧有效目标观测；两者均沿用 ID 1、2，未新增实例。第三点本次无有效目标投影，说明不能根据同步帧数推断目标可见性。专项跨视角配置中，侧面点也可能无检出；逐点实际数据在 `summary.json` 和 `observations.json` 中保留。

单点和多点均数到 2 是当前固定摆放的实测，不证明多点一定优于单点。最终单次定位结果与经 map→odom 对齐的场景模型原点比较，平面误差约 0.088 m、0.192 m（`localization_evaluation.json`）；可见实例中心并不严格等于 CAD 模型原点，这也不是多轮误差统计。

最终核对：`python3 scripts/summarize_validation.py`，输出 `results/validation/summary.json`。它检查实际任务状态、预期数量、TF、SVG 与实际保留的 RViz 编号、传感器、默认入口全新启动、建图、构建和单元测试记录。所有检查通过。地图预览可用 `python3 scripts/plot_result.py results/validation/task4_final.json` 重建。

最后用 `bash scripts/simulation.sh scene:=baseline gui:=false` 全新启动默认 Fuel 椅子场景，`fresh_start/sensors.json` 再次通过。RViz 标记抽样保存在 `markers_final.json`，包含两个位置球和两个编号文本。本轮所有测试节点已停止。

任务结果的 `elapsed_seconds` 使用仿真时间；巡游脚本的 `wall_seconds` 使用实际墙钟时间。

启动感知与搜索：

```bash
source scripts/env.sh
source .venv/bin/activate
export YOLO_CONFIG_DIR="$PWD/results/ultralytics"
ros2 launch semantic_search search.launch.py scene:=baseline with_detector:=true device:=cpu \
  model:="$PWD/yolov8n-seg.pt" output_dir:="$PWD/results/tasks" \
  rooms_file:="$PWD/src/semantic_search/config/rooms.json"
```

首次使用缺失权重时，Ultralytics 会联网下载。`with_detector:=true` 已改为使用激活的虚拟环境 Python；也可显式传入 `detector_python`。

另一个已加载 env.sh 的终端逐条运行，上一条结束后再发下一条：

```bash
python3 scripts/run_search.py '去厨房找椅子' --output results/task3.json
python3 scripts/run_search.py '去厨房统计椅子' --output results/task4.json
```

RViz Fixed Frame 设为 `map`，显示 `/map`、`/scan` 和 MarkerArray `/search/markers`。要在任务之间切换验收场景，可用 `ros2 param set /semantic_search rooms_file <绝对路径>`；下一条任务读取新配置。`tests/scenarios/` 中的局部观察配置用于专门的去重实验，其结果不等同于遍历整个房间的统计。

## 验证范围

文本链路是本轮验收范围。麦克风录音、Whisper 中文转写、CUDA 推理、其他类别/场景、动态目标和开放词汇识别尚未验收。固定视点无法保证任意摆放下的完整覆盖；实例中心是从可见几何估计的地图坐标，仍受遮挡和定位误差影响。

回归测试：

```bash
PYTHONPATH=src/semantic_search python3 -m unittest discover -s tests -v
```

包括镂空背景、毫米深度、掩膜排除遮挡、实例三维变换、同帧相近目标、重复观察、完整匹配反例、任务重置、中文命令和旋转地图标注。
