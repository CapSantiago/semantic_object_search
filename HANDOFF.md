# Ubuntu 和后续 Codex 交接

> 2026-11-09 场景部署更新：默认入口已切换为 `aws_house`，真实激光地图、厨房任务三找 1 把椅子、任务四统计 4 把及编号地图均已通过；15 项回归测试通过。运行命令与证据见 [docs/AWS_HOUSE.md](docs/AWS_HOUSE.md)。本文下方原验收表针对 `baseline`；复现时为仿真、导航、搜索统一加 `scene:=baseline`，或使用新文档中的包装脚本。`maps/course_house.*`、原房间配置和历史结果保留，新场景输出独立目录。

## 任务和工作边界

最新总约束：按 [docs/DEPLOYMENT_PLAN.md](docs/DEPLOYMENT_PLAN.md) 执行，**当前 Dijkstra+DWB / YOLO-seg 先完成任务书 G0，之后才能开展导航算法比较和视觉模型探索**。相机组件、桌面物品、兼容类别、房间覆盖和必要避障输入属于主线功能补齐。可提前下载/安装资源；现已暂存 8 个目标模型、固定版本相机描述、独立 CPU 语音环境及 Whisper small 离线权重，见 [docs/RESOURCE_PREPARATION.md](docs/RESOURCE_PREPARATION.md)。新增资产没有插入现有场景，当前任务白名单仍为 3 类；目标扩展顺序见 [docs/TARGET_CATALOG.md](docs/TARGET_CATALOG.md)。语音识别本身是任务书必做，放在核心执行功能稳定后的 P4、G0 之前；大语言模型是可选后期探索。

2026-11-09 用户明确先完成执行、感知、搜索覆盖、桌面目标、结果和接口，再接语音识别并人工联调。现已保留原文本入口并支持结构化 JSON 任务，新结果显式记录 `map` / `m`。具体接口见 [docs/INTERFACES.md](docs/INTERFACES.md)，要求书差距见 [docs/REQUIREMENTS_STATUS.md](docs/REQUIREMENTS_STATUS.md)。不要把厨房椅子固定布局通过写成全房覆盖或语音功能已完成。

课程项目选定任务四“物体搜寻机器人—扩展”。任务三“单个物体搜寻”是第一验收阶段。老师允许使用 TurtleBot3。最终演示要覆盖语音指定房间和物品、导航到房间、搜索、定位、编号、计数和保存标记地图。关注跨视角去重以及多视角搜索。开放词汇识别、自动选视点和大模型解析留待基础功能通过后再评估。

Windows 阶段编写了源码与配置，并克隆上游。2026-11-08 已在 Ubuntu 22.04.5、Humble、Gazebo Fortress 6.18.0 完成构建与基础联调，并逐步验收文本任务三、四；实测结果见文末和 [docs/VALIDATION.md](docs/VALIDATION.md)。主机有 Nav2、SLAM Toolbox、ros_gz，RTX 4060 Laptop 驱动可用，视觉先验证 CPU。上游位置、许可证和提交号见 SOURCES.md。无需切换至 24.04。

## Ubuntu 构建

GitHub 分发说明见 [docs/GITHUB_DEPLOYMENT.md](docs/GITHUB_DEPLOYMENT.md)。新克隆必须先执行 `python3 scripts/prepare_runtime.py`，再构建；本机已准备的虚拟环境、上游克隆、模型数据和下载缓存不随 Git 上传。已验收地图、资源版本清单及精选验收证据随源码保留。下文“复制完整目录”指包含已恢复依赖的工作目录。

复制完整目录，然后用 VS Code 打开项目根目录。在 Ubuntu 终端进入项目根目录。以下命令按阶段执行。每开一个 ROS 终端，都先进入项目根目录，再 `source scripts/env.sh`；本机存在旧 TurtleBot4 DDS 配置，直接只加载 Humble 会导致无法发现本地节点。

```bash
source scripts/env.sh
# 如 rosdep 可用，先核对缺失依赖。
rosdep check --from-paths src third_party/yolo_ros/yolo_msgs --ignore-src --rosdistro humble
bash scripts/build.sh
source scripts/env.sh
```

如果 rosdep 报缺依赖，确认条目后再安装。常见项有 `ros-humble-cv-bridge`、`ros-humble-tf2-geometry-msgs`、`ros-humble-message-filters`、`ros-humble-teleop-twist-keyboard`。工作空间只构建自己的包和上游 `yolo_msgs`。`third_party/COLCON_IGNORE` 避免无意构建全部上游，`--paths` 直接指向两个包。若 rosdep 跳过被忽略目录，仍以构建消息和 package.xml 为准。

## 1. 验证仿真与数据

终端 A 启动项目场景。保持运行。

```bash
bash scripts/simulation.sh scene:=baseline
```

终端 B 检查 ROS 话题。先确认 /clock、/odom、/tf、/scan、/rgbd/image、/rgbd/depth_image、/rgbd/camera_info 有数据，再用键盘验证底盘移动。Fortress 中也可运行 `ign topic -l` 对比原始 Gazebo 话题。

```bash
ros2 topic list -t
ros2 topic echo /scan --once --field header
ros2 run tf2_ros tf2_echo odom base_scan
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

期望 TF 包含 `odom → base_footprint → base_link → base_scan` 和 `base_link → camera_link → camera_optical_frame`。首次运行若话题名、桥接消息类型、雷达 frame 或相机内参不符，先修这里。采用上游 OdometryPublisher 提供仿真里程计和 TF，报告需说明其来源，不能据此验证真实轮式里程计性能。原几何椅子实测漏检，现默认使用有来源和署名的 Fuel Chair v3；`chair_asset:=''` 可切回原始几何道具用于对照。

## 2. 建图、保存地图、验证导航

终端 C 启动 SLAM，然后用键盘走遍三个房间。RViz2 固定坐标系选 map，加入 Map 与 LaserScan。终端 D 保存地图。

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true \
  slam_params_file:="$SEMANTIC_WS/src/semantic_search/config/slam.yaml"
```

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f "$SEMANTIC_WS/maps/course_house" --ros-args -p use_sim_time:=true
```

建图结束后停止 SLAM 和键盘节点，保留仿真。启动 Nav2，使用保存的地图。SLAM 和 AMCL 不能同时负责 map 到 odom 的坐标变换。

```bash
bash scripts/navigation.sh scene:=baseline
```

RViz2 用 2D Pose Estimate 在机器人当前实际位置初始化 AMCL，再用 Nav2 Goal 测试一个点。用真实建图坐标核对 `src/semantic_search/config/rooms.json` 的房间边界、观察点和朝向，示例配置采用场景坐标，SLAM 地图可能存在偏差。修改配置后可直接通过 `rooms_file` 指定源码里的配置文件。若 Nav2 不能通过房门，检查激光、TF、地图、机器人半径和膨胀层。

## 3. 启动搜索与感知

搜索节点默认独立启动，识别由外部节点提供，也可使用 `with_detector:=true` 联合启动。先用同一 ROS 环境里的系统 Python 创建虚拟环境。NumPy 约束用于兼容 Humble 的 cv_bridge；Ubuntu CPU 实测版本已锁定在 `requirements-vision-ubuntu-cpu.txt`，通用范围保留在 `requirements-vision.txt`。

```bash
python3 -m venv --without-pip --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements-vision-ubuntu-cpu.txt
python -m semantic_search.detector --ros-args -p use_sim_time:=true -p device:=cpu
```

默认权重已改为 `yolov8n-seg.pt`，首次运行可能下载。已有权重可用 `-p model:=/path/model.pt`，先验证 CPU。当前锁定的是 CPU PyTorch，不能直接切换 `device:=0`；需另行安装、验证 CUDA 版本。如果使用其他检测节点，确保它发布 `yolo_msgs/msg/DetectionArray`，框和可选掩膜对应 RGB 像素，header 保留原图采集时刻。

另一终端启动搜索，打开 RViz2 的 MarkerArray 显示，订阅 `/search/markers`。

```bash
ros2 launch semantic_search search.launch.py scene:=baseline output_dir:="$PWD/results" \
  rooms_file:="$PWD/src/semantic_search/config/rooms.json"
```

```bash
ros2 topic pub --once /search/command std_msgs/msg/String "{data: '去厨房找椅子'}"
# 第一条结束后，再发任务四
ros2 topic pub --once /search/command std_msgs/msg/String "{data: '去厨房统计椅子'}"
ros2 topic echo /search/result --qos-durability transient_local --once
```

`results/<任务时间>/result.json` 记录任务 ID、状态、目标数、地图坐标、观察点和传感器诊断，`observations.json` 记录逐次投影和确认编号。收到 `/map` 时还会导出 `semantic_map.svg`。搜索节点同步检测、深度和相机内参，按检测时间查 TF。默认使用实例掩膜和前景点云的稳健三维中心；无掩膜时回退到框内前景深度。关联改为全局一对一匹配，默认 0.30 m、至少两帧确认。任务间清空实例表。具体修复与失败对比见验收文档。

## 4. 语音与验收

此阶段按总计划放在 P4。已提前准备独立 `.venv-voice`、本地 PortAudio 和 Whisper small 权重，不向 `.venv` 视觉环境安装语音依赖。运行后按回车录音，转写发送给搜索节点；麦克风与端到端识别仍未实测。准备状态见 [docs/RESOURCE_PREPARATION.md](docs/RESOURCE_PREPARATION.md)。

```bash
bash scripts/voice.sh
```

后续开发顺序：

1. 先在 Ubuntu 用基础仿真、雷达、相机和 TF 验证链路。记录实际话题、深度单位和 optical frame。
2. 完成一次建图与自主导航。对照真实地图校正房间配置，再验证 task 3 的文本寻物和定位。
3. 在同一位置反复观察同一目标，验证不重复计数。再放置两个相近实例，检查是否误合并；最后使用语音发出统计命令。
4. 做固定场景的对比，记录单点和多点搜索结果、重复计数、定位误差和耗时。场景模型真值用于评估。验收结果在此文档追加，明确通过的环节和待修问题。

## 2026-11-08 Ubuntu 验收追加

已完成本轮授权的基础链路、任务三文本寻物和任务四计数/标注。完整复现、失败对比和环境版本见 [docs/VALIDATION.md](docs/VALIDATION.md)，机器可读核对见 [results/validation/summary.json](results/validation/summary.json)。

| 项目 | 实测结果 |
|---|---|
| 构建 | `yolo_msgs`、`semantic_search` 均成功；修复中文路径的消息生成/地图加载问题 |
| Gazebo/底盘 | Fortress 6.18.0 实际渲染；短时运动验证位移 0.2978 m |
| 雷达/RGB-D/TF | 360 点 5 Hz；RGB-D 640×480；深度 `32FC1`、m；光学帧 `camera_optical_frame`；TF 查询通过 |
| 最终地图 | 恢复默认目标后，用最终网格椅子巡游 21 个路点，实际耗时 146.6 s；地图 `maps/course_house.yaml` |
| 任务三 | 从起点自主导航，在第一个校准视点找到椅子，30.527 仿真秒；坐标约 (1.392, 1.758, 0.438) m |
| 任务四 | 3 个视点，46 帧同步、56 个有效投影，2 个实例，每个命中 28 次；25.167 仿真秒，TF 失败 0 |
| 重复观察 | 厨房仅一把目标时，40 次有效投影仍为 1 个实例 |
| 相近实例 | 两把椅子中心相距 0.75 m，27 帧均分别关联为 2 个实例 |
| 回归测试 | 12 项通过，覆盖镂空背景、掩膜、三维变换、全局关联、近邻、任务重置、命令和 SVG |

首版几何椅子不能稳定识别，已换为已署名的 Fuel Chair v3。默认模型改为 `yolov8n-seg.pt`、NMS IoU=0.5；使用实例点云稳健中心与全局一对一关联，避免后墙误投影和部件重叠框重复计数。合并半径仍为 0.30 m。最终厨房视点已按实际画面校正，来源和许可已更新至 SOURCES.md。

最终 [任务三 JSON](results/validation/task3_final.json)、[任务四 JSON](results/validation/task4_final.json)、[带编号 SVG 地图](results/tasks/20261108T080157_272160Z/semantic_map.svg)、[地图 PNG 预览](results/validation/semantic_map.png) 均已保存。单点与多点对比、逐帧实例 ID 和定位误差数据也保留在 `results/validation/` 及对应任务目录。

当前未验收：麦克风/语音转写、CUDA 推理、其他目标类别、动态场景和开放词汇。按用户最新顺序，先补核心搜索覆盖、桌面物品、接口及自动验收，再接现成语音识别/关键词解析并人工联调，完成任务书 G0；导航比较、视觉模型和大语言模型探索在 G0 后进行。不要把当前固定场景通过等同于任意场景都可靠。

后续优先级根据用户反馈更新为：先修复 RViz 桌面启动并优化房间陈设、物体数量和视点覆盖，重新建图与验收任务三/四，再接入语音。AWS 家具场景现已接入，状态和指令见 [docs/AWS_HOUSE.md](docs/AWS_HOUSE.md)。RViz 请用 `bash scripts/rviz.sh`，该脚本已处理 Snap VS Code 注入 GTK/GIO 库导致的 GLIBC_PRIVATE 错误，且预设地图、雷达、编号与 RGB 显示。

已按用户要求通过 GitHub 与网络调研现成住宅场景：[docs/SCENE_RESEARCH.md](docs/SCENE_RESEARCH.md)。已选用 AWS Small House 的房屋布局和独立模型适配 Fortress；Lake House 为备选，RoboCup@Home 可补充任务物品。原调研记录保留，部署与本机运行测试在 AWS 文档中单独记录。

最后已用 `bash scripts/simulation.sh scene:=baseline gui:=false` 全新启动默认真实资产场景，再次验证传感器与 TF 通过（`results/validation/fresh_start/`）。JSON、SVG 与 RViz 标记编号一致；本轮测试节点已停止，按验收文档即可重新启动。
