# AWS 家庭场景部署与运行

当前默认白黄杯/v4五区/可靠订阅配置统一按 [RUNNING.md](RUNNING.md) 启动。本文下方用于原AWS椅子场景和baseline回归，不与候选命令混用。

默认使用1m/.25rad相机组件、`nav2_depth.yaml`与GPU n/640；宽视场和导航保护需显式启用。原AWS椅子场景的命令与历史地图保留供回归；当前配置与实验结论见 [交接](HANDOFF.md) 和 [验证概览](DEVELOPMENT_VALIDATION.md)。早期蓝杯/v1、通信域及相机调整过程已移入 [历史场景记录](../archive/validation/AWS_HOUSE_20261117.md)。

本页下方的CPU、房间配置和计数记录仅对应原AWS椅子场景。当前五区、语义图层和任务输出共用底图，见[地图方案](DEPLOYMENT_PLAN.md#地图数据与输出视图)。

## 原已验收场景

平台保持 Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress。AWS Small House 资产包包含 68 种模型，本场景使用其中 63 种、共 87 个摆放实例，含厨房、客厅、卧室、杂物间及阳台，包含餐桌、冰箱、橱柜、沙发、床、书桌和 8 把椅子；厨房餐桌周围有 4 把椅子。模型真值只用于验收，不输入搜索算法。

## 场景选择与保留

启动脚本统一接受 `scene:=aws_house`（默认）或 `scene:=baseline`。仿真、导航、搜索必须选同一场景；切换前先 Ctrl+C 停止上一套节点。

| 内容 | AWS 家庭展示场景 | 基础测试场景（已验收） |
|---|---|---|
| 场景参数 | `aws_house` | `baseline` |
| Gazebo 世界 | `aws_house` | `course_house` |
| 地图 | `maps/aws_house/house.yaml` | `maps/baseline/course_house.yaml` |
| 房间与视点 | `rooms_aws_house.json` | `rooms_baseline.json` |
| 新任务输出 | `results/aws_house/tasks/` | `results/baseline/tasks/` |
| 验收材料 | `results/aws_house/validation/` | 原 `results/validation/`、`maps/course_house.*` |

房间配置位于 `src/semantic_search/config/`。基础场景另存档于 `scenes/baseline/maps/`，`preservation.json` 记录原始地图和配置校验值；旧路径及历史结果保留。基础场景重建地图时，保存脚本使用带时间的新文件名，保护原验收地图。

## 构建

在项目根目录运行。已有资产和视觉环境时无需重新下载。

```bash
cd "/home/willmatic/桌面/semantic_object_search"
bash scripts/fetch_aws_house.sh
bash scripts/build.sh
```

AWS 资产锁定到 `ros2` 分支提交 `ff9631ca6d1db9c1ba656498151464b5ab74aafe`；不编译其 Gazebo Classic 包。原始资产保留不变，运行时在临时目录生成 Fortress 世界和修复贴图路径后的网格。

## 用已保存的地图运行

每个终端先进入项目根目录。以下使用同一个场景参数；复现基础场景时，将各处 `aws_house` 一起改为 `baseline`。

终端 1：Gazebo 场景，保持运行。无窗口验证可加 `gui:=false`。

```bash
cd "/home/willmatic/桌面/semantic_object_search"
bash scripts/simulation.sh scene:=aws_house
```

终端 2：地图定位与导航，保持运行。

```bash
cd "/home/willmatic/桌面/semantic_object_search"
bash scripts/navigation.sh scene:=aws_house
```

终端 3：初始化仿真定位，脚本会等到 Nav2 导航器激活后再返回，避免第一条搜索命令被拒绝。脚本只适用于本项目地图与仿真坐标对齐的场景；也可在 RViz 中按实际位置使用 2D Pose Estimate。

```bash
cd "/home/willmatic/桌面/semantic_object_search"
source scripts/env.sh
python3 scripts/initialize_sim_amcl.py
```

终端 4：搜索与 CPU YOLO，保持运行。脚本直接使用 `.venv/bin/python`，无需手动激活虚拟环境。

```bash
cd "/home/willmatic/桌面/semantic_object_search"
bash scripts/search.sh scene:=aws_house
```

终端 5：RViz，保持运行。该入口处理先前 Snap 库引起的 `GLIBC_PRIVATE` 错误。

```bash
cd "/home/willmatic/桌面/semantic_object_search"
bash scripts/rviz.sh scene:=aws_house
```

回到终端 3，待 Nav2 和 YOLO 就绪后执行。第二个任务应在第一个结束后执行。

```bash
python3 scripts/run_search.py '去厨房找椅子' \
  --timeout 300 --output results/aws_house/task3_latest.json
python3 scripts/run_search.py '去厨房统计椅子' \
  --timeout 480 --output results/aws_house/task4_latest.json
```

任务目录输出 JSON、逐帧观测和带编号 SVG 地图，JSON 增加 `scene_id`。RViz 标记话题为 `/search/markers`。单次实测结果与局限以本页验收记录为准；原基础场景的通过结果不能替代新场景验收。

## 重建 AWS 地图

保持仿真运行，停止 `navigation.sh`、搜索和键盘控制节点。SLAM 与 AMCL 不能同时发布 `map → odom`。

终端 2 改为建图：

```bash
bash scripts/mapping.sh scene:=aws_house
```

另一终端启动建图期间的 Nav2：

```bash
bash scripts/mapping_navigation.sh scene:=aws_house
```

再开终端巡视并保存地图。使用真实激光构图，路线通过 Nav2 规划控制；巡游时不要同时发搜索或键盘速度命令。

```bash
source scripts/env.sh
python3 scripts/navigate_route.py --route scenes/aws_house/mapping_route.json \
  --output results/aws_house/mapping_route_latest.json
bash scripts/save_map.sh scene:=aws_house
```

保存后停止 SLAM 与建图期间的 Nav2，再按前文启动 `navigation.sh` 并初始化定位。修改家具碰撞布局后，需要重新建图和校正视点。

## 适配内容

- 用本工程已有 Fortress 物理、传感器、桥接和 TurtleBot3 底盘替换 Classic 启动链路。
- 保留家具位置、尺寸和纹理；修复相框照片错误的相对路径，去掉 Classic GUI layer 元数据。
- 静态家具移除不完整惯性；零厚度地板使用 Fortress 地平面提供碰撞，保留原木地板视觉。
- AWS RGB-D 安装高度为 0.65 m，SDF 与 TF 同步修改；基础场景仍为 0.30 m。
- 地图、房间配置、巡游路线、结果目录独立选择。来源、许可和修改范围见 `scenes/aws_house/source.json` 与 `docs/maintained/SOURCES.md`。

厨房首轮四视点试跑存在餐桌遮挡漏检。经过六视点试跑后，最终收敛为四个经真实 RGB-D 校准的观察点，分别提供更清晰的前、后排角度，让椅背、座面、椅腿进入画面。AWS 场景的 NMS IoU 调整为 0.30，抑制遮挡时的重叠检测；基础场景保留 0.50。曾尝试提高全局置信度到 0.50，但会漏掉其它有效目标，已恢复原来的 0.35。原始试跑、阈值试验和五视点结果保留为 `task4_initial.json`、`task4_threshold05.json`、`task4_side_only.json`；最终复验保存在 `final_run/`，不把“任务执行结束”等同于数量正确。

## 本机验收记录

2026-11-09 完成本轮部署和固定布局文本验收，机器可读汇总见 [summary.json](../../results/aws_house/validation/summary.json)。所有检查通过。

| 项目 | 本机实测 |
|---|---|
| 构建与回归 | 两个包构建成功，15 项测试通过；旧地图及房间配置 SHA-256 一致 |
| Gazebo / 数据 | 实际家庭场景渲染；雷达 360 点、5 Hz；RGB-D 640×480，深度 `32FC1`、米；TF 查询通过 |
| 新启动运动 | 位移约 0.2988 m，`final_run/sensors/sensors.json` 通过 |
| 地图 | 实际激光 SLAM，374×225，0.05 m；校准后的 30 个路点分段通过，非上游参考地图替代 |
| 任务三 | 首个视点找到 1 把椅子，25.282 仿真秒，TF 失败 0 |
| 任务四 | 四个校准视点，44 帧同步 / 44 个有效投影，正确计数 4，把编号 1–4 写入 JSON、SVG 和 RViz；73.198 仿真秒，TF 失败 0 |
| 定位对照 | 与四把椅子的模型原点作一对一 XY 对照，平均偏差约 0.109 m、最大约 0.238 m；这是可见几何估计与模型原点的比较，不是全场景精度保证 |
| 原场景回归 | 独立 ROS 域验证雷达、RGB-D、TF、运动通过；历史验收与原文件保留 |

结果与展示：

- [真实 Gazebo 家庭全景](../../results/aws_house/validation/final_run/house_overview.png)
- [任务三 JSON](../../results/aws_house/validation/final_run/task3.json)
- [任务四 JSON](../../results/aws_house/validation/final_run/task4.json)
- [编号地图 PNG](../../results/aws_house/validation/final_run/semantic_map.png)
- [任务四 SVG 与逐帧观测](../../results/aws_house/tasks/20261109T082709_166175Z)
- [实际 RViz 标记记录](../../results/aws_house/validation/final_run/markers.json)

旧验收汇总脚本已归档为 `archive/tools/scripts/summarize_aws_validation.py`；需要复核时按 [归档说明](../../archive/README.md) 在独立副本恢复原路径后使用。初期漏检、阈值试验、六视点试跑和启动抢先导致的拒绝记录保留，最终值使用 `final_run/` 的最终任务 JSON。

目前只验收固定布局的厨房椅子文本任务。其他房间的识别、杯子/瓶子、麦克风语音、GPU 推理、随机摆放和多次重复统计仍需后续专项验证。
