# 当前候选运行指南

适用于已经准备好依赖的本机：家庭场景、白黄杯布局、v4五区及观察点、实体相机支架和可靠传感器订阅。新机器先阅读 [部署说明](GITHUB_DEPLOYMENT.md)。无需手动激活视觉虚拟环境。

## 1. 日常一键启动

```bash
cd "/home/willmatic/桌面/semantic_object_search"
bash scripts/start_robot.sh
```

不需要激活虚拟环境，也不需要分别打开六个终端。脚本加载 ROS 和桌面运行库，依次启动 **Gazebo → 定位导航 → 视觉/搜索 → RViz**，同时打开本地网页 **http://127.0.0.1:8765**。等待页面显示“就绪”后再发送任务。

网页支持以下操作。

- 查看组件、雷达、RGB-D、定位状态，以及实际占据地图、五区轮廓/名称和机器人位置；可打开所选物品的观察点。
- 选择房间、物品和“寻找/统计”，或输入“去厨房找杯子”等文字任务。
- 点击“录音转写”并允许访问麦克风，说完可提前结束，最长 8 秒。检查填入的文字，再点击“开始任务”。语音资源用 `bash scripts/prepare_voice.sh` 准备。
- 查看前往哪个观察点、扫描进度和当前确认数量，随时停止任务。任务结束后可查看坐标、编号、JSON/SVG 和本次运行历史。
- 点击“进入建图”，可用方向键、网页按钮、文字或语音控制底盘。建图操作见 [下节](#3-建图保存与地图重载)。
- 在导航模式选择“清扫房间”和目标房间，或输入“请去健身区扫地”。也可以录音转写并确认。全房遍历会花数分钟，右侧显示覆盖、补扫和返航进度，地图显示实际轨迹。
- 点击传感器检查，采样10秒并生成报告；这只检查基础数据与TF，不替代四项课程功能验收。

每次启动创建独立的 `results/sessions/<时间_ID>/`，保存配置、组件日志、请求、任务产物和退出收据，不覆盖以前的结果。刷新网页会恢复当前状态；关闭网页不会停止机器人。**在启动终端按 `Ctrl+C` 退出整套系统。** 不要再同时执行下面的分终端启动命令。

清扫结束可下载报告、轨迹 JSON 和清扫地图 SVG。覆盖率以底图和机器人可达性估算的可清扫地面为分母，按实际定位轨迹累加。返回本次任务起点并停稳后，页面显示“模拟充电”。覆盖不足或返航未达标时，任务显示部分完成。

常用选项：

```bash
# 不打开Gazebo/RViz窗口，保留真实仿真和网页
bash scripts/start_robot.sh --headless

# 使用原CPU环境
bash scripts/start_robot.sh --device cpu

# 不自动打开浏览器，手动访问终端打印的地址
bash scripts/start_robot.sh --no-browser

# 原ROS域或网页端口已被其他程序占用时选择空闲值
bash scripts/start_robot.sh --domain 67 --port 8766
```

验证历史配置时可加 `--rooms-file <项目内JSON>` 或 `--layout-file <项目内JSON>`；实际选择与文件散列会写入会话记录，日常运行无需设置。当前v4包含周向观察，统计会执行完整路线，耗时通常比单物体寻找更长。

新增多物体测试布局可手动选择：

```bash
bash scripts/start_robot.sh \
  --layout-file scenes/aws_house/layout_coverage_v1.json \
  --rooms-file src/semantic_search/config/rooms_aws_house_coverage_v1.json
```

该布局配套独立补充观察点：首轮15项通过8项，七项遗漏补点复核通过5项；客厅、卧室的地面芥末瓶仍漏检，尚不适合用于全物体准确盘点。具体结果见 [验证概览](DEVELOPMENT_VALIDATION.md)。日常默认仍使用原布局。一键入口默认GPU推理，使用独立`.venv-gpu`；原CPU环境保留，可加`--device cpu`运行。GPU环境首次安装执行`bash scripts/prepare_gpu.sh`。

同一项目只允许一个一键实例。启动失败时网页保留原因和日志，退出后修复再运行。地图中的机器人图标只使用新鲜的 `map→base_link` 位姿，数据过期时页面会禁用任务提交。

## 2. 构建与分终端调试

首次部署或修改ROS包后构建：

```bash
cd "/home/willmatic/桌面/semantic_object_search"
bash scripts/build.sh
```

### 每个调试终端的公共配置

下面六个终端都先执行：

```bash
cd "/home/willmatic/桌面/semantic_object_search"
export SEMANTIC_ROS_DOMAIN_ID=66
export IGN_PARTITION=semantic_manual_demo
source scripts/env.sh
```

同域不要重复启动两套机器人节点。以下命令相对于项目根目录执行。

### 分终端依次启动

**终端1：Gazebo。** 等场景加载完成。

```bash
bash scripts/development.sh gui:=true \
  layout_file:=scenes/aws_house/layout_r0_yellow_v1.json \
  evidence_dir:=results/development/manual_demo
```

**终端2：导航。** 等待 `Localization and navigation active`。

```bash
bash scripts/development_navigation.sh
```

**终端3：视觉。**

```bash
SEMANTIC_DETECTOR_DEVICE=cuda:0 bash scripts/development_detector.sh -p sensor_reliability:=reliable
# 复现原CPU基线时，将上面的cuda:0改为cpu
```

**终端4：搜索。**

```bash
bash scripts/development_search.sh \
  -p rooms_file:="$PWD/src/semantic_search/config/rooms_aws_house_v4.json" \
  -p scene_id:=aws_house_manual_demo \
  -p output_dir:="$PWD/results/development/manual_demo/tasks" \
  -p sensor_reliability:=reliable -p reject_clipped_detections:=true \
  -p validate_object_size:=true
```

**终端5：RViz。** 包装脚本处理已有桌面运行库冲突。

```bash
bash scripts/rviz.sh scene:=aws_house
```

**终端6：传感器检查及任务输入。**

```bash
python3 scripts/validate_sensors.py --seconds 10 --sensor-reliability reliable \
  --output "results/development/manual_demo/sensors_$(date +%Y%m%d_%H%M%S)"
```

检查通过后，每次执行一条任务，等待结束再发下一条：

```bash
python3 scripts/run_search.py '去厨房找杯子' --timeout 480 \
  --output "results/development/manual_demo/find_cup_$(date +%Y%m%d_%H%M%S).json"

python3 scripts/run_search.py '去客厅统计瓶子' --timeout 480 \
  --output "results/development/manual_demo/count_bottle_$(date +%Y%m%d_%H%M%S).json"
```

完整任务产物在 `results/development/manual_demo/tasks/<任务目录>/`，包含 `result.json`、`observations.json`和收到地图时的 `semantic_map.svg`。

### 手动栈停止与其他工具

取消当前任务：

```bash
ros2 topic pub --once /search/command std_msgs/msg/String "{data: '取消'}"
```

退出时先取消任务，再依次停止搜索、检测、导航及RViz，最后关闭Gazebo，各终端按 `Ctrl+C`。

- 基础场景和原AWS场景：见 [AWS_HOUSE.md](AWS_HOUSE.md)。
- 矩阵验证：`python3 scripts/run_search_matrix.py --help`，每批使用新输出目录，准确记录启动条件。
- 原场景回归准备脚本：`bash scripts/validation/legacy_regression.sh`，必须在结束现有测试栈后显式执行；已有CPU原场景回归及退出清理证据，旧AWS偶发漏检记录保留，见验证概览。
- 一键入口固定当前候选配置：white/yellow单杯布局、v4五区视点、可靠传感器订阅、原导航与视觉参数；查本次实际参数请读会话的 `session.json`。

## 3. 建图、保存与地图重载

1. 按日常方式启动。在网页点击“进入建图”，等实时地图和运动保护就绪。
2. 用方向键或按钮移动，按空格或点击“停止”停车。也可输入或录音转写“前进”“后退”“左转”“右转”“加速”“减速”，核对文字后执行。每次行进或转向持续最多 1.5 秒。
3. 遇障时停车并显示距离。需要绕开时下达“绕行”，系统只在已探索且足够宽的区域尝试通过。可随时停止。
4. 停车后点击“保存地图”，下载当前任务里的 YAML 和 PGM 文件。点击“返回导航”恢复使用已校准的默认地图及五区配置。

使用新地图开展五区任务前，应核对初始定位、区域和观察点。保存的地图可用下方地图服务器步骤独立重载。

### 分终端调试

退出一键控制台后，从项目根目录给各终端加载同一环境。

```bash
export SEMANTIC_ROS_DOMAIN_ID=68
export IGN_PARTITION=semantic_keyboard_mapping
source scripts/env.sh
```

1. 启动仿真，等待雷达开始发布数据。

   ```bash
   bash scripts/development.sh gui:=true \
     layout_file:=scenes/aws_house/layout_r0_yellow_v1.json \
     evidence_dir:=results/keyboard_mapping/scene
   ```

2. 在另一终端启动 SLAM，再用第三个终端控制底盘。此时仅由键盘节点发布速度，保持 Nav2 和 AMCL 关闭。

   ```bash
   # 第二个终端
   bash scripts/mapping.sh scene:=aws_house
   # 第三个终端
   ros2 run teleop_twist_keyboard teleop_twist_keyboard \
     --ros-args -p speed:=0.2 -p turn:=0.7
   ```

3. 在空旷位置用 `i` 前进、`j` 和 `l` 转向，按 `k` 停车。停车后另开终端保存到新目录。

   ```bash
   bash scripts/save_map.sh scene:=aws_house map_output:="$PWD/results/keyboard_mapping/house"
   ```

4. 停止键盘和 SLAM，启动地图服务器。在另一个终端配置并激活它，通过 RViz 或 `ros2 topic info /map` 查看。

   ```bash
   # 地图服务器终端
   ros2 run nav2_map_server map_server --ros-args \
     -p use_sim_time:=true -p yaml_filename:="$PWD/results/keyboard_mapping/house.yaml"
   # 生命周期控制终端
   ros2 lifecycle set /map_server configure
   ros2 lifecycle set /map_server activate
   ```

保存脚本将 PGM 的空闲阈值设为 0.19，重载时保留未探索栅格。键盘短路线可用来演示建图、保存和重载。将新地图用于五区服务前，还需走遍所需区域并重新核对地图坐标、初始定位、房间边界和观察点。
