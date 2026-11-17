# 当前候选运行指南

适用于已经准备好依赖的本机：家庭场景、白黄杯布局、v4五区及观察点、实体相机支架和可靠传感器订阅。新机器先阅读 [部署说明](GITHUB_DEPLOYMENT.md)。无需手动激活视觉虚拟环境。

## 1. 日常一键启动

```bash
cd "/home/willmatic/桌面/semantic_object_search"
bash scripts/start_robot.sh
```

不需要激活虚拟环境，也不需要分别打开六个终端。脚本加载 ROS 和桌面运行库，依次启动 **Gazebo → 定位导航 → 视觉/搜索 → RViz**，同时打开本地网页 **http://127.0.0.1:8765**。等待页面显示“就绪”后再发送任务。

网页首版支持：

- 查看组件、雷达、RGB-D、定位状态，以及实际占据地图、五区轮廓/名称和机器人位置；可打开所选物品的观察点。
- 选择房间、物品和“寻找/统计”，或输入“去厨房找杯子”等文字任务。
- 停止当前任务，查看执行耗时、结果坐标、物品编号、JSON/SVG和本次运行历史。
- 点击传感器检查，采样10秒并生成报告；这只检查基础数据与TF，不替代四项课程功能验收。

每次启动创建独立的 `results/sessions/<时间_ID>/`，保存配置、组件日志、请求、任务产物和退出收据，不覆盖以前的结果。刷新网页会恢复当前状态；关闭网页不会停止机器人。**在启动终端按 `Ctrl+C` 退出整套系统。** 不要再同时执行下面的分终端启动命令。

常用选项：

```bash
# 不打开Gazebo/RViz窗口，保留真实仿真和网页
bash scripts/start_robot.sh --headless

# 不自动打开浏览器，手动访问终端打印的地址
bash scripts/start_robot.sh --no-browser

# 原ROS域或网页端口已被其他程序占用时选择空闲值
bash scripts/start_robot.sh --domain 67 --port 8766
```

同一项目只允许一个一键实例；更改域或端口不绕过此限制。启动失败时网页保留原因和日志，退出后修复再运行。地图中的机器人图标只使用新鲜的 `map→base_link` 位姿；数据变旧时页面会禁用任务提交。首版不提供建图运动、清扫或麦克风按钮，这些随总计划接入。

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
bash scripts/development_detector.sh -p sensor_reliability:=reliable
```

**终端4：搜索。**

```bash
bash scripts/development_search.sh \
  -p rooms_file:="$PWD/src/semantic_search/config/rooms_aws_house_v4.json" \
  -p scene_id:=aws_house_manual_demo \
  -p output_dir:="$PWD/results/development/manual_demo/tasks" \
  -p sensor_reliability:=reliable
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
- 原场景回归准备脚本：`bash scripts/validation/legacy_regression.sh`，必须在结束现有测试栈后显式执行；该回归尚未验收。
- 一键入口固定当前候选配置：white/yellow单杯布局、v4五区视点、可靠传感器订阅、原导航与视觉参数；查本次实际参数请读会话的 `session.json`。
