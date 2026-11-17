# Ubuntu 和后续 Codex 交接

## 当前可续接摘要（2026-11-17，增量周期 C001–C008）

先读本节，再按需读取下方历史命令与日志。唯一总计划仍为 [docs/DEPLOYMENT_PLAN.md](docs/DEPLOYMENT_PLAN.md)，不另开计划。这里是依据可见对话与当前文件保存的状态摘要；本轮没有调用或声称执行宿主原生上下文压缩，也未删除原对话或历史记录。

### C008：2026-11-17 里程碑总计划与UI需求确认

- **用户决定**：认可C007系统方案，要求总目标和框图置顶、从头到尾的里程碑各用一张表、最后一列“已完成/未完成”、表达简明；系统启动后应有UI展示状态、任务输入输出、执行情况与地图。
- **本轮完成**：重排同一 `docs/DEPLOYMENT_PLAN.md`，沿用P0–P6/P4a/P4b及G0/G1/G2。已完成的基础实现与待完成的正式验证分行标注。重排前原文保存在 `docs/history/2026-11-17-plan-before-milestone-reformat.md`，仅作历史快照，不是第二份执行计划；README/要求对照的链接同步更新。
- **UI设计**：首版采用本地浏览器界面，统一启动后自动打开；状态栏、任务输入、中央地图、执行过程、历史结果。P4先接寻找/统计和语音；P4a接运动/建图；P4b接清扫/返航/模拟充电；P6接语义记录查询。显示真实后端数据，重连不重复提交，停止有反馈，数据过期明确提示。框架/依赖留待实现时选择，本轮未安装或编写UI。
- **当前状态**：已保存计划和交接摘要，未执行宿主原生上下文压缩。无新实验、测试结果或运行功能，仍在P3收尾；实测数字和失败记录沿用C006。工作区未提交或推送。
- **下一步**：恢复开发后先完成C006剩余验证和覆盖收尾，再进入P4统一任务接口与UI首版；按原序接语音G0、建图G1、清扫及整机G2，之后优化和创新。当前可直接查看总计划第2节里程碑、第3节UI、第5节当前进度。

### C007：2026-11-17 四项必做功能与系统化开发复盘

- **最新目标**：用户明确任务一、二、三、四均必须完成，围绕真正家庭机器人组织共用功能，再开发语义地图及融合创新；描述应精准易懂，不过度强调任务划分。沿用唯一总计划，本轮仅只读审阅、上游调研和文档更新，没有启动新实验或新增运行代码。
- **统一设计**：在 `docs/DEPLOYMENT_PLAN.md` 第0节补充同一语音/文本入口、任务管理、环境数据、共用动作、运动控制与安全、展示记录。逐步封装现有搜索执行器，任务一/二接入同一系统；SLAM/AMCL切换、地图版本、请求ID、取消和控制权统一组织。新增模块属于设计，不能读作已经实现。
- **次序**：四项全做，保留P3/P4搜索/统计与语音G0→P4a建图运动G1→P4b清扫/返航与整机G2。共用系统入口随P3/P4建设，运动保护在任务一接入时完善。完整演示为带领建图→保存/命名→日常寻找/盘点/清扫→返回起点模拟充电→统一查看地图与结果。
- **拓展建议**：优先持久物品语义地图、主动观察/核验、清扫与观察协同；变化检测、暂停恢复和自然语言任务组合逐步加入。记录时间和可信度，跨任务实例关联不能仅靠距离。对照无记忆/有记忆/主动核验，以及独立执行/协同执行，以成功率、计数误差、路程、时间、地图更新质量衡量。本轮查阅Nav2、Open Navigation Coverage、ConceptGraphs和Goal-Oriented Semantic Exploration的官方资料，链接在总计划和SOURCES；尚未新增安装或选定新运行依赖。
- **事实更正**：直接核查 `search_node.py::finish()`，已打印“已找到，地图坐标为…”、“统计完成…”、“未找到指定物品。”，此前任务三/四口头对照将中文结果提示列为待开发不准确。尚需统一系统进度/拒绝反馈及语音播报。当前先发布结果再保存文件，写盘失败只日志记录；拟统一反馈执行与保存状态。
- **实测和工作区**：C006的66项工程+5项隔离ROS通过、旧45例44通过/1 partial、新候选27通过后发送前失败等事实不变，本轮无新的实测结果。代码和文档仍在本地工作区、未提交/推送；保存摘要不是执行宿主原生压缩。
- **下一步**：用户要求恢复实现时，按C006续接步骤重新启动验证链路和补跑，结合第0节统一接口设计推进。当前完成复盘与总计划更新，不自动恢复长时间实验。

### C006：2026-11-17 自主续接（按用户要求停在客户端就绪修复完成处）

- **目标/决策**：用户再次授权离开期间继续既定任务；沿用 P3、原 NavFn/DWB/权重/阈值，不需要新增用户选择。技能继续手工按 context-compression / grill-me / grilling 流程使用；8个技能文件哈希和发现链接复核一致。
- **实现**：`rooms.py` / `search_node.py` 增加逐视点诊断，到达、完整停留和数据完整性分别记录；局部缺帧应 partial。保存真实到达 map TF 与时间戳；`record_navigation_state.py` 被动记录 TF/odom/cmd_vel。独立审阅又补非正尺寸拒绝、取消与失败原因区分，相关20项测试通过；补等版本最终全量58项通过（此前51），另2个需显式启用的隔离ROS客户端用例在域69实测通过。两个包重新构建成功；收据 `docs/evidence/baseline_preservation_20261117.json`。
- **已验证**：新批次 `results/development/20261117_bedroom/`，域66 / `IGN_PARTITION=semantic_20261117_bedroom`，单白黄杯布局、1m/.25相机、CPU640；真实 sensors passed。独立 `rooms_aws_house_v3.json` / `spatial_v3.json` 静态通过，卧室采用同位置[-8.3,.9]朝向2.1/2.4，15次warm连续任务全部通过、TF0/深度错误0，25次导航成功且该时间窗无恢复。证据 `docs/evidence/bedroom_repeats_20261117.json` 和 `bedroom_v3_review_20261117.json`；不算5次独立进入或冷启动。
- **诊断/限制**：旧v1碰障与v2短程回退10秒未进展是不同症状；旧两张卧室截图都在转移中，不能证明到达姿态偏差。v3实际朝向差仅.104–.108rad，不等于广角覆盖。请求点到规划路径终点偏移.05m，实测停靠距路径终点约.114m，在原.12m控制容差内；到请求点约.161m应分别记录。旧场景和v1/v2都保留。
- **真实异常**：第二观察点暂停检测器，第一点4帧、第二点0，返回partial/1，缺口index1；已恢复检测器。证据 `docs/evidence/partial_observation_20261117.json`。厨房/客厅30例批次在第12例被矩阵鲜度检查中止：客户端收到上一条保留结果，前11例通过、12例不通过，后18例没运行；新机器人任务随后完成不回写客户端通过。原始 `other_rooms/` 和 `docs/evidence/client_stale_result_20261117.json` 保留。客户端已改VOLATILE并排除既往任务ID，2项独立ROS测试通过，证据 `docs/evidence/client_freshness_20261117.json`。`final_matrix/` 已跑完45例：44通过，卧室瓶子第4次首视点0同步消息、次视点4帧，正确partial/1，不能登记全45通过；便携审计 `docs/evidence/tabletop_matrix_20261117.json`。该失败窗口机器人停稳、导航成功，无逐输入时延记录，断流/同步缺口根因尚未定；已加仅对0有效帧视点的有界补等（默认最多4仿真秒），不改已有结果。受控短断流完成1、永久断流no_sensor_data0、NaN深度localization_failed0且恢复后完成1；详见 `docs/evidence/observation_recovery_20261117.json`。补等后卧室15例全部通过，其中一次自然补等.516秒，见 `bedroom_bounded_wait_20261117.json`。离线审计也增加跨案例task_id唯一性检查，能正确拒绝旧12例批次。
- **可靠订阅实测**：只读启动参数 `sensor_reliability` 默认保持 best_effort，候选 detector/search 显式 reliable。10秒传感器检查通过；20秒观测后15秒检测120/120与RGB/depth/info同stamp匹配，4个输入端点RELIABLE。底层传输/调度根因未定，不能宣称彻底解决。`20261117_reliable/repeat_matrix/` 实际为27个任务通过，第28次客户端发现失败、`command_sent=false`，后17例未运行，整体aborted；不是45/45通过。失败收据 `docs/evidence/client_discovery_failure_20261117.json`。
- **最新子任务已完成**：`scripts/run_search.py` 增加预备阶段后的有界订阅者发现等待，默认15墙钟秒、0可立即检查；任务超时从发布后开始。等待中退出/超时不发任务或取消；保存等待时间、匹配数及发布前最新位姿。矩阵转发该参数并扩展看门狗；所有超时参数拒绝NaN/Inf。最终66项工程测试通过、5项ROS测试默认跳过并在隔离域69单独全部通过；证据 `docs/evidence/client_discovery_fix_20261117.json`。这验证客户端协议，不代替机器人实测。之前两个包构建成功，本次只改客户端脚本和测试，没有再次构建包。
- **停止边界/工作区**：用户要求完成最新子任务后停下，已停止继续开发和实验，没有执行20例补跑、椅子兼容或原场景物理回归。当前主机进程核查未发现此前仿真/Nav2/搜索/检测器；没有向旧PID发信号，退出原因未恢复，不声称完成了主动正常关机。当前修改在本地工作区，尚未提交或上传GitHub；旧Git检查点不代表本轮代码。保存摘要不是宿主原生上下文压缩。
- **下次恢复顺序/标准**：用户要求恢复后，先重新启动隔离仿真并验证传感器/定位，显式yellow布局、v3视点、reliable输入；另目录补客厅瓶子5次和卧室15次，记录新批次为重新启动后的结果，不能沿用“同栈warm”说明。核对状态/逐点数据/数量/JSON/SVG及跨批task_id唯一性；即使新增20例全部通过，也只能报告历史27+新增20及保留的启动失败，不能写成连续45/45。再验证新相机下椅子及原baseline/aws_house物理回归（脚本 `results/development/legacy_20261117_procedure.sh` 已备未运行）。随后补代表性遮挡/邻房、独立冷启动、异常重复和定位标定，P4语音/人工验收完成G0后才进入任务一/G1、任务二/G2。当时的对照报告已随总计划旧版归档，现行状态见总计划第5节。


前一批 C005 的完整状态已移至 [历史快照](docs/history/2026-11-16-context-C005.md)，保留原9例首轮、双杯5次及边界实验的证据与失败决策。

C001–C005 的原始状态快照已归档，当前事实以本节为准，历史失败按需读取。

### 保留的目标与边界

- 唯一计划 [docs/DEPLOYMENT_PLAN.md](docs/DEPLOYMENT_PLAN.md)：P1 场景/房间 → P2 相机/覆盖 → P3 文本闭环 → P4 语音与人工验收/G0 → P4a 任务一/G1 → P4b 清扫返航/G2 → P5/P6 导航、视觉与长期语义地图探索。语音识别必做，大语言模型后置；任务一、二未实现完整闭环。
- Ubuntu 22.04 / ROS Humble / Fortress，原 NavFn Dijkstra + DWB、CPU yolov8n-seg.pt；默认阈值 .35，AWS NMS IoU .30，检测 640。R0 调用白名单 chair/cup/bottle，暂存 9 模型不等于开放 9 类。当前候选以人工多边形细化房间，未实现自动分割或全房覆盖。
- 原 baseline/aws_house、地图、配置和失败记录保留；新增独立入口/布局/输出。真值只作场景设计/评估，禁止填入感知或搜索结果。老师允许 TurtleBot3 来自既有交接/可见对话，未恢复原始确认材料。封闭柜内/始终遮挡物品不能保证可见。
- 历史椅子实测：AWS 厨房找 1/统计 4，TF0，JSON/SVG/RViz 一致，见 `results/aws_house/validation/summary.json`、`docs/VALIDATION.md`；原 baseline 统计 2。固定布局证据不等于全场景验收。新输出仍是 map/m，实例 ID 每次任务重置，不是跨任务语义记忆。
- 技能安装：项目 `skills/{context-compression,grill-me,grilling}/` 完整目录，`.agents/skills → ../skills`；固定上游提交与逐文件哈希见 `resources/project_skills.lock.json`，许可见 `skills/upstream-licenses/`。原生目录列出前者与 grilling；grill-me 为显式入口，无原生 Skill 工具，手工执行三个 SKILL.md。grilling 决策树没有当前必须由用户选择的分支；模拟充电的课程认可待任务二正式验收前核对。
- 当前可见历史不完整，不补写缺失对话或未取得的教师确认。C001–C003 原始状态见 [历史快照](docs/history/2026-11-16-context-C001-C003.md)，C004 桌面调试过程见 [开发快照](docs/history/2026-11-16-context-C004.md)，保留其时间边界，避免把旧“当前进程/未恢复 Git”读作最新事实。

> 2026-11-09 场景部署更新：默认入口已切换为 `aws_house`，真实激光地图、厨房任务三找 1 把椅子、任务四统计 4 把及编号地图均已通过；15 项回归测试通过。运行命令与证据见 [docs/AWS_HOUSE.md](docs/AWS_HOUSE.md)。本文下方原验收表针对 `baseline`；复现时为仿真、导航、搜索统一加 `scene:=baseline`，或使用新文档中的包装脚本。`maps/course_house.*`、原房间配置和历史结果保留，新场景输出独立目录。

## 任务和工作边界

2026-11-14 计划更新：任务一（语音控制建图）和任务二（指定房间清扫/返航）已融入原总计划。顺序为 **P1–P4 完成任务三、四/G0 → P4a 完成任务一/G1 → P4b 完成任务二及四模式集成/G2 → P5/P6 算法与模型探索**。任务一、二尚未实施或验收。P1–P3 提前统一房间入口、初始位姿、指令分派、运动控制权和记录接口；现有 `/search/command` 仍只支持原搜索契约，新增命令属于待开发项。下文历史验收追加中的旧阶段顺序以本段及总计划为准。

最新总约束：按 [docs/DEPLOYMENT_PLAN.md](docs/DEPLOYMENT_PLAN.md) 执行，**当前 Dijkstra+DWB / YOLO-seg 先完成 G0、G1、G2，之后才能开展导航算法比较和视觉模型探索**。相机组件、桌面物品、兼容类别、房间覆盖和必要避障输入属于主线功能补齐。可提前下载/安装资源；现已暂存 8 个目标模型、固定版本相机描述、独立 CPU 语音环境及 Whisper small 离线权重，见 [docs/RESOURCE_PREPARATION.md](docs/RESOURCE_PREPARATION.md)。新增资产没有插入现有场景，当前任务白名单仍为 3 类；目标扩展顺序见 [docs/TARGET_CATALOG.md](docs/TARGET_CATALOG.md)。语音识别本身是任务书必做，放在核心执行功能稳定后的 P4、G0 之前，之后复用于任务一、二；大语言模型是可选后期探索。

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
