# 当前项目交接

## 当前可续接摘要（2026-11-19）

**唯一总计划为 [DEPLOYMENT_PLAN.md](DEPLOYMENT_PLAN.md)。P1–P3 当前基线通过，P4/G0 和 P4a/G1 自动验证通过。P4b 已接入清扫，返航停靠短程实测通过，完整覆盖仍待验收。** 已按用户要求完成本项后暂停，工时预算记在总计划。真人麦克风试用由用户择时进行。

### 当前目标与已确定方案

- 完成同一家庭机器人系统的语音建图、清扫、寻物、盘点；开发顺序P3→P4/G0→P4a/G1→P4b/G2→优化与创新。
- Ubuntu22.04/Humble/Fortress，NavFn Dijkstra+DWB，YOLOv8n-seg（默认GPU、保留CPU基线）、640输入、置信度.35、AWS IoU .30。不改原基线推进主要功能。
- 首批目标chair/cup/bottle；房间采用人工多边形、类别固定视点。场景真值只供设计和评估。
- 网页由 `bash scripts/start_robot.sh` 统一启动，已支持寻物、盘点和建图。用户已授权在既定范围内自主实施与本地验证。
- 语义地图、主动核验、清扫与观察协同是已认可的发展方向；模拟充电的课程表达待正式验收前核对。

### C018 新电脑部署验证（2026-11-19）

- 新增 `scripts/deploy.sh`，用系统 Python 还原资源、安装 CPU 依赖并构建。按默认布局自动还原白黄杯和芥末瓶，固定与 colcon 兼容的 `setuptools==78.1.0`。每个检出目录拥有独立的构建路径和 ASCII 别名。
- 临时干净副本在线恢复资源并构建成功。CPU 模式网页就绪，实际传感器检查和厨房找 1 杯通过，重新准备后的离线语音可转写合成音频。工程测试 167 项通过，临时副本、缓存及日志已清理。用户确认发布到现有公开仓库。[部署说明](GITHUB_DEPLOYMENT.md)

### C017 清扫后返航与停靠通过（2026-11-19）

- `scripts/run_cleaning.py` 使用实际记录的起点和已标定的出房间路径返航，为最终对位预留时间。靠近起点且原目标失败时，可分开尝试位置接近与朝向恢复。仍须同时满足 0.15 m、0.15 rad 并停稳至少 0.45 秒。单元测试核对朝向超差时保持返航未完成。
- 首次测试在记录起点后未更新定位新鲜度计时，导致首个导航目标被过早终止。修复为每次发送目标前用已确认的新鲜位姿刷新计时，冷启动后续测通过。
- Gazebo 健身区短程试验将任务时限设为 135 秒，实际清扫后经出口返回初始位姿，耗时 109.82 秒，位置误差 0.116 m、朝向误差 0.097 rad、停稳 0.46 秒。覆盖率 13.7%，任务正确保存为部分完成，Firefox 显示模拟充电及清扫地图。工程测试 167 项通过，7 项独立 ROS 用例按开关跳过。两次测试实例已关闭，测试端口释放，域 67 无节点。[阶段收据](../evidence/p4b_docking_20261119.json)
- 暂停点。恢复后优先按完整预算试验的遗漏图诊断不可达区、条带与补扫，再做同标准覆盖验证和四功能连续演示。基本任务与报告预算 12–19 小时，详见总计划。真人麦克风和课程对模拟充电的确认仍待安排。

### C016 P4b/G2 清扫与返航首轮（2026-11-18）

- 网页新增“清扫房间”，接受按房间选择和确认后的语音指令。执行入口为 `scripts/run_cleaning.py`，占据栅格、往返条带和覆盖计算位于 `src/semantic_search/semantic_search/cleaning.py`。沿用现有 Nav2、入口坐标及五区地图。
- 覆盖口径为机器人中心可通行并可由 0.50 m 假定扫幅触达的已知地面。按实际地图位姿计数，目标 85%。返程使用本次记录的起点和朝向，0.15 m / 0.15 rad 内停稳才显示“模拟充电”。覆盖不足或停靠不达标时返回部分完成并保存结果。
- 首轮健身区完成 42.4% 覆盖，返航停在距起点 2.13 m 处，已保存 JSON、实际轨迹和 SVG。实时 Nav2 代价地图显示部分静态空闲位置受障碍影响。改为按连续条带分组、使用实时代价图拆分受阻条带，并缩短清扫目标的恢复时间；入口和返航仍使用原导航行为。
- 一次复测因 Nav2 初始化脚本漏收 DDS 状态回执而中断，已增加有时限的查询重试。其后同场景完整预算复测覆盖 34.9%，耗时 261.14 秒，停在距起点 0.426 m 处，朝向误差 2.679 rad。条带分组方案仍需优化，覆盖未达到 85%。
- Firefox 以合成普通话“请打扫卧室”启动实际清扫，途中取消后的 51 个里程计样本线、角速度均为零，随后厨房寻物找到 1 杯。原始数据集中于 `results/development/p4b_g2/`。真人麦克风与课程对模拟充电的认可待正式验收。

### C015 P4a/G1 自动验收完成（2026-11-18）

- 网页建图依次退出 Nav2 和搜索、启动 SLAM 和 `mapping_motion`。核心文件为 `scripts/robot_console.py`、`scripts/robot_runtime.py`、`scripts/robot_monitor.py`、`scripts/run_mapping_motion.py`、`src/semantic_search/semantic_search/mapping_motion.py`、同目录 `mapping_control.py` 与 `ui/`。
- 键盘、网页按钮和确认后的语音文字共用运动保护。单次短动最多 1.5 秒，行进限速 0.06–0.18 m/s。合成普通话经 Firefox 转写并驱动前进约 0.180 m。前后移动、转向、加减速、取消、长按超时及忙碌时拒绝切换通过。
- 前后障碍距约 0.35 m 时立刻停止。补扫后绕过小箱净移约 1 m，绕行中取消后观察到连续零速度。宽屏障及初次未探索通道拒绝绕行。绕行须下达新指令，由已探索栅格与当前激光计算路线。
- 新图保存 YAML/PGM，独立重载 83,402 格逐格一致，保留 51,801 个未知格。恢复原五区导航后，厨房找杯、盘点各得到 1 个，JSON/SVG 已保存，原地图散列一致。新图用于五区任务前须核对定位与房间标定。
- 切回导航的首轮验证将正常的 6 个速度发布端点误判为冲突，修正为核验发布节点。另有初次未就绪 409 与重载测试的 DDS 回执超时，已续测。工程 158 项和独立 ROS 7 项通过，测试实例已关闭，66/67/69 域无节点。原始数据集中在 `results/development/p4a_g1/`。[P4a 阶段收据](../evidence/p4a_g1_20261118.json)
- 下一步沿用既有地图与 Nav2，先为选定房间生成可清扫栅格及往返路线，再实测覆盖、遗漏和补扫。随后以实际起点验证返航、停靠和 UI 模拟充电，最后综合演示。真人试用可单独安排。

### C014 P4/G0 自动验收完成（2026-11-18）

- 搜索节点在 `/search/progress` 发布实际导航、观察、等帧、取消及保存阶段。网页以 `request_id` 对应任务、进度和停止。共用任务记录覆盖搜索、转写和传感器检查。修改涉及 `search_node.py`、`semantic_map.py`、`run_search.py`、`robot_monitor.py`、`robot_console.py` 和 `ui/`。
- 浏览器录音由 `.venv-voice` 的离线 Whisper 转写，显示可编辑文字并等待确认。合成普通话经 Firefox MediaRecorder 实测找到1杯、盘点4椅、杂物间未找到、取消、静音拒绝。取消两秒后的202个里程计样本速度为零。真人麦克风尚待试用。精选 [P4验收](../evidence/p4_g0_20261118.json)，原始数据集中于 `results/development/p4_g0/`。
- 首次盘点因首视点多计1把椅，第一候选又有一次漏左前排，保留 `voice_ui_initial.json` 和 `kitchen_initial_candidate_audit.json`。第二候选调整首点并加入两个前排补看点，厨房椅子路线由14增至16点。同配置在两个独立冷启动会话及后续热运行中完成五次盘点，共5/5数量、定位、到点和产物审计通过。最大物品XY .16775m、到点XY .13826m。当前默认 v4 已提升，旧快照 `rooms_before_calibration.json` 与 `profile_selection.json` 保留。其余房间和类别路线按配置逐一比对一致。
- 前排探测工具曾退出143，发送者未知。在核验原导航目标完成、客户端已退出后另采剩余两个点。复测脚本的环境加载问题和缺失参数均在发布任务前发生，已修正并用完整批次重跑，未计作通过。完整失败路径见阶段收据。
- 键盘建图通过，实际位移 .524m 并生成激光占据地图。修正 `scripts/save_map.sh` 的 trinary PGM 空闲阈值为 .19，并允许显式输出新路径。重载82,806格与 SLAM 原图一致，未知格保留，原地图文件散列一致。YAML 原点舍入误差最大约4.6mm。短路线用于链路验证。
- 工程154项通过，7项 ROS 客户端测试另在域69通过，提升默认后9项配置测试通过。各自动测试会话 clean，当时确认67/68/69域无残留。

### 真人麦克风验收（用户延期）

- 原待测会话 `results/sessions/20261118T061411_635301Z_406acd34` 已关闭，未收到真人试用结果。恢复试用时重新启动当前版本，核对真人语音转写和实际执行产物。C014 合成语音的证据继续保存在原收据。

### C013：GPU及P1–P3基线验收完成（2026-11-18）

**目标与授权**：用户要求GPU验证后继续完成P1/P2/P3，已撤销C012结束时的停止要求；常规实施和验证无需重复确认。完成本阶段后报告结果，后续开发顺序仍为P4/G0→P4a/G1→P4b/G2。

**已验证事实**：

- 独立`.venv-gpu`：torch2.14.1+cu130/torchvision0.29.1+cu130、CUDA13.0、Ultralytics8.4.207、numpy1.26.4；RTX4060 Laptop8GB、驱动580.178.04。原CPU环境保留，未改系统或驱动。`scripts/prepare_gpu.sh`负责项目内安装。
- `robot_console.py`/`robot_runtime.py`一键默认`cuda:0`，支持`--device cpu`；会话记录设备/环境/配置散列。旧分终端视觉入口仍默认CPU，可设`SEMANTIC_DETECTOR_DEVICE=cuda:0`。`detector.py`显式FP32，原权重、640/.35/.30不变。
- 同6张RGB-D图像各60帧：同环境CPU/GPU中位16.95/4.41ms，3.84倍；检测数一致，框/掩膜IoU最低.999786/.999010。实际GPU找杯1/数椅4通过，检测约7.99Hz、RGB-D8Hz、scan5Hz，进程显存约268MiB。不是整机加速比。见 [GPU收据](../evidence/gpu_inference_20261118.json)，原始`results/development/gpu_20261118/`。
- GPU遮挡12/12、异常恢复45/45及57项JSON/SVG/观测次数审计通过，夹具和参数恢复；证据`results/development/p123_gpu/{occlusion,fault_recovery,fault_artifact_audit.json}`。工程152项通过、7项ROS客户端本轮未重复（C012已有独立7项通过）。
- 正式案例未改动的15组×5次严格审计75/75：`p123_gpu/unchanged75_{report,audit}.json`。最大物品XY.12745m、到点XY.13283m/朝向.12463rad。按完整案例组保留全部5次，逐路线核实旧/新配置一致，不能称单会话连续100次。
- 原布局缺失24扫视点均Nav2到达并抓帧，最大XY.11973m/朝向.10004rad；`remaining_baseline_navigation.json`和`baseline_captures/manifest.json`组合61条记录。`baseline_visibility.json/png`按类别路线保留未观察采样点，几何可见不等于识别完整。
- 新布局`layout_coverage_v1.json`有9杯/7瓶/10椅/1碗/1书。首轮15项严格通过8项，`placement_probe/aws_house/`、`placement_probe/audit.json`；七项遗漏保留。书/碗仍未开放，现有碗图像未检出。
- 校准观察点后，新布局健身椅计数5/5严格通过，`gym_refined/{report,audit}.json`。七项补点诊断`supplement_count/audit.json`严格5/7：客厅杯3/椅1、健身瓶1、杂物间杯1/瓶1恢复，客厅/卧室各一只地面瓶仍漏检。单帧偶尔检出不等于完整任务可靠识别。

**修改、依据与未采用方案**：

- 默认v4已在正式审计通过后提升；提升收据`p123_gpu/promotion.json`。实测候选`p123_gpu/rooms_refined.json`仅将卧室杯/瓶首对视点x=-8.3改为-8.1，并给健身椅增加正面[0,2.6,2.3]、[-.1,2.9,2.4]替换桌后点；旧快照`rooms_v4_before_refinement.json`保留。独立新增布局配置为`src/semantic_search/config/rooms_aws_house_coverage_v1.json`；实测快照`p123_gpu/rooms_coverage_tested.json`，结束后仅修正说明字段，不改路线。
- `scripts/validation/audit_search_results.py`增加到点TF/动作结果审计；冻结XY≤.20m、yaw≤.15rad，控制器原.12m/.10rad不改。`tests/test_acceptance_audit.py`覆盖缺TF/超差；五区测试允许实测校准卧室位置，仍要求双朝向与完整扫视。
- 默认导航仍NavFn Dijkstra+DWB、`nav2_depth.yaml`。PoseProgressChecker候选仅8朝向对照均通过（原37.70s/候选37.93s，无收益证明），完整候选路线未验证；文件归`p123_gpu/nav2_observation_candidate.yaml`，不启用。
- 新布局杂物间驶出曾172秒进展失败，`bottle_chair_navigation.json`；实际连接方向[-3.4,-2]→[-2,-2]可退出，`bottle_chair_retry_navigation.json`。不据一次成功声称所有路线修复。靠墙间距.20m的校准候选已撤出，原4点实测和取消记录保留。
- 客厅地面芥末瓶多角度被误认fire hydrant，卧室地面瓶单帧检出而任务漏检；不改标签、不降阈值、不用真值补结果。`supplement_navigation.json`、`floor_angles*_navigation.json`、`*_frames.json`保存校准过程。

**失败与进程管理**：

- 原正式B50项严格46/50：卧室瓶首次两点到达约.213m；健身椅两次计数1/预期2，另一次数量正确但位置偏差约.85m。`formal_b/audit.json`保留失败。
- 原正式A前5次通过，第6次因服务退出超时900秒；`formal_a/`保留原报告，`formal_a_completed5.json`只收完整前缀。重启A完成35个未改动案例后由本代理按身份SIGTERM停止，余下旧卧室重复由候选新批替换；`formal_a_retry/superseded.json`、各`shutdown.json`可追溯。
- 早期外部终止来源仍未知；最初strace默认忽略跟踪器信号的空日志不能证明未发生。`motion_check/simple/adoption.json`记录核验PID/UID/启动时间后接管、完成34点并清理。后续独立进程会话保留正常信号处理及所属清理，不能称根因已解决。

**最终结果、文件与证据**：

- 域67独立冷启动`p123_gpu/refined_baseline/aws_house/`的五组×5次全部严格通过；与未改动75次合并的`final100_audit.json`覆盖20组×5、100个唯一任务ID。最大物品XY.19152m（卧室瓶）、到点XY.13283m/朝向.12463rad，最大任务墙钟303.75s。冻结标准未放宽。
- 精选 [阶段验收收据](../evidence/p123_gpu_acceptance_20261118.json) 包含100次来源、57项异常、原场景回归、覆盖、全部失败路径、配置散列及清理。不是连续单配置100次，早期失败不删除。
- `baseline_final_captures/`有65条同布局实测引用，`expanded_final_captures/`75条；两套配置所有观察点已采齐。`baseline_final_visibility.json`、`expanded_visibility.json`保留各高度未观察采样点；[基线可见性图](../evidence/p123_gpu_coverage_20261118.png)可直接查看，不宣称全房全部物体可见。
- 最终工程152通过/7跳过，`p123_gpu/final_tests.{json,log}`；首轮漏设scripts模块路径的收集失败另存`final_tests_missing_pythonpath.*`，修正调用后通过，没有对应产品缺陷。7项ROS专用用例已有C012独立验证。
- 所属正式批次退出均clean；新增布局会话`results/sessions/20261117T175718_684264Z_79372d02/shutdown.json`无残留。`p123_gpu/final_domain_checks.json`确认66/67/68无节点，监督脚本、控制台和只读采集器均已退出。没有待接管的运行任务。
- 总计划P1/P2/P3已标完成；README、RUNNING、DEVELOPMENT_VALIDATION、REQUIREMENTS_STATUS、TARGET_CATALOG同步。新视点已通过工程测试，不需为文档改动重复机器人实验。

**限制与下一步**：

1. 当前已完成的是固定家庭场景的传感器、地图、文本寻物/计数核心及覆盖诊断；新增布局的两处地面瓶漏检保留，不称任意位置完整识别，书/碗不开放。完整课程任务一至四仍未全部验收。
2. 下一开发阶段沿唯一总计划进入P4/G0：先在现有搜索协议/UI接真实导航/观察阶段反馈，再补共用任务接口与真实语音输入，验证语音寻物/统计、停止和失败反馈。随后P4a建图运动控制，P4b清扫/返航；本批未启动这些新功能。
3. 默认CUDA FP32，原CPU环境和旧配置/失败快照保留；日常`bash scripts/start_robot.sh`，回退加`--device cpu`。当前无需要用户重新确认的常规决定；后续模拟充电课程表达须在正式验收前核对。
4. 工作区修改未提交、未推送；保留已有C012变更。重要决策继续更新本摘要，实验具体过程集中在本批证据，不另建重复计划。

保存本摘要不等同于执行宿主原生上下文压缩；未恢复的信号发送者等历史保持未知。

### C012历史：P1–P3验收与多位置探测（当时已按要求停止，C013已恢复）

**当时目标与停止要求（历史）**：完成P1–P3剩余实现及验收，追加多物体/多位置覆盖检查。用户11月18日要求本批结束后停下；现在所有所属测试实例已关闭，不自动补测、不切换GPU、不继续P4。唯一总计划仍是DEPLOYMENT_PLAN；P1完成，P2/P3尚未完成。

**已验证事实与证据**：精选收据 [p123_acceptance_20261118.json](../evidence/p123_acceptance_20261118.json)，原始证据集中在 `results/development/p123_acceptance/`。

- 两个ROS包构建通过；工程149项通过，7项ROS用例在完整套件按开关跳过、随后独立域69实测7项通过（`client_discovery_fix_retry/`）。149项结果来自实际工具输出，7项另有逐例收据。最后仅调整离线覆盖图排版，无运行核心变更。
- P1五区连接/起点21次到点通过，最大XY0.1192m、朝向0.0986rad（`connections_navigation.json`）；scan约5Hz、RGB/depth约8Hz。18视点独立导航/RGB-D通过，最大XY0.1382m、朝向0.1106rad。冻结后续到点审计0.20m/0.15rad，控制器原0.12m/0.1rad不改。
- 原baseline找1/计数2通过；旧AWS首次计数3/预期4失败，独立复核找1/数4通过（`legacy_aws_retry/aws_house/`），偶发漏检不抹去。v3剩余20例与产物/定位审计通过（`v3_remaining20/`），与旧27次成功分开统计。
- 杂物间边缘球误识别为chair：调整观察点并过滤截断检测后，五次计数1通过（`utility_confirmation_retry/`）。客厅垃圾桶误识别为cup：可见尺寸检查排除它，离线六个真实杯瓶样本保留，后续客厅杯计数正确（`size_filter_calibration.json`、`expanded_remaining/`）。厨房椅子截断漏检经三处前/后排视点校准，五次计数4通过（`formal_kitchen_chair_v2/`）。
- 当前正式100例**累计33例通过严格数量/定位/ID/JSON/SVG/观测/路线审计**（`completed33_audit.json`），分为5+25+3，非连续通过。原95例批次中断，只抽取前25例；最新70例批次完成3例、第4例取消、66例未执行，退出143，需补67个有效案例。`formal_remaining70_retry/`保存中断及清理收据；原95例缺完整退出收据，不补写。
- 受控遮挡12例通过：两种视点基准、五次正面遮挡0/换侧面恢复1，屏障移除、参数恢复（`occlusion_retry/`）。异常/恢复45例通过：邻房过滤、不可达、TF、检测器中断、取消停车、隔离无效深度及恢复，各重复5次（`fault_recovery/`）。57项产物核对通过（`fault_artifact_audit.json`），隔离深度子节点和转发线程均已停止。
- 新增独立布局 `scenes/aws_house/layout_coverage_v1.json`：20个放置目标加原8把椅子，共28个（9杯、7瓶、10椅、1碗、1书）。15项计划仅完成厨房杯2/瓶1两项并通过定位/产物审计，椅子任务计数4但被取消，不能算通过，后12项未执行（`placement_coverage/`）。15张实测RGB-D离线检查中，厨房侧视画面可见的碗未检出；书及其他房间新增位置未验证，书/碗未开放任务调用。
- 基线覆盖已采37/61条清单记录，缺24个扫视记录；可见性报告明确缺失位姿，见`visibility_coverage_available.json`及 [覆盖图](../evidence/p123_coverage_20261118.png)。几何可见不等于识别充分，不能把未采集区域当作已验证覆盖。新增布局帧未混入基线覆盖。
- 浏览器桌面/真实390px只读检查通过（`ui/report.json`）；未在停止要求后另起UI任务测试。11月18日GPU检查为RTX4060 Laptop 8GB、驱动580.178.04；视觉环境torch2.14.1+cpu，当前推理没有CUDA加速，未切换配置。

**关键修改与决策**：

- v4各区增加8方向固定扫视，保留类别桌面首点；厨房补[5,-1.5,.8]、[6.5,-1.4,pi/2]、[8.9,2,-2.9]，杂物间两点改为[-6,-2.6,-2.35]、[-6.5,-3.1,-2.75]。`ui/app.js`合并同XY多朝向显示。模型、阈值和导航算法不变。
- `depth.py`/`search_node.py`增加可选截断过滤和分割深度尺寸检查，一键入口启用，旧入口默认关闭。杯XY包围盒对角线≤0.25m/高≤0.30m，瓶≤0.35m/0.60m；只作用于掩膜路径，是可见表面先验，不是真值或精确尺寸。修正整数观察点传入ROS浮点坐标问题。
- `run_search.py`发现判据要求同一节点既订阅command又发布result，避免被只读跟踪器误判为服务就绪。7项真实ROS客户端测试通过；初版Humble不支持API造成的失败在`client_discovery_fix/`保留。
- `robot_runtime.py`/`robot_console.py`支持启动时选择项目内rooms/layout并记录散列；`development.py`复用项目内已有家具资产目录。`navigate_route.py`及`validation/{audit_search_results,capture_observation_views,visibility_coverage,occlusion_recovery,fault_recovery}.py`增加事后审计和可恢复夹具。真值始终只用于设计/评估。

**阻碍、失败与清理**：

- 最新两个长批次均收到终止信号，驱动退出143；取消由退出清理送达客户端，**最初信号发送者未知**。不能归因于模型、用户或已修复的发现竞态。首次70例启动参数发现失败、早期外部SIGINT/取消、厨房计数3、球/垃圾桶误检和采集JSON错误均保留原路径。
- 主会话`results/sessions/20261117T154823_480004Z_bbbd81d1`已按核验PID/UID/启动时间退出；域68正式及域67新增布局均有`shutdown.json`确认clean。随后只读检查66/67/68域均无节点，未结束其他用户进程。
- 当前没有运行中工作或需要重复确认的常规决策。保存本摘要不等于执行宿主原生上下文压缩。

**当时待续步骤（现已由C013接续）**：先查明长批次终止原因；按精选收据逐案例补正式67次有效结果与24条基线扫视采集。再完成新增布局剩余13项有效任务及漏检定位，必要时调整独立候选视点。完整产物/覆盖/退出验证后再将P2/P3标完成，随后沿用P4/G0→P4a/G1→P4b/G2。当前不启动这些工作。

### C011：五区配置与标注图

- 用户确认五区并授权配置坐标、观察点、UI、绘图skill图片置于总计划前。当前一键入口/解析/UI已切至 `rooms_aws_house_v4.json`，原四区v3未改；中部以y=1.0拆为下方客厅/上方健身区，其余覆盖范围不变，无新增墙体。
- 新增 `scenes/aws_house/spatial_v4.json`、`tests/test_five_zones.py`，UI从后端配置读取五区轮廓、名称及可选观察点。139测试通过、5项隔离ROS测试跳过；静态坐标检查通过，健身区3个视点Nav2实走通过；浏览器12项通过，证据在 `results/development/five_zones/`。
- 会话 `results/sessions/20261117T105354_590939Z_9f841a29` 已停止且shutdown无残留；健身区找椅子在第2视点途中随会话停止而canceled，不能记为寻物成功，停止信号来源未确定。此前已走通3点的独立导航记录不受影响。尚未完成五区完整感知/覆盖验收。
- 内置imagegen已生成 `docs/assets/house_five_zones.png` 并插入总计划开头，精确坐标图为 `house_five_zones_coordinates.png`。用户指出左侧蓝紫边界偏离挡板：实际模型RoomWall_01_001的y=-1.496180，程序边界y=-1.5（差约4mm），生成图标注偏移已修正，程序坐标保留。
- 修正版图片已置于总计划前，旧图片草稿仅留本地results；RUNNING/接口已更新。验证摘要并入现有 `docs/evidence/oneclick_ui_20261117.json` 的 `five_zone_update`，不新建重复报告。
- 用户要求后续语言准确、简洁、清晰，减少无用运行记录/重复文档，已加入AGENTS。GitHub主体更新已发布并核验：`ca462b8`，文件树与本地提交一致；本地发布前提交保留在备份分支。日常继续使用main。

### 已实现与已验证

- 原场景和AWS场景的传感器、SLAM、定位导航已实测；原baseline计数2椅、AWS厨房找1/计数4，结果/标记一致。
- 独立候选：实体相机支架、高1m/下俯.25rad、深度障碍输入；三房间桌面杯瓶和双杯5次重复已有通过记录。
- 搜索节点已有文本/JSON、取消、中文结果、map米制坐标、去重编号、JSON/SVG/RViz输出；逐视点到达/停留/数据诊断及有界补等已实现。
- 当前候选房间配置 `src/semantic_search/config/rooms_aws_house_v4.json`；布局 `scenes/aws_house/layout_r0_yellow_v1.json`；导航 `src/semantic_search/config/nav2_depth.yaml`；detector/search显式 `sensor_reliability:=reliable`。独立包装脚本默认仍是旧v1/BE；新一键入口已固定v4/reliable，分终端复现须显式指定，见 [RUNNING.md](RUNNING.md)。
- 旧45例为44通过/1 partial；补等后卧室15例通过。可靠输入新批27通过，第28次客户端发现失败且没有发任务，后17例未执行。客户端发现等待已修复：默认15墙钟秒，超时/中断前不发布任务；矩阵看门狗同步延长。
- C006工程66项及隔离ROS客户端5项通过。C009整理后的检查收据见 [目录整理核验](../evidence/reorganization_20261117.json)，不会把离线检查计成新机器人实验。

### 关键限制与失败决策

- 蓝杯被识别为bowl，降低相机/紧容差/960输入未解决；保留640及原阈值，改用白黄杯独立布局。失败数据已归档。
- 卧室v1/v2短程移动/恢复问题保留；v3同位置双朝向重复较稳定，实测朝向差仅约6°，不代表全房覆盖。`full_room_coverage_verified=false`。
- 旧客户端误收保留结果已改VOLATILE并排除旧任务ID；逐点缺帧用partial反馈和有界补等。RELIABLE短窗数据改善，但底层传输/调度根因未定。
- 当前仍是单搜索任务执行，跨任务永久物品ID、完整统一调度/运动仲裁、清扫覆盖和语音人工联调待开发。
- 完整历史、具体参数、早期源文件散列及失败经过见 [归档交接](../archive/history/HANDOFF_before_cleanup_20261117.md) 和 [验证概览](DEVELOPMENT_VALIDATION.md)。历史未恢复部分不补写。

### 本次结构整理

- 长期文档统一在 `docs/maintained/`；原始教材在 `docs/course_materials/`；历史在 `docs/archive/`；精选 `docs/evidence/`保持原路径及内容。
- 根HANDOFF/SOURCES只保留短入口，根README负责入门；[docs/README.md](../README.md)是阅读索引。
- 6个旧专项脚本及配套C++源码移入 `archive/tools/`；已结束的开发批次和旧日志压缩在 `archive/runs/`，逐文件校验后再清理原目录。保留最新可靠输入批次、用户manual_demo及已跟踪的原场景验收数据。
- 只删除可再生成的旧构建/Python/pytest缓存；运行源码、install、外置构建、依赖、资产、地图、权重和全部未提交修改保留。迁移/恢复索引见 [reorganization_20261117.json](../../resources/maintenance/reorganization_20261117.json)。
- C009为结构维护；C010已按新授权恢复启动/UI集成实验。相关代码和文档已随C011发布。保存摘要不等同于执行宿主原生上下文压缩。

### C010：一键启动与控制台

- 新增 `scripts/start_robot.sh`、`robot_runtime.py`、`robot_monitor.py`、`robot_console.py` 及 `ui/{index.html,app.js,style.css}`；入口无需激活venv，默认打开网页、Gazebo和RViz，绑定127.0.0.1:8765，默认域66，每次独立IGN分区与 `results/sessions/<时间_ID>/`。
- 实际数据驱动就绪判定、map/TF显示、文字/选择寻物与统计、单任务互斥/幂等、取消、结果链接与本次历史已接入。静态地图保留原点朝向，过期位姿不显示为实时定位；不使用场景真值控制搜索。
- 两次GUI冷启动通过；第一会话传感器检查通过（scan约5Hz、RGB-D约8Hz）、厨房找杯子和计数均1且JSON/SVG已保存；第二会话取消后恢复寻找成功。浏览器宽屏/390px、无JS错误、刷新不重发任务已有证据。
- 修复启动后写盘失败遗留子进程、取消等待过长、关闭时未等待任务记录，以及Popen后进程身份暂不可读导致取消无效的竞态。进程清理只处理本会话token/UID/启动标识核验后的进程，浏览器不带此标记。
- 实际浏览器开始/停止按钮也已验证；取消2秒后的193个odom样本速度为0。两次运行均正常清理；结束时最终会话无所属残留进程、8765端口已关闭，普通浏览器可留在断线页面。
- 工程测试134通过、5项ROS客户端测试本轮跳过；没有重复运行P3正式矩阵。本轮证据和原始会话路径统一见 [oneclick_ui_20261117.json](../evidence/oneclick_ui_20261117.json)。原10项基线散列保持不变。
- 首版仍是现有搜索协议的适配器：无跨功能ROS Action/运动仲裁、实时导航步骤、麦克风、建图操作、清扫或跨运行语义历史。请求ID与核心task_id分别保存；不要再从外部终端向同域并发发任务。

俯视截图：`results/screenshots/house_top_20261117/house_top.png`，来自当前白黄杯家庭场景的Gazebo实际窗口；仅调整临时GUI相机，未修改正式场景或房间配置。

本次计划复核仅修改既有文档：总计划纠正三房间旧表述，区分已完成的搜索UI接入与待完成的共用任务协议，明确v3/v4证据和五区验证顺序；要求对照、验证概览同步修正。未新增实验或验收结论，无新增待确认决策。

### 下一步执行顺序与完成标准

按上方当前摘要推进 P4b/G2 房间清扫与返航。真人麦克风试用由用户择时进行。

技能位于 `skills/`，发现链接 `.agents/skills`保留。沿用context-compression维护本摘要、grilling检查关键未决选择；目前无需重复访谈。
