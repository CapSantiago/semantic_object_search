# R0 桌面目标候选验证（2026-11-16）

原 `baseline` / `aws_house` 的默认入口、地图、导航参数、权重与历史记录保留。这里记录独立候选的工程验证；唯一开发计划仍是 [DEPLOYMENT_PLAN.md](DEPLOYMENT_PLAN.md)，最新状态以 [HANDOFF.md](../HANDOFF.md) 为准。G0/G1/G2 尚未通过。

## 2026-11-17 增量验证：卧室 v3 与逐视点数据反馈

本节补充 11 月 16 日记录；下方旧候选和失败记录保留其时间边界。本批使用单白黄杯布局 `layout_r0_yellow_v1.json`、独立 `rooms_aws_house_v3.json`、域 66 / `IGN_PARTITION=semantic_20261117_bedroom`，原 NavFn/DWB、CPU 权重、640/.35/.30、1.0 m/.25 rad 相机与 `nav2_depth.yaml` 保持不变。v3 只把卧室杯/瓶的第二视点从 `[-8.25, 1.15, 2.4]` 改为 `[-8.3, 0.9, 2.4]`，与第一点 `[-8.3, 0.9, 2.1]` 共用 XY；默认开发搜索包装脚本仍指向 v1。

仿真/Nav2/检测器最初干净启动，从原点经自主导航进入卧室，未瞬移机器人。完成一次 v3 杯子计数预试后，为启用到达 TF 记录重启搜索节点，再连续执行以下 **15 次 warm 重复**；矩阵期间没有重启整套系统或重置机器人。这些结果不算 15 次冷启动，也不证明反复跨房间进入稳定。

| 卧室案例 | 重复结果 | 有效投影/次 | 仿真耗时/次 |
|---|---|---|---|
| cup find | 5/5 found，均确认 1 | 2 | 0.534–2.278 s |
| cup count | 5/5 completed，均确认 1 | 6–16 | 9.700–11.600 s |
| bottle count | 5/5 completed，均确认 1 | 10–15 | 11.338–11.620 s |

[完整任务与编号审计](evidence/bedroom_repeats_20261117.json)核对状态、数量、唯一 ID、持久化结果、逐帧命中次数和 SVG 编号，最初这 15 次均通过、TF 失败均为 0；两类 count 的两个视点每次均有有效投影。[独立导航与到达位姿复核](evidence/bedroom_v3_review_20261117.json)在这些任务时间窗内记录 25 个导航成功、0 个失败，未出现 spin/backup/wait 恢复、进度失败或代价地图清理请求。此结论只适用于该固定布局和批次，不能据此唯一确定旧恢复行为的根因或证明无碰撞。随后厨房/客厅 30 例批次在第 12 例被重复任务 ID 检查中止，前 11 例通过，后 18 例未运行。客户端误收上一条保留结果的证据见[失败记录](evidence/client_stale_result_20261117.json)；机器人另行完成的第 12 项任务不回写为客户端通过。修复后在新 `final_matrix/` 目录执行的 45 例已全部结束，结果为 44 例通过、1 例 partial，详见下方；原失败批次保留。

**请求视点、路径终点和实际停靠要分开读取。** 旧 `viewpoints` 及逐点 `pose` 是请求位姿；新增 `arrival_map_pose` 才是到达时最新可用 map→base_link TF，另存 `arrival_sim_seconds`、`arrival_transform_stamp_ns` 和 `arrival_pose_lookup: "latest"`。本批 25 次到达的 TF 年龄为 0–18 ms，相对请求 XY 的距离为 0.160726–0.161680 m。采到的 24 条实际发布路径终点均约为 `[-8.25, 0.9]`，比请求 `[-8.3, 0.9]` 东移 0.05 m；到达位姿距离这个共同路径终点为 0.113677–0.114697 m，落在原 0.12 m XY 容差内。其中 12 次到达有之前 2 秒内、朝向匹配的路径样本直接对应，较早的到达只能与共同终点比较。因此不能仅凭相对请求约 0.16 m 的距离断言控制器违反容差；也不能把 TF 新鲜度当作定位精度。

**v3 是同位置转向候选。** 配置朝向相差 0.3 rad，目标接受容差下，两点实际朝向只相差 0.104080–0.108291 rad，约 6°。这些结果支持该桌面布局的重复识别，不能当成空间覆盖或充分多视角的证据。find 在首点确认后提前结束，观察计划可为 false；所有结果仍为 `full_room_coverage_verified=false`。

逐视点数据反馈还进行了[实际断流回归](evidence/partial_observation_20261117.json)：第一点到达 2 个仿真秒后，仅对本批检测器发 `SIGSTOP`，仿真、深度和导航继续运行，随后恢复检测器。两个视点均成功到达且完成停留，但同步帧/投影分别为 `[4, 0]`；结果为 `partial/1`，并标出视点 1 的 `no_sensor_data`。`arrival_plan_completed=true`、`observation_plan_completed=true`、`observation_data_complete=false`，明确区分到达、停留和数据完整性；第一点有数据不会掩盖第二点断流。各字段的迁移及无数据/定位失败语义见 [INTERFACES.md](INTERFACES.md)。

初轮源码回归 **47 项通过**，本机日志 `results/development/20261117_bedroom/tests.log` 为 `47 passed in 4.57s`。新增覆盖逐点无帧、提前结束、TF 到达记录、矩阵进程退出/取消、旧结果拒绝和证据保留；这不替代 Gazebo、语音或 G0 验收。可用 `source scripts/env.sh` 后的 `PYTHONPATH="$PWD/src/semantic_search:$PYTHONPATH" python3 -m pytest -q tests` 复核，避免未加载 ROS 消息类型时跳过回调测试。

随后独立审阅补了两处边界：深度和内参同为 0×0 时拒绝计帧，正常首点加零尺寸次点返回 partial；未到达视点的取消状态不再被笼统标为 navigation_failed，使用明确的任务取消标记处理取消与成功回包竞态，保留原执行策略。该阶段 `tests/test_rooms.py` 与 `tests/test_search_observation.py` 共 **20 项相关离线测试通过**；上面的 47 项是修补前全套快照。最初 15 次卧室矩阵与 `[4,0]` 断流实测来自更早加载的版本，历史证据不回写。`final_matrix/` 重启搜索节点加载了这两处边界修复，实际参数为 `live_search_final.yaml`；每次任务记录所用源文件与配置散列。

客户端改为 VOLATILE 订阅，矩阵逐次传入既往任务 ID 以拒绝延迟旧结果。该阶段工程检查为 **51 passed、2 skipped**（`tests_final.log`）；跳过的两个独立 ROS 客户端用例已在域 69、随机专用测试话题[单独实测通过](evidence/client_freshness_20261117.json)，人工测试消息不进入机器人话题。两个 ROS 包重新构建成功（`build_final.log`），仅出现系统 pkg_resources 的既有版本格式警告。

### 45 例结果与补等版本的边界

[45 例任务与编号集合](evidence/tabletop_matrix_20261117.json)及[最终导航复核](evidence/final_navigation_20261117.json)记录同一套仿真中的 warm 批次：厨房 15/15、客厅 15/15、卧室 14/15，共 **44 例通过、1 例 partial**；集合整体状态保留为 failed。第 44 例 `044_bedroom_count_bottle_04` 的首点在 4 秒名义停留内没有合格同步帧，第二点有 4 帧并确认 1 瓶，因此返回 partial/1；其 TF/目标深度错误、导航失败均为 0。该点期间记录的底盘速度为 0，不能把无同步观测直接解释为导航失败、原始话题全停或 CPU 故障。

这一 45 例批次的 75 个导航目标全部成功，但厨房/客厅分别出现 1/4 次进度失败与对应局部代价地图清理，另有 5 次控制循环频率告警；卧室这 15 次未见上述恢复事件。这与前面最初 15 次卧室重复的时间窗分开统计。原始任务、缺口和失败均保留，未把 partial 改记 completed，也不因数量恰为 1 就把它当成完整观察通过。

| 搜索源码版本（SHA-256） | 实际验证范围 |
|---|---|
| `4a856d128edc3bfa58e702224ea92800ee9524e3a46c2939cc6cfa5740b123de` | 上述 `final_matrix/` 45 例，尚无有界补等，44 通过/1 partial |
| `4ae89f64a33538eb80b8fd811e0f369bddb3cbe6853f5e8b3beea19086128d1f` | 后续有界补等、受控恢复实验及额外 15 次卧室重复；不能追溯标记为旧 45 例所用版本 |

新版本只在某点名义停留结束仍为 0 有效同步帧时补等，默认额外上限 4 仿真秒，收到有效帧即可结束本点；上限后仍无数据则保留 partial/no_sensor_data。参数 0 可关闭补等，名义停留与补等参数均校验有限、非负，并冻结每点的时间上限；不提高全局最低帧数、不改变原权重/导航算法。逐点记录实际补等时间，截止后的迟到帧不能补救过期视点，取消仍可立即结束。接口细节见 [INTERFACES.md](INTERFACES.md)。

[受控故障及恢复证据](evidence/observation_recovery_20261117.json)已在新版本上完成以下真实 ROS/Gazebo 实测；故障输入有明确标记，原始传感器话题和导航继续运行：

| 条件与记录目录 | 实际结果 |
|---|---|
| `20261117_depth_invalid/invalid`：把真实深度复制到测试话题 `/validation/invalid_depth`，仅测试副本的 32FC1 样本改为 NaN，保留时间戳和尺寸 | localization_failed/0，3 帧、3 次目标深度失败、TF0；首点最初无帧，补等 1.830 s 后收到 1 帧无效深度，次点 2 帧无效深度、补等 0 s。等待原因是起初零帧；收到数据但无法定位仍明确报错 |
| `20261117_depth_invalid/restored`：搜索恢复原深度输入 | completed/1，9 帧/9 投影、TF0，两点补等均 0 s |
| `20261117_bounded_wait/transient`：只暂停本批检测器，首点到达后 6 仿真秒恢复 | completed/1，首点补等 3.650 s，两点帧/投影为 `[2,4]`，TF0 |
| `20261117_bounded_wait/permanent`：检测器持续暂停至任务终态 | no_sensor_data/0，两点额外实际等待 4.074/4.140 s，均 0 帧；没有无限等，任务后检测器已恢复 |

这些额外时长由真实结果记录；超过配置 4 秒的 0.074/0.140 秒来自上限之后的计时回调调度，不隐藏为整 4 秒。计时基于仿真时钟，不能据此声称 `/clock` 暂停时也有墙钟超时保证。故障实验验证的是错误处理和恢复，不证明检测准确率或全房覆盖。

[随后 20 秒只读传感器时序观测](evidence/sensor_timing_20261117.json)发现该新监听器接收的 RGB/检测较稀疏：剔除最初 5 秒后，检测的最大接收间隔约 2.894 秒，14 条检测中仅 8 条在捕获集合内有同时间戳深度和内参；这些离线匹配检测的最大到达间隔约 5.505 秒。该观测没有运行搜索同步器，不能代替第 44 例的同步回调证据或确定其根因。更早 `20261117_depth_invalid/original_sensors/sensors.json` 的短时健康快照因 RGB 仅 1 帧而 failed，亦原样保留；另一次沙箱无网络权限导致的全零监听已单独排除，不当成机器人故障。**CPU、DDS、相机/传输或调度的根因仍未确定；有界补等是对短时缺数据的容错措施，不是根因修复。**

补等版本的工程检查为 **58 passed、2 skipped**，日志 `results/development/20261117_bedroom/tests_bounded_wait.log`（4.68 s）；两项默认跳过的 ROS 客户端测试已有前述独立实测证据。[补等后的额外 15 次卧室定向重复](evidence/bedroom_bounded_wait_20261117.json)已全部通过，其中一次自然补等约 0.516 s；该批仍使用原 best_effort 输入，独立保存在 `results/development/20261117_bounded_wait/repeat_matrix/`，与最初 15 次及旧 45 次分开统计。旧 `baseline`/`aws_house` 场景回归脚本已准备、尚未执行；冷启动、原场景物理回归和后续 G0 工作仍保留，不以当前进展宣布验收完成。

### 可靠订阅候选链路

[同窗双 QoS 观测](evidence/qos_probe_20261117.json)先保持运行中的检测器与搜索不变，在同一个只读观察器内分别使用 BEST_EFFORT 和 RELIABLE 监听；两组只改变 reliability，均为 KEEP_LAST、depth 5、VOLATILE。20 秒观测剔除最初 5 秒后，收到消息数如下；图中发现的发布端均为 RELIABLE。

| 后 15 秒收到的消息 | BEST_EFFORT 监听 | RELIABLE 监听 |
|---|---:|---:|
| RGB | 5 | 120 |
| 深度 | 43 | 120 |
| 相机内参 | 119 | 119 |
| 检测 | 6 | 6 |

当时检测器的 RGB 输入仍使用原 BEST_EFFORT，两组监听器收到的是同一发布端的相同 6 条检测 header。切换下游观察器并未切换上游检测器输入；这组对照支持进一步验证输入可靠性，但不能单独证明稀疏检测或旧第 44 例的根因。两个 reader 共享发布端，RELIABLE reader 也可能影响确认、重传或背压，因此不把它当成完全隔离的交叉实验。

随后只重启 detector/search，分别显式设置只读启动参数 `sensor_reliability:=reliable`，形成新的候选链路；**默认仍为 best_effort**，包装脚本默认值、系统 DDS、模型/权重/阈值和 Nav2 未修改。共享 helper 保留其余原 sensor-data QoS 设置，参数非法值明确拒绝，运行时不能切换。[可靠输入时序与工程收据](evidence/reliable_sensor_timing_20261117.json)记录这次搜索源码 SHA-256 为 `4f87befe76146938be6bc6ecce5c0b2631552d083ff61a1b8ff18ce2cff9f4ca`，并核对权重、`nav2_depth.yaml`、yellow 布局及 v3 房间配置散列与前一补等批次相同。

新候选的 20 秒只读观测剔除最初 5 秒后，RGB/深度/内参/检测分别收到 **120/121/120/120** 条，检测约 8 Hz、最大接收间隔约 0.175 s。这 120 条检测的 header 均能在整个捕获集合内匹配同 stamp 的 RGB、深度和内参，即共同 header **120/120**。五次图端点快照均确认 `course_detector` 的 RGB 输入，以及 `semantic_search` 的检测、深度、内参三个实际输入端点为 **RELIABLE/VOLATILE**。RMW 对这些端点的 history/depth 报 UNKNOWN/0，未将其当成运行时队列深度证据；KEEP_LAST 5 来自代码及测试。离线 header 匹配不等于搜索同步器确实执行了 120 次回调，也不证明目标召回率或持续任务稳定性。

同批另用 `validate_sensors.py --sensor-reliability reliable` 完成 10 秒传感器检查并通过，RGB/深度/内参为 76/76/77 条，深度有效比例为 1，TF 检查通过。工程日志 `results/development/20261117_reliable/tests.log` 为 **63 passed、2 skipped**（5.01 s），包含实际 rclpy QoS 兼容性、默认等价及无全局 QoS 突变回归；两项跳过用例是前述已在隔离域 69 单独通过的 ROS 客户端测试，不计作本次执行。`build.log` 记录 `yolo_msgs` 与 `semantic_search` **2 包构建成功**（1.43 s），保留既有 pkg_resources 版本格式警告。短窗链路改善支持继续候选实测；底层传输/调度原因仍未确定，有界补等仍是容错措施。

新的 45 例保存在 `results/development/20261117_reliable/repeat_matrix/`。截至报告 `2026-11-17T05:40:57Z`，前 27 例通过，第 28 例 `living_count_bottle` 第 3 次因客户端 `No search command subscriber` 启动失败，`command_sent=false`，后 17 例未执行，报告为 **aborted**。检查时搜索进程仍在运行，不能据此认定搜索节点崩溃。客户端有界发现等待已修复并验证；按用户最新要求在此停止，另批客厅瓶子 count 5 次及卧室 15 次没有执行，核心搜索、检测器及 QoS 源码保持不变。即使补跑全部通过，也应分别报告原启动失败与合计 47 个成功任务，不能称单批 45 例通过；短窗链路收据本身也未包含任务矩阵验收。旧 45 例和两批卧室 15 例的源码、QoS 与结果均保持原记录，后续重跑必须另存。

### 2026-11-17 最新子任务完成及停止

客户端等待订阅者就绪的修复已完成：预备阶段后最多等待15墙钟秒，0可立即检查；任务超时从发布后开始，未发布前超时或被打断都不发送任务/取消。矩阵看门狗包含发现等待，超时参数必须有限。最终工程测试 **66 passed、5 skipped**；这5项在隔离域69另测 **5 passed**，涵盖延迟3秒出现订阅者、无订阅者、等待中SIGTERM、旧结果拒绝及真正任务超时取消。证据见 [client_discovery_fix_20261117.json](evidence/client_discovery_fix_20261117.json)。这些使用独立测试话题，不是新增机器人成功案例。

按用户要求停止。旧仿真/Nav2/搜索进程在当前进程核查中已不存在，退出过程未恢复，没有对旧PID发信号；下次不能继续标记为同一运行栈的warm批次。20例补跑、椅子兼容和原场景回归都未执行；所有历史失败不变。当前本地修改尚未提交或上传。

### 复现当前可靠订阅 v3 候选

以下命令对应**当前可选 reliable 候选源码**，不倒写此前 best_effort 批次的运行参数或结果。每个终端先进入项目根目录，统一使用一个空闲的隔离域/分区；下面的域 66 仅在其现有任务和节点退出后使用，否则另选空闲域。仿真显式指定**单杯** yellow 布局，搜索显式指定 v3，不能套用下文历史双杯布局的期望数量。

```bash
export SEMANTIC_ROS_DOMAIN_ID=66
export IGN_PARTITION=semantic_v3_reliable_replay
```

四个终端分别运行，仿真先启动，导航等待 `Localization and navigation active`：

```bash
# 终端 1
bash scripts/development.sh \
  layout_file:=scenes/aws_house/layout_r0_yellow_v1.json \
  evidence_dir:=results/development/v3_reliable_replay

# 终端 2
bash scripts/development_navigation.sh

# 终端 3
bash scripts/development_search.sh \
  -p rooms_file:="$PWD/src/semantic_search/config/rooms_aws_house_v3.json" \
  -p scene_id:=aws_house_v3_reliable_replay \
  -p output_dir:="$PWD/results/development/v3_reliable_replay/tasks" \
  -p sensor_reliability:=reliable

# 终端 4
bash scripts/development_detector.sh -p sensor_reliability:=reliable
```

在同域的客户端终端先检查当前可靠链路，确认传感器检查通过、检测已有真实消息，再完成一次普通预热任务并运行卧室 3 案例 × 5 次；`--startup-note` 按本次实际启动历史填写：

```bash
export SEMANTIC_ROS_DOMAIN_ID=66
export IGN_PARTITION=semantic_v3_reliable_replay
source scripts/env.sh
python3 scripts/validate_sensors.py --seconds 10 --sensor-reliability reliable \
  --output results/development/v3_reliable_replay/sensors
python3 scripts/run_search.py '去卧室统计杯子' --timeout 480 \
  --output results/development/v3_reliable_replay/preliminary_bedroom_cup.json
python3 scripts/run_search_matrix.py tests/scenarios/tabletop_repeat_matrix.json \
  --case bedroom_find_cup --case bedroom_count_cup --case bedroom_count_bottle \
  --output results/development/v3_reliable_replay/repeat_matrix \
  --first-start warm --subsequent-start warm \
  --startup-note 'After the preliminary bedroom task; no stack restart or robot reset during this matrix' \
  --config src/semantic_search/config/rooms_aws_house_v3.json \
  --config src/semantic_search/config/nav2_depth.yaml \
  --config scenes/aws_house/layout_r0_yellow_v1.json \
  --weights yolov8n-seg.pt
```

矩阵输出目录必须未存在；每次重跑换新目录，保留失败。省略三个 `--case` 可执行文件声明的三房间 9 案例 × 5 次；上述旧版本 45 例实际为 44 通过/1 partial，当前可靠订阅候选的重跑必须另存版本和结果。脚本保存每次任务前实际收到的 TF/odom、任务结果、耗时、诊断、配置/权重散列及总报告；超时、取消或客户端异常中止批次。启动状态和配置路径是操作者声明，脚本不重启系统、重置位姿或自动验证运行节点的参数。矩阵的期望数量只在结果返回后评估，布局真值不进入感知或搜索决策。

## 配置与已经核对的结果

Ubuntu 22.04 / Humble / Fortress，原 NavFn Dijkstra + DWB，原 CPU `yolov8n-seg.pt`，640 输入、置信度 0.35、NMS IoU 0.30。相机高 1.0 m、下俯 0.25 rad，增加外壳、支柱、安装板、碰撞和惯性近似，SDF/TF 同步。点云仅针对已验证的 Fortress X 前/Y 左/Z 上约定修正到 `camera_link`；深度进入独立 `nav2_depth.yaml`，原规划器/控制器及容差未改。

| 案例 | 首轮实际结果 | 仿真耗时 |
|---|---|---|
| 厨房杯子 find / count | 找到 1 / 计数 1，12 次计数投影 | 34.374 / 22.538 s |
| 客厅杯子 find / count | 找到 1 / 计数 1，10 次计数投影 | 25.082 / 28.402 s |
| 卧室杯子 find / count | 找到 1 / 计数 1，11 次计数投影 | 63.530 / 50.750 s |
| 厨房 / 客厅 / 卧室瓶子 count | 各计数 1 | 35.056 / 49.488 / 46.516 s |
| 厨房双杯，相距 0.45 m | 连续 5 次均计数 2，13–21 次投影/任务 | 首次跨房间 92.284 s，后四次 33.650–37.282 s |

以上 TF 失败均为 0。审计脚本逐例核对结果文件、实例数量、唯一 ID、逐帧关联的命中次数及 SVG 编号；双杯还核对了真实 `/search/markers` 的球体/文本 ID 与 JSON 一致。结果及配置散列可随仓库保存：

- [单实例完整结果集合](evidence/tabletop_single_20261116.json)：9 例首轮。
- [双杯与开阔卧室候选集合](evidence/tabletop_two_20261116.json)：5 次双杯及后续候选视点验证；每例包含检查项和原始结果。
- [双杯实际 RGB](evidence/two_cups_rgb.png)、[编号地图](evidence/two_cups_map.svg)、[实际 MarkerArray 核对](evidence/two_cups_markers.json)。
- [卧室实际 RGB](evidence/bedroom_yellow_rgb.png)、[杯子编号地图](evidence/bedroom_cup_map.svg)。图像来自真实 Nav2 运行中的同步帧，没有瞬移机器人作为任务验证；材质来源和许可见 [SOURCES.md](../SOURCES.md)。

与设计模型原点的 XY 对照仅是固定场景评估，不是独立定位精度标定，也不把模型真值输入搜索。`completed` 仅表示配置观察点执行完，所有结果仍明确 `full_room_coverage_verified=false`。除双杯案例外，不能把首轮通过写成每例重复 5 次通过。

## 已完成的异常与空目标检查

| 条件 | 实际反馈与证据 |
|---|---|
| 搜索配置指向不存在的相机 TF（明确的故障注入） | localization_failed；11 同步帧/11 次 TF 失败，未冒充 not_found |
| 仅停检测器，相机/深度避障正常 | no_sensor_data；同步检测帧 0；4 秒内 RGB 13、障碍云 5、检测 0 |
| 实际导航运动中取消 | canceled；取消一秒后的 53 条 odom 样本线/角速度均为 0 |
| 移除两只厨房杯子、正常感知 | find 和 count 均 not_found/0；分别 12 / 8 同步帧、TF0 |
| 一个观察点故意设在地图外，另一个有效 | 跳过失败点，返回 partial；仅确认 2 杯中的 1 个，不冒充总数正确 |
| 恢复正常双视点配置 | 再次 completed/2，18 投影、TF0 |

[故障与取消完整证据](evidence/failure_modes_20261116.json)、[空目标结果与编号审计](evidence/tabletop_empty_20261116.json)。这些测试明确标注配置故障和检测器停机，不篡改图像、深度或检测标签。原地图、权重等 10 文件散列仍一致，见 [基线保留检查](evidence/baseline_preservation_20261116.json)。

## 失败记录和边界

- 蓝色宽口杯在实际停靠帧常被识别为 bowl，塑料杯也漏检；较低相机、杯把朝向、紧停靠容差及 960 输入未解决。保留原权重/阈值/640，通过原 COCO cup 类中的典型白黄杯适配；没有把 bowl 改名为 cup。
- 首版支柱自遮挡已修复；白/黑材质尝试保留。Fortress 相对 OBJ/MTL/纹理缓存修复只改派生缓存，原资产和版权信息保留。
- 卧室旧后侧点在 scan-only 下可达、深度障碍开启后不可规划；桌侧 v1 点已完成任务，但重复进入有恢复动作。更开阔的 v2 点完成首轮杯/瓶计数，杯子仅 2 次有效投影，后续进入仍出现恢复；尚无证据证明 v2 更稳定，默认搜索包装脚本仍使用 v1。
- 域 64 的一次桥接进程未被 DDS 发现，根因未确认；清理旧节点后用域 65/独立 Ignition 分区完整重启恢复。失败批次 `r0_yellow_v1` 不计入视觉成功率。导航包装脚本已增加真实传感器门槛与进程组退出，实际退出检查无残留。
- 悬空箱的激光足迹回波 0、点云箱体点 2213、两代价层值 100，实际绕行成功一次，轨迹圆形外形最小余量约 0.0377 m；这是几何检查，不是接触传感器证明或动态障碍全面验收。

长日志、RGB-D 原始数组、逐帧观测与受控失败留在本机 `results/development/`，该目录按项目规则不纳入 Git。便携结果集合保留关键配置、原始任务结果、检查项、来源路径和 SHA-256；克隆后可重跑，而不是假定本地日志会随源码下载。

## 复现候选双杯布局

先按主 README 准备系统依赖、运行资源和 9 个目标资产，并构建。每个终端先进入项目根目录，统一设置：

```bash
cd "/home/willmatic/桌面/semantic_object_search"
export SEMANTIC_ROS_DOMAIN_ID=65
export IGN_PARTITION=semantic_tabletop_demo
```

四个终端分别运行，仿真应先启动：

```bash
# 终端 1；可加 gui:=true
bash scripts/development.sh \
  layout_file:=scenes/aws_house/layout_r0_yellow_two_v1.json \
  evidence_dir:=results/development/demo

# 终端 2；等到输出 Localization and navigation active
bash scripts/development_navigation.sh

# 终端 3；默认 v1 观察点，原场景不受影响
bash scripts/development_search.sh -p scene_id:=aws_house_r0_demo \
  -p output_dir:="$PWD/results/development/demo/tasks"

# 终端 4；无需手工激活虚拟环境
bash scripts/development_detector.sh
```

第五个同样设置环境的终端发送任务。前一个结束后再运行下一个：

```bash
source scripts/env.sh
python3 scripts/run_search.py '去厨房找杯子' --timeout 360 \
  --output results/development/demo/task3_cup_kitchen.json
python3 scripts/run_search.py '统计厨房的杯子' --timeout 360 \
  --output results/development/demo/task4_cup_kitchen.json
bash scripts/rviz.sh
```

JSON/SVG 为每次任务自动产物；PNG 可用 `scripts/plot_result.py --map scenes/aws_house/r0_v1/maps/house.yaml` 导出。该地图是原激光地图副本，不是重新 SLAM；家具未移动、桌面新目标均高于激光平面。不得同时启动同域的旧导航、SLAM 或键盘速度控制。

便携任务集合可以重新审计：

```bash
python3 scripts/summarize_tabletop_validation.py \
  --results results/development/r0_yellow_v2 \
  --layout scenes/aws_house/layout_r0_yellow_v1.json \
  --output results/development/r0_yellow_v2/audit.json
```

后续仍须补足代表性正式案例的重复/冷启动、遮挡和覆盖不足、基础场景物理回归、语音和人工演示；不能据上述首轮工程验证宣布 G0 或进入算法/模型比较。
