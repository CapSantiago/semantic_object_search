# 任务输入与结果接口

2026-11-17 更新。语言/语音模块后置；先用文本和结构化任务验证执行层。当前不调用大语言模型，`voice.py` 中的可选 Whisper 适配器尚未验收。

## 输入

话题 `/search/command`，消息类型 `std_msgs/msg/String`，`data` 支持两种格式：

```text
去厨房找椅子
去厨房统计椅子
取消
```

```json
{"mode":"count","room":"kitchen","category":"chair"}
```

结构化格式只接受 `mode`、`room`、`category`。`mode` 为 `find` / `count` / `cancel`；`find` 和 `count` 必须指定当前场景的房间 ID，以及 `chair` / `cup` / `bottle` 中的类别。取消格式为 `{"mode":"cancel"}`。房间 ID 从所选场景的 `rooms_*.json` 读取。非法模式、房间、类别、字段类型或多余字段会被拒绝。

可用现有客户端测试接口：

```bash
source scripts/env.sh
python3 scripts/run_search.py '{"mode":"count","room":"kitchen","category":"chair"}' \
  --timeout 480 --output results/aws_house/task_json_latest.json
```

客户端以 **VOLATILE** QoS 订阅结果，只接收之后发布的消息，不请求 transient-local 保存的历史结果。连续发送相同命令时，还可重复传入 `--exclude-task-id <已完成任务ID>`，明确忽略延迟到达的已知旧结果并继续等待新 ID。`scripts/run_search_matrix.py` 自动传入本批已收到的任务 ID，并在报告层再次拒绝重复 ID。诊断文件记录 `excluded_task_ids`、`ignored_task_ids` 和 `pre_command_task_ids`；客户端可用 `--diagnostics-output <路径>` 保存它们。文本和 JSON 命令字符串仍原样发布，不追加请求字段，不改变核心输入协议；当前协议仍要求串行执行任务，不能据此保证任意并发发布者之间的请求归属。

重复运行使用 `scripts/run_search_matrix.py`，`--case` 筛选案例、`--limit` 限制执行次数；输出目录必须新建。`--first-start`、`--subsequent-start` 与 `--startup-note` 由操作者显式声明冷热启动情况，脚本不负责重启。每次保存实际结果、启动前 TF/odom、文件哈希与诊断，预期数量只在收到结果后评估。超时或取消停止批次；任务 ID 新旧检查只验证记录归属，不能作为感知准确率、数量正确或物理停车的测试。

客户端独立 ROS 回归使用域69和随机的 `/matrix_client_test_<id>/command`、`/matrix_client_test_<id>/result`，测试人工发布缓存旧 ID、延迟旧 ID、新 ID 和取消回包。它不启动仿真或搜索节点，也不向真实 `/search/*` 话题发布测试结果。两个案例及源文件散列见 [客户端结果归属证据](../evidence/client_freshness_20261117.json)。可在单独子 shell 中复现，避免改变正在运行场景的终端环境：

```bash
(
  export SEMANTIC_ROS_DOMAIN_ID=69
  source scripts/env.sh
  SEMANTIC_RUN_ROS_CLIENT_TESTS=1 python3 -m unittest discover \
    -s tests -p test_search_client_ros.py -v
)
```

解析支持杯子和瓶子不等于这些类别已实测通过；“可乐”目前只能映射为 `bottle`，没有品牌/饮料种类识别。关键词文本解析和结构化 JSON 最终进入同一执行链路。

当前每次执行一个任务，支持取消，没有任务队列。不合法输入、忙碌或导航未就绪会在 ROS 日志中拒绝；当前并非完整的 ROS Action 任务接口，没有逐步反馈或每次拒绝的结构化应答。这些状态接口在完善执行功能时继续补齐。未来语音转写、规则解析或语言模型适配器只负责生成上述任务，导航和避障由执行层负责。

## 传感器输入可靠性

`course_detector` 和 `semantic_search` 各自提供只读启动参数 `sensor_reliability`，只接受 `best_effort`（默认，原行为）或 `reliable`。前者只作用于检测器的 RGB 输入；后者作用于搜索的检测、深度和相机内参三个同步输入。需要分别在两个节点启动时设置，例如给 `development_detector.sh`、`development_search.sh` 各加 `-p sensor_reliability:=reliable`；运行中 `ros2 param set` 会因只读描述符拒绝，修改后必须重启对应节点，非法字符串在启动时明确报错。

共享 helper 为每次选择构造独立 QoS 实例，仅改变 reliability；`KEEP_LAST`、depth 5、`VOLATILE` 及其余 sensor-data 策略保持原值，不修改全局 `qos_profile_sensor_data`。检测输出、任务/地图话题、点云适配器、默认包装脚本和系统 DDS 配置均沿用原设置。`reliable` 输入要求发布者提供 RELIABLE；它与 BEST_EFFORT 发布者不兼容，不能把该选项当成对任意输入的通用保证。实际 RMW 兼容性和全局预设隔离由 `tests/test_sensor_qos.py` 在不启动节点的条件下检查，运行中的接收率仍需另存实测证据。

`scripts/validate_sensors.py` 同样支持 `--sensor-reliability best_effort|reliable`（默认 best_effort），使用同一 helper 配置其六个传感器/时钟输入，并把选择写入 `sensors.json` 的 `sensor_reliability`。验证显式 reliable 候选时使用 `--sensor-reliability reliable`，以免把不同监听策略的健康快照混称为同一链路结果。既有 45 例、补等重跑及故障记录保留各自加载版本和 QoS，不能追溯改标为新选项的验证。

## 输出与坐标

- `/search/result`：`std_msgs/msg/String`，`data` 为结果 JSON。
- `/search/markers`：`visualization_msgs/msg/MarkerArray`，包含地图坐标系中的物体球和编号文本。
- `results/<场景>/tasks/<UTC时间戳>/result.json`：任务、状态、实例列表、坐标、命中次数、视点及传感器统计。
- 同目录 `observations.json`：逐帧投影、关联 ID 和已确认 ID。
- 收到 `/map` 时，同目录自动生成 `semantic_map.svg`，叠加本次目标类别的物体编号。
- PNG 是通过 `scripts/plot_result.py` 额外导出的预览，不是每次任务默认生成的文件。

新运行结果显式加入 `frame_id: "map"`、`coordinate_unit: "m"`。原历史结果不回写，其坐标语义同样来自代码中的 `map` 变换。

`x/y/z` 是 **map 坐标系中的米制三维位置估计**，由图像像素、深度、相机内参和采样时刻的 TF 计算。XY 用于二维地图标注，Z 表示高度分量；估计受可见表面、遮挡和定位误差影响，不是抓取点或已知模型中心。

地图原点由建图/定位确定，不是图片左上角；保存地图 YAML 的 `origin` 是栅格图像在 map 中的位姿。当前仿真与地图做了对齐，不能对其他地图假定 `map == Gazebo world`。图像像素、机器人 `base_link`、相机光学坐标和 `map` 是不同坐标系。

地图底图 `maps/<场景>/*.yaml` 和 `*.pgm` 独立于任务保存，不会每次搜索重新建图。物体 ID 当前每个任务从头分配；没有跨任务永久 ID，也没有持续更新的对象数据库。

`completed` 表示全部配置观察点已到达、完成停留、均有可用观测且已确认实例；相同条件下没有确认实例为 `not_found`。`count` 是本次已确认的数量，不能据此保证房间所有物品均被看见。新增字段和状态如下（历史 JSON 不回写）：

- `viewpoints` 只列实际成功到达的点，`viewpoint_failures` 记录失败索引、位姿与 Nav2 状态码。单点导航失败后继续尝试其余视点；有失败视点的计数任务返回 `partial`，不冒充完整结果；find 确认对象后仍可提前返回 `found`。
- `viewpoint_diagnostics` 按配置顺序列出所有视点，包括未到达点。每项包含 `index`、`pose`、`reached`、`navigation_status`、`dwell_completed`，以及该点独立的 `synchronized_messages`、`rejected_sensor_frames`、`synchronized_frames`、`tf_failures`、`invalid_target_depth`、`valid_projections`。消息只在停留阶段计入，排除到达前和重复时间戳；内参、尺寸或编码不合格的同步消息记入 rejected，不能当成有效帧。深度与内参尺寸相等也必须宽高均大于 0，空图不能支持 not_found。旧的任务级统计字段继续保留。
- `observation_extra_seconds` 是搜索节点参数，默认 4.0 仿真秒；它和名义 `dwell_seconds` 在启动及运行中设置时均要求有限且非负，extra 设为 0.0 可恢复不补等的行为。仅当某点完成名义停留后仍为 0 有效同步帧时，才继续等至名义时长加此上限；收到有效帧后在下一个计时回调结束该点，不提高最低帧数。有帧但目标深度/TF 失败不触发这项补等，仍按原定位失败规则报告。每次到达冻结本点时长，因此运行中改参数只影响后续视点；长期断流达到上限后仍返回 partial/no_sensor_data，find 确认与取消仍可提前结束。
- 逐点 `nominal_dwell_seconds`、`extra_wait_limit_seconds` 保存本点使用的时长，`extra_wait_started` 记录是否触发补等，`waiting_for_sensor_data` 表示当前仍在补等，`extra_wait_seconds` 记录实际经过的额外时间。计时器周期为 0.2 仿真秒，到上限后的首个回调退出，实际时长可含回调调度延迟；截止后的迟到帧不再补救过期视点。结果生成时 waiting 字段清为 false，取消时也保留已经经过的补等时间。`dwell_completed` 仍指名义停留已完成，数据完整性单独判断。旧批次结果及其验证结论不回写；节点重启后才加载此行为。
- `viewpoints` 与逐点 `pose` 仍是请求的导航目标，不是实际停靠坐标。成功到达的诊断新增 `arrival_sim_seconds`；`arrival_map_pose` 保存当时查询的 map→base_link 平移 `xyz` 和四元数 `xyzw`，`arrival_pose_lookup: "latest"` 明确使用 TF 缓冲区最新可用变换，其真实时间保存在 `arrival_transform_stamp_ns`。不能把该变换时间假定为到达时刻；查询失败时位姿/变换时间为 null、`arrival_pose_error` 给出错误，不更改导航成功状态。
- `coverage.arrival_plan_completed` 表示成功到达全部配置视点。`observation_plan_completed` 从 11 月 17 日起要求全部视点还完成了配置的停留时长；旧版本仅检查到达，消费方若需要旧语义应改读 `arrival_plan_completed`。因此在最后一个视点提前 found 或取消时，到达可为 true，观测计划仍为 false。
- `coverage.observation_data_complete` 单独表示全部视点均收到可用的同步帧，且不存在“有 TF/目标深度失败、但该点没有任何有效投影”的整点定位故障。空检测帧可以证明本次数据链路有输入，不要求一定检测到目标；此字段不保证检测召回率、时间采样充分性或每个目标均成功投影。偶发失败保留在逐点计数中。
- `coverage.viewpoints_observed` 是到达、停留和上述数据条件均满足的视点数；`incomplete_viewpoints` 给出缺口索引和原因列表：`not_visited`、`navigation_failed`、`canceled`、`dwell_incomplete`、`no_sensor_data` 或 `localization_failed`。未确认到达时，Nav2 状态 5 或明确的逐点 `task_canceled=true` 使用 canceled；后者也涵盖取消与导航成功回包同时发生的情况。仅状态 6 使用 navigation_failed，其余保守记录 not_visited；这项诊断修补不改变原有导航执行策略。`full_room_coverage_verified` 仍为 false，`basis` 仍为 configured_viewpoints_only，固定视点执行结束不是全房覆盖证明。
- 一个视点有帧不能掩盖另一个视点无数据或整点定位故障：部分视点有正常观测、其他视点有缺口时返回 `partial`，保留已确认对象。所有已到达视点均无有效帧时为 `no_sensor_data`；全部视点导航失败而未到达任何点时为 `navigation_failed`。全任务无有效投影且存在 TF/目标深度失败时仍为 `localization_failed`。`navigation_rejected`、`canceled` 保留独立含义；find 确认对象后仍可提前返回 `found`，其 coverage 同时报告未完成的计划。
- `elapsed_seconds` 保留仿真时钟耗时；新增 `elapsed_wall_seconds` 为本机单调时钟耗时。两者从接收任务到形成结果计时，不含启动和文件写盘耗时。运行中的旧节点需重启才能输出新字段。


## 接口验证边界

11 月 9 日结构化接口阶段共 18 项回归通过；11 月 16 日加入几何、传感器组件、点云布局和部分观察状态后共 30 项回归通过。11 月 17 日逐视点覆盖修复后，共 40 项源码回归通过，其中 5 项直接调用搜索回调而不启动 ROS 节点；需先 `source scripts/env.sh` 加载消息类型，否则这 5 项显式跳过。覆盖无帧、整点定位失败、空检测、提前结束、停留完成、实际到达位姿与结果序列化。物理候选场景证据单独保存，未把单元测试记作语音或完整覆盖验收。历史 Gazebo / 任务三四证据见 `AWS_HOUSE.md`。

## 客户端就绪等待（2026-11-17）

`run_search.py --discovery-timeout 15` 在位姿/至少2秒预备阶段后，独立等待命令订阅者，单位墙钟秒；0只做即时检查。`--timeout` 从任务发布后开始。未发现返回码2、`outcome=discovery_timeout`；等待中SIGINT/SIGTERM返回130；两者均 `command_sent=false`，不发送取消。诊断保存发现等待耗时及发布前匹配数，初始TF/odom在等待后取最新可用值，仍不保证位姿新鲜度。所有超时必须有限；任务超时正数，其余等待非负。矩阵同名参数原样转发，加入子进程看门狗预算。独立测试收据见 [客户端就绪修复](../evidence/client_discovery_fix_20261117.json)。


## 本地网页入口（首版）

`bash scripts/start_robot.sh` 启动 `robot_console.py`，仅监听 `127.0.0.1:8765`；静态界面位于 `ui/`，无需Node/npm安装。网页通过同源HTTP访问，后端通过现有搜索客户端接入ROS；没有更换原 `/search/command` 协议。

| 接口 | 用途 |
|---|---|
| `GET /api/state` | 本次会话、组件进程、实际话题新鲜度、定位/导航就绪、当前任务及最近30条历史 |
| `GET /api/map` | 实际 `/map` 栅格、分辨率、原点/朝向及坐标系 |
| `POST /api/task` | `request_id`加`mode/room/category`，或加`command`文字；串行执行find/count |
| `POST /api/cancel` | 空JSON对象，取消当前网页发起的任务 |
| `POST /api/validate` | 空JSON对象，真实传感器/TF采样，完成后返回报告 |
| `GET /api/log?component=search` | simulation/navigation/detector/search/rviz/task日志尾部 |
| `GET /api/artifact?id=…` | 使用状态响应返回的结果链接，只能读取本次会话登记的文件 |

`request_id`用于本次控制台会话内的重复提交保护：相同ID和任务返回原状态，相同ID和不同任务返回409。它不等于搜索节点生成的 `task_id`。串行客户端排除已有任务ID，避免历史结果串入；仍不要从其他终端向同域搜索节点并发发指令。

控制台状态分开表示启动阶段、组件是否存活、ROS数据是否就绪、任务执行状态及文件保存状态。`running`仅表示进程或任务在运行，不能视为完成；物品坐标仍为map米制坐标。传感器或TF过期会阻止新任务，停止按钮保留。没有收到可信任务终态时阻止新任务并提示重启，避免控制权不明时继续执行。

`robot_runtime.py`负责项目/ROS域锁、独立会话目录及进程清理；`robot_monitor.py`只读订阅和生命周期查询。退出仅处理带本次所有权标记的进程，核对UID、启动标识并使用pidfd，包含导航包装脚本另起的进程组。普通浏览器不继承此标记。

本版是搜索/盘点的控制台适配器：尚无跨建图/清扫的统一ROS Action、运动仲裁、任务队列或持久语义记忆。传感器检查也不是四项功能的综合验收。


五区配置：一键入口和控制台解析共同读取 `rooms_aws_house_v4.json`。`GET /api/state` 返回 `rooms_file`、`rooms_frame_id` 和 `rooms`；UI直接使用其中的 `display_name`、`color`、`label_position`、`polygon`、`views/category_views`，避免前后端维护两套区域。支持kitchen/living/bedroom/utility/gym；健身区也接受“运动区、健身房”，厨房也接受“餐区、餐厅”。分区不改变导航障碍地图；场景仍保留原家具与通道。
