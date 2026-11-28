# 任务输入与结果接口

2026-11-21更新：`GET /api/state`的`object_catalog`提供16类目录和`admitted`状态，默认正式支持杯、瓶、椅子；其他类别通过`--candidate-categories`显式开启验证，仍标为候选。`GET /api/memory` 按当前实际地图与房间几何隔离历史记忆，可用 `room`、`category` 过滤；返回最后观测位置、时间、证据、空间记录 ID、关联状态、修订号及 `graph_available`。历史记录总数不能作为当前房间库存。

搜索结果与本次持久化文件一致后，控制台事务写入 `results/semantic_memory.sqlite3`；同一任务重复导入不增加观测次数，内容变化则拒绝覆盖。盘点返回`completed`或`not_found`，且`observation_plan_completed`与`observation_data_complete`均为true时，未再确认的旧物品标为待核验；空盘点仍返回本次计数0。历史坐标、最后观测时间及证据保留，部分观察与单物体寻找不触发这一状态更新。已准备语义环境时调用 Spark-DSG 导出家庭—房间—物品场景图，位置取自感知结果，房间归属由坐标计算。`GET /api/memory/graph` 仅提供与当前地图和记忆修订一致的图；尚无图时返回 409。图中保留原证据引用，当前无外观特征或物体完整尺寸，物理身份保持未核验。记忆或导出失败不改写已确认的导航终态。

网页支持文字、选择目标和离线语音转写，三种输入共用现有房间与类别解析。语音转写后由用户确认并提交任务。

房间配置可用`inspection_views`按区域名定义必巡观察姿态，`inspection_labels`提供显示名；启用的姿态优先于类别视点并去重，未启用的历史配置保持原路线。`inspection_coverage`分别记录各区域到达、完整停留及有效同步数据的完成状态；find须完成必巡区域后才提前返回，必巡未完成保留partial和当前确认物品。进度提供`inspection_area`，UI显示巡视位置，结果显示各区已巡视/未完成。新增厨房v5为餐桌与L形操作台配置，真值不参与决定当前数量或定位。

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

结构化格式接受 `mode`、`room`、`category` 和可选的 `request_id`。`find`和`count`指定当前场景的房间ID及已启用类别，默认类别为`chair`/`cup`/`bottle`；验证启动时可从16类目录增加候选。取消使用 `{"mode":"cancel"}`，附带 `request_id` 时只取消对应任务。请求 ID 长度为 8–80，仅允许字母、数字、下划线和连字符。房间 ID 从所选场景的 `rooms_*.json` 读取。

可用现有客户端测试接口：

```bash
source scripts/env.sh
python3 scripts/run_search.py '{"mode":"count","room":"kitchen","category":"chair"}' \
  --timeout 480 --output results/aws_house/task_json_latest.json
```

客户端以 **VOLATILE** QoS 订阅之后发布的结果。连续发送相同旧格式命令时，可重复传入 `--exclude-task-id <已完成任务ID>`，忽略延迟到达的已知结果。`scripts/run_search_matrix.py` 自动传入本批的历史任务 ID，并在报告层拒绝重复 ID。可用 `--diagnostics-output <路径>` 保存客户端诊断。网页调用发送带 `request_id` 的原串，结果也回传该 ID。当前协议一次执行一个任务。

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

杯、瓶、椅的基线已有实测；解析支持某个类别或别名本身不代表新外观和变化布局已验收。“可乐”目前只能映射为 `bottle`，没有品牌/饮料种类识别。关键词文本解析和结构化 JSON 最终进入同一执行链路。

当前串行执行一个任务，支持取消，无任务队列。原始ROS命令话题的拒绝信息主要写入日志，任务协议尚非完整ROS Action；已有`/search/progress`逐步反馈，网页HTTP接口对忙碌、未就绪和非法输入提供结构化状态。语音转写、规则解析或后续语言模型适配器生成任务，导航与避障由执行层负责。

## 传感器输入可靠性

`course_detector` 和 `semantic_search` 各自提供只读启动参数 `sensor_reliability`，只接受 `best_effort`（默认，原行为）或 `reliable`。前者只作用于检测器的 RGB 输入；后者作用于搜索的检测、深度和相机内参三个同步输入。需要分别在两个节点启动时设置，例如给 `development_detector.sh`、`development_search.sh` 各加 `-p sensor_reliability:=reliable`；运行中 `ros2 param set` 会因只读描述符拒绝，修改后必须重启对应节点，非法字符串在启动时明确报错。

共享helper为每次选择构造独立QoS实例，仅改变reliability，其他sensor-data策略和全局`qos_profile_sensor_data`保持原值。一键入口显式选择RELIABLE；原始节点/调试入口的默认值与安全候选点云策略分别说明，不能混作同一配置。`reliable` 输入要求发布者提供 RELIABLE；它与 BEST_EFFORT 发布者不兼容，不能把该选项当成对任意输入的通用保证。实际 RMW 兼容性和全局预设隔离由 `tests/test_sensor_qos.py` 在不启动节点的条件下检查，运行中的接收率仍需另存实测证据。

`scripts/validate_sensors.py` 同样支持 `--sensor-reliability best_effort|reliable`（默认 best_effort），使用同一 helper 配置其六个传感器/时钟输入，并把选择写入 `sensors.json` 的 `sensor_reliability`。验证显式 reliable 候选时使用 `--sensor-reliability reliable`，以免把不同监听策略的健康快照混称为同一链路结果。既有 45 例、补等重跑及故障记录保留各自加载版本和 QoS，不能追溯改标为新选项的验证。

## 输出与坐标

2026-11-23杯尺寸检查补充：启用`validate_object_size`时，除原最大尺寸上限外，cup的实际掩膜三维extent至少有一轴≥.04m，否则可见几何不足，不进入实例关联；不改检测score=.35、2hits或merge_radius=.30。依据本轮真实灶台旋钮误识(.01012/.02339/.02713m)和冻结真实杯最大片段约.079m的区分加入，不是通用物体身份识别。较小/严重遮挡的杯可能被拒，须另一个充分观察视点；无掩膜路径、其他类别不新增此下限。冻结回放通过，后续5次现场计数通过但未触发本分支。[证据](../../results/development/demo_two_rooms_20261123/count_diagnosis/audit.json)

- `/search/result`：`std_msgs/msg/String`，`data` 为结果 JSON。
- `/search/markers`：`visualization_msgs/msg/MarkerArray`，包含地图坐标系中的物体球和编号文本。
- `results/<场景>/tasks/<UTC时间戳>/result.json`：任务、状态、实例列表、坐标、命中次数、视点及传感器统计。
- 同目录 `observations.json`：逐帧投影、关联 ID 和已确认 ID。
- 收到 `/map` 时，同目录自动生成 `semantic_map.svg`，叠加本次目标类别的物体编号。
- PNG 是通过 `scripts/plot_result.py` 额外导出的预览，不是每次任务默认生成的文件。

新运行结果显式加入 `frame_id: "map"`、`coordinate_unit: "m"`。原历史结果不回写，其坐标语义同样来自代码中的 `map` 变换。

`x/y/z` 是 **map 坐标系中的米制三维位置估计**，由图像像素、深度、相机内参和采样时刻的 TF 计算。XY 用于二维地图标注，Z 表示高度分量；估计受可见表面、遮挡和定位误差影响，不是抓取点或已知模型中心。

地图原点由建图/定位确定，不是图片左上角；保存地图 YAML 的 `origin` 是栅格图像在 map 中的位姿。当前仿真与地图做了对齐，不能对其他地图假定 `map == Gazebo world`。图像像素、机器人 `base_link`、相机光学坐标和 `map` 是不同坐标系。

地图底图独立于任务保存，搜索不会重新建图。当前AWS导航由`map_resources.json`注册的`aws_house_v1`载入`scenes/aws_house/r0_v1/maps/house.yaml`；baseline使用独立`baseline_v1`。`map_resources.py`为场景选择、导航脚本、Runtime、充电绑定和离线绘图/空间检查提供公共入口；两处AWS兼容副本保留，现存副本除image路径外的元数据及图像散列必须一致，否则拒绝混用。显式未注册文件保持独立选择，不按图像相同自动归并。

会话`config`新增`map_id`、`map`、`map_resource`收据：主YAML/图像相对路径、元数据、双SHA、原请求YAML及其SHA、已校验副本列表；主YAML和图像冻结到`configuration/`。ROS路径继续使用ASCII工作区别名。充电配置仍核验所绑定原YAML/PGM双SHA，并与实际选择的地图几何和图像一致性核对。文件资源ID不替代实际`/map`与房间几何产生的语义namespace，也不证明新地图的房间或充电坐标有效。

`save_map.sh`默认选择`results/maps/<scene>/<UTC时间_随机标识>/map`文件前缀，显式`map_output:=<前缀>`也拒绝注册图、已有输出或项目外路径。控制台继续指定会话请求内的新前缀；保存不自动激活新底图。任务内物体ID每次重新分配，持久记忆另有跨任务空间记录ID和SQLite数据库；空间记录ID尚不代表移动后的同一物理身份。房间、物品、轨迹和覆盖作为同一底图的图层，PNG/SVG是导出快照，统一视图清单见[总计划](DEPLOYMENT_PLAN.md#地图数据与输出视图)。

`completed` 表示全部配置观察点已到达、完成停留、均有可用观测且已确认实例；相同条件下没有确认实例为 `not_found`。`count` 是本次已确认的数量，不能据此保证房间所有物品均被看见。新增字段和状态如下（历史 JSON 不回写）：

- `viewpoints` 只列实际成功到达的点，`viewpoint_failures` 记录失败索引、位姿与 Nav2 状态码。单点导航失败后继续尝试其余视点；有失败视点的计数任务返回 `partial`，不冒充完整结果；find 确认对象后仍可提前返回 `found`。
- `viewpoint_diagnostics` 按配置顺序列出所有视点，包括未到达点。每项包含 `index`、`pose`、`reached`、`navigation_status`、`dwell_completed`，以及该点独立的 `synchronized_messages`、`rejected_sensor_frames`、`synchronized_frames`、`tf_failures`、`invalid_target_depth`、`valid_projections`。消息只在停留阶段计入，排除到达前和重复时间戳；内参、尺寸或编码不合格的同步消息记入 rejected，不能当成有效帧。深度与内参尺寸相等也必须宽高均大于 0，空图不能支持 not_found。旧的任务级统计字段继续保留。
- `reject_clipped_detections` 控制是否过滤检测框距图像边缘不足2像素的截断目标；一键入口启用，原入口默认关闭。过滤数量记录在任务级和逐视点的 `clipped_detections`，被过滤实例需在其他完整视角重新确认。v4在既有视点后增加45°间隔周向观察，同一位置的不同朝向分别记录；它们不等于不同物理位置或遮挡区域已看清。
- `validate_object_size` 对有实例掩膜的杯瓶使用RGB-D可见表面尺寸检查；一键入口启用，原入口默认关闭。杯的水平包围盒对角线/高度上限为0.25/0.30m，瓶为0.35/0.60m；不是精确外形尺寸，也不读取场景真值。任务级和逐视点的 `size_rejected_detections` 记录过滤数量；无掩膜的兼容回退路径不执行此项检查。
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

`run_search.py --discovery-timeout 15` 在位姿/至少2秒预备阶段后，独立等待命令订阅与结果发布均已发现的同一服务节点，排除只读指令跟踪器，单位墙钟秒；0只做即时检查。`--timeout` 从任务发布后开始。未发现返回码2、`outcome=discovery_timeout`；等待中SIGINT/SIGTERM返回130；两者均 `command_sent=false`，不发送取消。诊断保存发现等待耗时、发布前匹配数及`serving_nodes_before_publish`/`result_publishers_before_publish`，初始TF/odom在等待后取最新可用值，仍不保证位姿新鲜度。所有超时必须有限；任务超时正数，其余等待非负。矩阵同名参数原样转发，加入子进程看门狗预算。独立测试收据见 [客户端就绪修复](../evidence/client_discovery_fix_20261117.json)。


## 导航代价快照与侧向补扫（本轮）

`POST /api/map/export {}`允许任务运行时保存只读派生快照，不下发动作。返回session_id与allowlist结果链接，文件在本会话map_snapshots/随机ID下：navigation_cost.png、navigation_cost.svg、snapshot.json。只接受同map的/map与新鲜/global_costmap/costmap（源戳/接收保护仍2.5秒）；过期/坐标不符返回409，不把缓存冒充实时。背景按两源原点、朝向、分辨率投到共用底图；原生计划仅有效时绘制且用虚线，实际清扫轨迹明确为UI采样，各自源戳保留、未做同步插值。局部odom图未接入。PNG中文使用python3-pil与fonts-noto-cjk；旧后台无map_export_available时隐藏按钮。

默认nav2_depth现将local_costmap与behavior_server配套设为odom并关闭local静态层，全局地图仍map；两区单次清扫回归通过，日常避障当前配置完整复测另记，不等于历史body候选已启用。侧向补扫仅在原条带直接补扫无候选后产生安全短段，按实测漏格/当前连通安全中心/刷盘半径选取，有限重试且不改原分母。 主条带遍历后，覆盖仍不足80%时，可先选择预计新扫格/秒至少高25%且剩余时间足够的可达区域补扫，随后重算原条带缺口；达到80%后仍优先原条带完整性，不因总覆盖达标提前结束。remaining_selection.candidates保留selection_reason及被暂后置修补的原lane_id/格数/估时，预测不产生覆盖记录。覆盖仍由实际map轨迹刷盘重算，不从预测或计划增加。具体现场证据见清扫修复收据。

## RGB-D独立清除流与导航代价图（2026-11-23）

`/rgbd/obstacles`保持原有限XYZ点及原标障高度/距离；`/rgbd/clearing`是**仅供clearing的射线端点流**，包含有限实际返回和经过校验的仿真正Inf无回波量程端点，不能用于marking或语义物品定位。默认`clearing_max_depth=0`不补射线，仅仿真launch从实际相机clip/far标定值显式开启。要求原云≤.25秒、内参≤.5秒/无畸变/尺寸一致，有限点与Fortress半像素投影p95误差≤.002m；NaN/零/负Inf及无法验证的几何不变成自由空间。默认Nav2清除源只清除不标障，expected_update_rate .4秒、raytrace4m，端点由原生体素层裁剪，不依赖物品真值或按时间擦除历史。`/rgbd/cloud_adapter_state`新增`clearing_no_return_rays`/`clearing_rejected_frames`供诊断。

显式`--depth-cloud-source image`将`config.depth_cloud_source`记录为image（默认native）。适配器`input_source=depth_image`用ROS TimeSynchronizer收齐三对不同源戳的原云/深度/CameraInfo；逐像素采样核对独立原生XYZ、有限返回和正Inf掩膜，最大差≤.002m才输出。校准后销毁原云校准订阅，复用同一密度的已有深度图；640宽stride4、1280宽stride8，未降低导航点密度。CameraInfo内参/尺寸/帧/编码锁定且须新鲜≤.5秒，输入仍≤.25秒。有限XYZ只用于真实标障；只有正Inf可补至实际far端点且每帧仍需至少8个有限锚点，NaN/零/负Inf不得清空地图。

此模式用启动独立校准＋逐帧固定签名核对；不同于native原云模式逐帧以原云核对投影。native路径保留原规则，不能将候选自动用于其他传感器。`/rgbd/cloud_adapter_state`兼容增加input_source、input_payload_bytes、native_calibration_required/verified/pairs/metrics/last_error。端点格式/原源戳/camera_link轴向、Nav2标障和清除主题/高度/距离及运动.4秒诊断不变。YAML中lazy真实生效，原云ROS发布者仍可被发现，但无消费者时不再桥接大数据。候选状态见点云收据，不能仅据小的重投影误差宣称避障验收通过。

`GET /api/costmap`是独立只读全局导航图接口，字段沿用/map的宽高/分辨率/原点/data/frame_id/stamp，额外source、fresh、age_seconds；内容来自`/global_costmap/costmap`，不加入/api/state的大载荷。0为可通行、1–98为通行代价、99为禁行膨胀、100为占据、-1为未知。后台源/收包超过2.5秒不发送有效数据；UI按有效帧缓存覆盖1Hz发布＋轮询链路，用4秒**显示**租约，暂时断流不闪隐，保留并淡化上一帧、明确“已过期”，不把它当实时观测，不放宽导航传感.4秒保护。图层与机器人/轨迹/语义对象共用同一底图。预设巡视点合并inspection_views及类别路线并去重，当前方向橙、已完成方向绿，底层路线本身不由显示层改写。

## UI避障只读遥测（2026-11-23）

`GET /api/state.telemetry.obstacle`来自`/scan`同戳TF与`/odom`，包含`fresh`/`motion_fresh`、`state`、`source`、`stamp_seconds`、map坐标`points`（最多90）、当前运动方向的`nearest{distance_m,bearing_degrees,base_xy,map_xy}`、`sector`、实测`velocity[v,w]`及最近30个状态变化`events`。前进/后退使用对应±60°方向，原地转动检查周围；`/cmd_vel`仅订阅来选方向，不发布命令。距离从base中心计算，≤.70m是显示提示。`near_moving`/`near_stopped`表示近障下运动或速度接近零，不是Nav2成功或完整停车协议结论。scan/odom任一无效或>.4秒状态为stale，失效点/nearest不作为当前观测。

`telemetry.navigation_path`来自原生`/plan`，包含map `points`（最多300）、`stamp_seconds`/`source`和`fresh`；收包及源时间≤2.5秒才可画，UI仅任务运行时显示。该路径与实际清扫/返航轨迹独立，不能计为实际覆盖。地图复用同一底图。激光图层不冒充全部深度障碍，模型真值、类别和障碍名称不进入这些接口。终端`[避障观察]`同源打印，UI只展示最近5个事件。证据见[两展示分区收据](../../results/development/demo_two_rooms_20261123/audit.json)。

## 本地网页入口

`bash scripts/start_robot.sh` 启动 `robot_console.py`，仅监听 `127.0.0.1:8765`；静态界面位于 `ui/`，无需Node/npm安装。网页通过同源HTTP访问，后端通过现有搜索客户端接入ROS；没有更换原 `/search/command` 协议。

| 接口 | 用途 |
|---|---|
| `GET /api/state` | 本次会话、组件进程、实际话题新鲜度、定位/导航就绪、当前任务及最近30条历史 |
| `GET /api/map` | 实际 `/map` 栅格、分辨率、原点/朝向及坐标系 |
| `POST /api/task` | 导航模式接受 `request_id` 和 `mode/room/category`，或 `mode:"clean"` 与 `room`，或 `command` 文字。建图模式接受 `request_id` 和运动指令 `command` |
| `POST /api/mode` | 空闲时提交 `{"mode":"mapping"}` 或 `{"mode":"navigation"}`，通过 `/api/state` 查看切换状态 |
| `POST /api/mapping/save` | 停车后提交空JSON对象，保存当前 SLAM 地图为 YAML/PGM |
| `POST /api/transcribe` | 发送 `application/octet-stream` 音频，最多 256 KiB。返回转写任务状态，最终通过 `/api/state` 读取 `result.text` |
| `POST /api/cancel` | 空JSON对象，取消当前任务；建图空闲时也可请求停车 |
| `POST /api/validate` | 空JSON对象，真实传感器/TF采样，完成后返回报告 |
| `GET /api/log?component=search` | simulation/navigation/detector/search/rviz/task日志尾部 |
| `GET /api/artifact?id=…` | 使用状态响应返回的结果链接，只能读取本次会话登记的文件 |

`request_id` 用于本次会话内的重复提交保护。相同 ID 和任务返回当前会话状态，避免重复执行。复用 ID 提交不同任务返回 409。搜索节点生成 `task_id`，并在进度和结果里附带网页的 `request_id`。

控制台任务记录包含 `id`、`kind`、`state`、`result`、`error`、`save_status` 和 `artifacts`。搜索、转写、建图运动、地图保存和传感器检查复用这个结构。网页将搜索节点在 `/search/progress` 发布的同请求事件显示在 `task.progress`。进度事件使用 `std_msgs/msg/String` 封装 JSON，包含 `version=1`、`request_id`、`task_id`、`kind`、`stage` 及房间、视点和确认数量。搜索阶段依次由导航、观察、等待数据或保存等实际事件触发。保存完成事件提供任务状态和文件写入结果。

清扫任务 `kind=clean` 串行复用 Nav2。报告包括 `coverage`、实际路径长度、`initial_pose`、`final_pose`、停靠误差及 `return_state`。网页用 `trace_preview` 显示实际轨迹，完整采样在 `clean_trace.json`。导航回执缺失时控制台关闭本次运行并禁用新任务。地图 `clean_map.svg` 绿色表示实测已扫，浅黄色表示可清扫但未覆盖，灰蓝色表示扫幅无法到达的已知地面。

C041按用户确认将清扫目标改为80%，同时要求`spatial_coverage.completed=true`：以原始规划条带中心线为检查单元，每格必须被实际刷盘覆盖；计划线和后续受阻均不增加覆盖或删除原检查格。报告公开`planned_lanes/covered_lanes/incomplete_lanes`，全局80%但仍缺条带时继续清扫并如实保留partial。中间`lane_start`使用`navigate_cleaning_connector.xml`和原生位置checker，回充与入房保持完整朝向；门槛变化不改旧结果。

C041队列保护：当前实测位置没有安全连通中心格时，内部选择原因记为`selection_pose_not_navigable`，保持未执行条带队列，不能据此报告地面已扫尽。最多2次复用已有锚点恢复，恢复事件为`clean_plan.events[].kind=selection_pose_recovery`（含attempt/limit/previous_anchor与实时掩码）；仅确认停车且恢复后的实测安全检查通过才重选。恢复失败走原导航失败/回充流程，`spatial_coverage`原检查格不删，规划和锚点不计入覆盖。

C041定向补扫：普通选路保持原主条带顺序；队列处理后，`remaining_floor`按原主条带的栅格轴向优先生成垂直方向短段，无有效安全候选时回退另一方向。只取当前实际未扫地面的刷盘增益区，分开相隔的漏扫块，不将中间已扫区合成一条长补扫带。原eligible、主条带检查格、实时障碍/停车条件不变。`remaining_lane_attempts`及选路候选记录`orientation/preferred_orientation`，方向为地图栅格坐标的horizontal/vertical；主条带可为null。预测增益不更新覆盖。若已满足80%且全部原条带已实扫，则直接返航，不强制整房交叉第二遍。

语义引导候选由`--memory-guidance`显式启用，默认关闭。`find`读取同namespace的真实历史坐标，保留必巡前缀，再优先历史成功的可选点及朝向/距离得分；在现有已标定点中选择.8–2.2m近点，原方向未覆盖历史方位时最多加1个核验朝向。位置不新增、原配置点全部保留回退。成功视点由SQLite任务内ID/空间记录ID及observations至少2帧关联支持，缺证据回退几何先验。`count`保持原完整路线，不从历史补数量。`memory_guidance`公开baseline_viewpoints、verification_views（原点/历史ID/历史XY/距离/依据）、added_viewpoints、顺序变化及counts_from_history=false。`viewpoint_diagnostics`包含完整预留计划，`viewpoints`是实际已到访列表；find满足必巡和所需数量后可早停，不要求走完全部回退，但回退不能预先删掉。Gym单次正确查找耗时减少，新增朝向0有效投影且原无记忆任务时效超界，仍属候选，未通过全系统收益/稳定验收。

视觉辅助均为显式候选，默认不启用。旧`--ball-model yoloe-11s-seg-pf.pt`保留；新`--auxiliary-model yoloe-11s-seg.pt --auxiliary-categories book 'sports ball'`加载资源锁内37KB提示嵌入，基础权重、嵌入、词表与配置冻结到会话。主模型继续输出，辅助最多2Hz，只补书/球；同类别像素框IoU≥.5保留主模型，非重叠实例保留。exercise ball/beach ball等明确球子类归为COCO32，book为73，balloon及辅助模型的其他类别不进入输出。原分数、图像掩膜和时间戳保留，辅助推理结束后仍核对原帧≤.4秒。球面几何仅用实测点云拟合，失败回退原表面估计；未放宽计数/位置或大小检查。

网页语义/运行地图共用原栅格底图，房间—物品关系图是附加视图。统计分别显示历史记录、待核验记录、所选任务确认数量；历史记录数量不等于当前现场物品数量。关系边仅来自同版SQLite记录的房间归属，节点选中联动档案与地图；尚未推断物理身份、支撑或相邻关系。地图类别着色与虚线状态独立：分类颜色不表示当前确认。

固定充电配置为`charging_station.json`，冻结到会话并绑定当前地图双SHA；`GET /api/state.charging_station`公开配置设施的map位姿，区别于感知物品。默认场景出生与固定充电处一致；旧`spatial_v4.json`的home(0,0,0)继续作为历史导航校准点，不改写成充电处。历史回归可显式`--no-charging-station`。

寻物/盘点的持久化result/observations/SVG完整且与客户端结果一致后，控制台串行运行公共返航，保持同一任务忙碌。原感知`result`不变；独立`task.return`、`return_result.json`、`return_trace.json`保存返航状态、目标/实测位姿、误差、原动作状态及停车确认。返航失败不删除感知结果或记忆；保存期间取消会跳过返航，返航中取消切到当前受管子进程，终态或停稳未知时关闭所属栈。建图结束切回默认导航后共用同一返航。文字“返回充电处”或`POST /api/task`的`{request_id,mode:"return"}`可单独回充。

C035公共返航进程在原25秒就绪窗口内持续处理消息并至少等待2秒，并继续要求新鲜位姿、停稳和Action服务就绪后才发首目标；报告新增`discovery_warmup_seconds`及`startup_ready_after_seconds`（尚未就绪为null）。总截止、目标响应超时和终态/停车屏障不变。该等待针对实际响应发送超时做缓解，单次冷复测通过，不保证所有DDS响应丢失均已解决。

清扫报告`initial_pose`始终为实际任务起点，`return_target_pose`单独表示固定充电目标，`charging_station_id`标识配置设施；返航预留按该目标计算，避免先回任务起点再二次回充。`navigation_client.py`共用Nav2终态/新鲜停稳屏障，`docking.py`共用停靠判定。各任务只有实际XY/yaw及停稳达标才置`simulated_charging`。

清扫入房/跨房时限依据实际定位到目标的距离估算并有界在40–180秒，最终返航最低50秒；这是墙钟预算，非规划路径或保证到达时间，仍受任务总截止和Nav2进展/恢复检查限制。报告新增`return_reserve_seconds`，目标记录`timeout_seconds`和接受后的`effective_timeout_seconds`（受当前阶段截止限制）；条带目标保持原40秒。Nav2报告到达后仍独立核对实测停靠，距起点.75m内超差时调用既有approach/alignment校正，最终仍须.15m/.15rad且停稳才能模拟充电。入房失败消息区分超时，并说明实际返航是否达标。

清扫每个已接受的导航目标收到终态后，重新累计至少0.45秒停稳里程计（最新数据不超过0.40秒），在2秒墙钟内确认后才允许发送下一目标，覆盖正常到达、受阻失败、超时和取消路径。`clean_trace.json`的`goals`保留原`action_status`，并记录`terminal_seconds`、`finished_seconds`、`stop_confirmed`，停车等待失败另存`stop_error`。未确认终态或停稳时`navigation_settled=false`，任务保存失败原因，控制台关闭本次运行并禁用新任务；被拒绝、未接受的目标不要求新增停车区间。位置端点实验仍默认关闭，覆盖与停靠容差保持原值。
`goals_finished`为历史兼容计数：C032当前是已处理条带数的两倍（含失败/未发送）；C031按原计划端点处理（含跳过/失败）。它不等于Nav2成功数量；重排动作另见`lane_replans`。实际成功/取消/失败以`clean_trace.goals`原始终态为准，覆盖仍只按实测轨迹。最终客户端在路径准备后、发送前再次核对截止、停止请求及新鲜定位；未发送记`deadline_before_send/canceled_before_send/pose_unavailable_before_send`，不伪造接受或动作终态。


清扫恢复锚点单独检查实测位姿是否新鲜、在当前房间可通行中心掩码内且实时全局代价可通行。导航成功回执不直接覆盖旧锚点；不合格位置保留旧的已验证锚点。`clean_trace.json`每个已检查目标的`recovery_anchor_check`包含实测位置、栅格坐标和接受标志；从房间内起步且尚无导航目标时，报告另含`initial_recovery_anchor_check`。恢复动作即使返回成功，实测位置仍不可通行时记`unsafe_arrival`并停止继续清扫，原`action_status`保留。该检查不证明未来动态通道一定畅通。

已核验清扫条带使用`FollowPath`直接传入半栅格步长的map参考路径；入房、条带连接和返航继续`NavigateToPose`/NavFn。参考路径逐格检查静态连通中心、当前实时可通行与斜向角格，计划不计入覆盖。`clean_trace.goals`另存`action_type`、`reference_path`和`plan_endpoint.source`，同一客户端统一取消/终态/停车屏障。条带目标已在真实.12m内且新鲜停稳时记`already_reached`、不发送动作，不填写action_status冒充Nav2成功；路径受阻记`path_blocked`。优先从当前安全连通位置有界重排，失败才回旧锚点。`cleaning_stop_reason`区分入房失败、清扫预算、受阻恢复、定位、路线与补扫用尽、覆盖达标、取消和执行异常；返航状态另记。

默认保持C032往复式主清扫顺序，按最新连通实时掩码切段，主遍历后再按实际未扫收益、四邻绕行距离和转向代价重生成.35m候选条带。C033收益调度需显式启动`--cleaning-budget-selection`，控制台向客户端传`--budget-selection`：同时评估主路线及未扫条带，替代段的预计新扫面积/时间至少高25%才提前选择，未选工作保留；时间估计为`5 + (四邻连接距离+条带长度)/.20 + 两次连接转角/.6`秒，另留2秒余量，预计超出清扫剩余时段则不选。候选整房70.32%且回充失败，默认关闭；估计不保证真实执行耗时。连接及重排起点使用普通Nav2完整恢复树和完整朝向；条带仍同一CleaningPath的FollowPath/.12m Position checker。安全直达参考优先，仅直达会穿邻障时经≤.18m的局部安全接入原条带。到起点后重新切段，实际发送新段终点，`nominal_lane`保留原段；规划收益不增加覆盖。原动作成功但实际odom平移不足.20m记`no_measured_progress`，原action_status保留。仅Path未发送且当前实测安全时可直接重选。最多64段/原截止/10个点补扫及2次观察转向均有界；继承Nav2原有恢复服务，不停用传感器或放松99/角格判断。

条带与补扫候选默认在原静态可通行中心上另加一格四邻跟踪余量（格长约.05m），不缩减原eligible或刷宽。C041新增房间级`cleaning_tracking_margin_cells`，仅接受整数0或1（布尔值无效）；默认1，0仅用于显式局部通行候选。0不侵蚀附加格，基础`CENTER_CLEARANCE_M=.32`、Nav2实体轮廓、实时障碍检查和覆盖分母均保留，不能把SciPy的`iterations=0`直接当无侵蚀。当前候选`rooms_aws_house_v11_margin0.json`仅健身区设0，尚未通过实际局部通行，不进入默认。NPZ的`static_centers`/`planning_centers`为全图中心掩码，房间候选再由冻结房间配置裁剪，格网原点/分辨率见`clean_plan.json`。`tracking_margin_cells`写入报告与选路记录；分别保存原实时掩码和增加静态规划余量后的几何刷盘潜力。位姿接入仍以原静态/实时安全门槛逐格检查。

`clean_plan.json`保存候选/时间/连通潜力/受阻编号，`clean_navigation.npz`保存去重原始代价图/实际覆盖，最多256事件，网页可下载。报告含`route_selection`（默认`seeded_route_then_reachable_remaining_lanes`，实验`budget_aware_seed_and_remaining`）、`budget_selection_enabled`、`remaining_lane_attempts`、`coverage_observations`及`navigation_masks_saved`；尝试来源`seeded_route`表示保持队首、`budget_seed`表示提前选择主路线段、`remaining_floor`表示重生成未扫段。实验候选另存`selection_reason`、`available_seconds`和`queued_score`；恢复另存odom实走及采样起止索引。`cleaning_time_remaining_seconds`记录进入返航前的清扫剩余时段，未到正常收尾分类时可为null；`insufficient_segment_budget`表示实验候选预计无法在剩余时段内完成，`recovery_budget_insufficient`表示恢复时段不足，两者区别于预算耗尽和真实恢复动作失败。实际验收结论见交接与清扫收据，不将几何潜力或冻结掩码重放算现场收益。

C031原生清扫checker当前候选使用原.12m、stateful:false，仅忽略条带端点朝向；普通BT显式选择general checker，保留.12m/.1rad。清扫BT显式选`CleaningPath`（RotationShim包装DWB，移除RotateToGoal），普通BT仍用原`FollowPath`。shim采样.15m、入口.4/退出.1rad、.6rad/s并开启碰撞检查；其异常回落主DWB，不能将配置等同每次纯转向。local滚动窗与behavior共用map并加载静态层，消除目标跨map/odom的历史/latest变换分歧。动作记录`terminal_pose`与停稳后`measured_pose`分开，`plan_endpoint`记录本动作新收到的最后规划末点/时间/年龄，仍不把Nav2成功等同请求坐标精确到达。PoseProgressChecker候选.20m/.50rad/10秒认可转向，不延长单目标/总截止。导航失败后，静态/实时当前位置安全且停车已确认时优先尝试当前连通条带，再回旧锚点恢复；一次最多2起点、全局最多4次，原失败条带至多尾部重试1次。`lane_replans`保留尝试、失败及连续odom实测位移，成功到端点且位移≥.20m才恢复；原覆盖分母不变。`odom_samples`另存原始里程计采样及有效增量，区别于map轨迹和定位修正。

C034默认开发配置将激光二维层与深度体素层独立按最大值融合，外接圆.25m；显式安全候选仍.30m。普通FollowPath复用原生RotationShim，保留普通DWB全部critics、.26m/s与general checker；异常/路径样本不足时仍可能回落DWB，不能保证每次纯转向。清扫`lane_start`的`connector_budget`保存`observed_path_distance_m`、`timeout_seconds`和`source`（`current_mask_four_neighbor_distance`或`euclidean_fallback`），同字段写入尝试及实际发送的goal；阶段截止仍优先。四邻距离只估计当前二值可达绕行，不等于加权NavFn路径长度，完整朝向仍需完成。`/robot/contacts`为Fortress机身/支架接触诊断，未覆盖轮子/脚轮或接入正常任务决策；验证器可据实测接触取消任务，该通道不是语义场景真值输入。

清扫`--cleaning-local-recovery`为默认关闭的实验，传入清扫客户端`--local-recovery`并记入会话配置与报告`local_recovery_enabled`。开启时，旧成功端点距当前超过1m才尝试近期连续实测轨迹中的.4–.85m锚点，仍以普通Nav2执行一次有界恢复。`lane_recoveries`记录原端点、选点来源、轨迹样本、选择距离、时间预算和连续有效TF采样累计里程；新版含`trace_start_index`/`trace_end_index`（起始包含、终止不包含）供重放核对。该里程在采样缺口处不连线。覆盖与停靠门槛不受实验开关影响，当前尚无覆盖收益结论。

候选 `--active-observation` 增加真实 RGB-D 可见采样及有界补点，结果包含 `geometric_coverage`，完整采样在 `coverage_samples.json`。`decisions`记录补点来源、姿态、线索与预测增益。`active_visibility`采用未见采样，`tentative_object_recheck`采用未确认目标，`measured_support_surface`采用实际检测到的桌面线索；语义近景最多2点，整体最多6个补点，选点预算420秒。预测不修改实际覆盖，只有新的传感器观测才能增加已见采样或确认数。覆盖预算耗尽或数据不可用时保留部分结果，不把几何可见率等同于识别完整率。`--image-size` 只在启动时选择 640/960/1280，默认 640，写入会话配置。

物品记忆的 `classification_conflict` 表示同一地图、同一房间中，8 cm 内且高度差不超过 10 cm 的便携物品历史记录出现不同类别。它提示可能误识或物品替换，保持各自证据和类别，不自动选定新标签；家具与支撑物允许空间重叠。UI 与 Spark-DSG 保留该提示；图适配版本改变后旧导出须重新生成。

语音文件在本机转写进程结束后清理，任务可以保留转写文本。网页将文字填入可编辑指令框，用户确认后提交当前模式的任务。转写和执行共用单任务互斥和停止。

控制台状态分开表示启动阶段、组件是否存活、ROS数据是否就绪、任务执行状态及文件保存状态。`running`仅表示进程或任务在运行，不能视为完成；物品坐标仍为map米制坐标。传感器或TF过期会阻止新任务，停止按钮保留。没有收到可信任务终态时阻止新任务并提示重启，避免控制权不明时继续执行。

显式候选`--precision-docking`加载`charging_station_precision.json`：仅dock/dock_alignment/dock_approach使用`navigate_docking.xml`、`DockingPath`及`docking_goal_checker`，位置执行目标.05m、完整朝向.10rad；到达位置阶段后由原生rotation shim完成朝向控制；原独立回充验收.15m/.15rad及停稳门槛保持。一般观察与清扫目标不切换该检查器。新控制器复用原DWB运动/避障参数，参数重写输出禁止YAML别名，避免Humble原生解析失败。该候选是否通过以实测为准，默认充电配置保留。 当前本地后续候选还可由充电配置`refresh_localization_before_dock`请求最多3次`/request_nomotion_update`：仅精确候选、普通或恢复停靠结束后、距目标≤.75m且至少剩8秒时触发；多次请求共用总刷新等待≤3秒，必须收到请求之后的新AMCL源戳/接收时间、仍新鲜且停稳的位姿，并等待TF与该定位测量在1cm/.01rad内一致（AMCL的未来TF时间戳可能使消息先到），不能只凭Empty服务应答。若刷新后偏差>.05m或>.10rad，再在原截止内微调一次（≤12秒）；所有最终独立验收门槛保持。`return_result/clean_result.docking_localization_updates`分别保留请求、来源戳、前后定位与不可用原因；不读取仿真真值，也不设置initialpose。

普通`--memory-guidance`若有真实≥2帧、距历史物品.8–2.2m的成功可选固定视点，先按该实际观测排序，不再抢先插入无成功依据的新朝向；远距历史近点核验能力保留。所有必巡前缀/固定回退保持，计数仍完整现场巡视。新部署无历史时按正常路线；完成真实任务后才有可复用记录。所选客厅案例两组收益通过，不能视作所有场景必然提速。

显式试验`--memory-guidance --memory-safe-approach`默认关闭。新policy_version=2只从真实历史成功视点（至少2帧现场关联）邻近.75m内生成一个核验点：沿用≥.35m已知自由间距、当前安全连通区；历史状态必须last_observed，无分类冲突，观察距离不超过成功距离加半格量化误差，方向变化≤45°，未知格拒绝、目标附近二维遮挡预测不劣于原成功视点。它不保证新视野可识别，必须到场完整驻留/现场hits确认。模型至少预测省1秒才试，仅作为候选选择启发式，不替代原正确性、20cm和实测两组收益要求；旧10%估计门槛属于失败v1启发式，不是验收标准。计数、必巡前缀及全部固定回退不变。verification_views[].planning保留真实历史视点任务/帧数、实际地图与可用掩码/时间戳，验收器重算并核对历史来源；缺数据退回原策略。旧v1仅用于原收据重算，原失败不改写。

`robot_runtime.py`负责项目/ROS域锁、独立会话目录及进程清理；`robot_monitor.py`只读订阅和生命周期查询。退出仅处理带本次所有权标记的进程，核对UID、启动标识并使用pidfd，包含导航包装脚本另起的进程组。普通浏览器不继承此标记。

`--navigation-safety` 的候选链路为 `/cmd_vel` → CollisionMonitor → `/cmd_vel_collision_filtered` → 末端时效保护 → `/cmd_vel_safe` → Gazebo。末端以墙钟20 Hz检查雷达、有效深度、点云、里程计、动态TF和时钟（.4秒）及指令（.3秒），数据过期输出零速。`/safety/state` 是包含 ready/reasons/output 的 JSON；ready 表示数据和防护就绪，空闲时 command_stale 不等同于传感器失效。控制、图像、点云分桥接，导航显式加载 `nav2_safety.yaml`；默认仍保留原配置。

C031桥大数据publisher及adapter/guard/detector DDS输入使用KEEP_LAST(1)，保留原可靠性；感知同步器的20帧时间对齐缓存不改。adapter在抽样前拒绝过期来源、保持原stamp，并发布`/rgbd/cloud_adapter_state`计数/年龄/处理时间。`/safety/state`增加`source_ages`、`output_samples`及`blocked_nonzero_command_samples`，原因变化立即发布；.4秒时效门槛不放宽。RMW图depth=0表示未知，实际桥参数已通过GetParameters核实为1。

搜索默认门槛为2次观测、.30m关联、.35置信度；新增会话显式记录并传入。只读多类筛查工具使用3次门槛，不能混作实际搜索参数。

会话 `session.json` 保存所选布局、房间、导航、模型和源码散列；新增会话的 `configuration_snapshots` 指向 `configuration/` 内关键参数原件，`source_snapshots` 指向 `sources/` 内已散列的小型运行源码，避免继续编辑后无法恢复旧实验版本。权重仅记录散列，按资源锁恢复，不重复复制大文件。旧会话没有该字段时不补称已自动冻结原件。

建图模式先退出原导航和搜索组件，再启动 SLAM 与 `mapping_motion`。控制台通过 `/mapping/command` 和 `/mapping/result` 关联运动请求，在 `telemetry.mapping_motion` 显示 `/mapping/state` 的反馈。建图时运动节点独占 `/cmd_vel`，读取 `/scan`、`/odom` 和 TF 执行限速、短时运动及障碍保护。终端键盘可将速度话题重映射为 `/mapping/keyboard`，按 `k` 停车并解除连续按键限制。导航恢复时加载默认校准地图。


五区配置：一键入口和控制台解析共同读取 `rooms_aws_house_v4.json`。`GET /api/state` 返回 `rooms_file`、`rooms_frame_id` 和 `rooms`；UI直接使用其中的 `display_name`、`color`、`label_position`、`polygon`、`views/category_views`，避免前后端维护两套区域。支持kitchen/living/bedroom/utility/gym；健身区也接受“运动区、健身房”，厨房也接受“餐区、餐厅”。分区不改变导航障碍地图；场景仍保留原家具与通道。

房间配置可显式选择`cleaning_route_order=coarse_then_infill`作为C039候选，默认`adjacent`保持原相邻条带顺序。候选只重排初始原条带并按临近端重定方向，先隔条走向远侧，再反向填间隔；剩余区域再生逻辑、Nav2、实时检查、覆盖分母/计数及停车门槛不变。`clean_result.initial_route_order`保留实际选择，候选已实跑但整房74.66%低于原75.20%、重复面积增加，未接受；v6已恢复adjacent，仅保留显式实验能力，不把规划端点当作已扫。条带失败且安全停稳、原队列尚未结束时先继续原队列，不立刻插入全局横向补扫；记录remaining_lane_attempts[].continuation_policy，失败状态保留。修复后完整实跑收益仍待验证。


C040杯/瓶的instance_geometry投影新增category参数：仅这两类在原近深度窗口内做IQR深度异常值过滤，排除掩膜边缘/把手后背景，再执行原尺寸检查；不是按目标尺寸裁剪点云，也不改变类别先验、确认hits、坐标或计数门槛，其他类别保持原估计。默认nav2_depth普通/清扫控制器与平滑器线速度上限同步.35m/s；原基础与安全候选配置保留，实际速度/停车/回充以收据为准。v6/service_surfaces_v2已成为功能联调默认，旧配置显式选择能力保留。

C041开发仿真控制点云`/rgbd/obstacles`默认采用紧凑float32 XYZ（point_step=12、camera_link、保留原源stamp），按原160×120采样密度过滤非有限值；仅删除该控制点云的非XYZ字段，感知仍使用原RGB/深度图。输入`/rgbd/points_raw`可靠接收、KEEP_LAST(1)，.1秒处理周期，原.25秒最大输入年龄；独立桥接避免大消息与控制共享同一执行进程。

C041默认补扫在原条带队列后生成`remaining_floor`候选：按实际未扫格卷积增益的连续区间拆短段，优先与原主条带网格方向垂直；无有效/安全候选时回退另一方向。`clean_plan.events`候选及`remaining_lane_attempts`补充`orientation/preferred_orientation`（horizontal/vertical，均相对底图网格；原条带可为null）。达到80%且原条带实扫检查通过时可正常结束，不额外强制整房横竖两遍；预算实验保持显式关闭，原eligible/实时阈值/覆盖计算不改。


C041健身区修整（2026-11-23）：清扫FollowPath可在整段原中心线已被实际刷盘覆盖、当前位置静态规划与实时可通行、odom/TF新鲜、平移连续停≥.45秒时，发原生取消并重新确认≥.45秒完全停车；报告另记`brush_covered`和原生`action_status=5`，不改普通目标/原生成功状态。短恢复仅原位置不可通行→新位置可通行且总odom位移≥1格/净位移≥半格可少于.20m，零位移/绕圈拒绝。入房实际线速中位数（≥20移动样本、上限.35）可重估返航预留，保留100秒最低与短任务半预算上限；`return_budget_estimate`保存原值及来源。

显式`--body-footprint`使用`nav2_body_footprint.yaml`的局部完整机身/轮/传感器碰撞外包8点轮廓（全局同样包住全部碰撞体的矩形；local padding .03/global .01），深度最低障碍高度.02（默认圆形配置仍.08），两控制器均使用官方`ObstacleFootprint`且配置scale=.4（Humble乘.05m后有效.02）；默认圆形配置保留，暂不与navigation-safety组合。房间级`cleaning_transit_clearance_m`（默认.32，显式范围.22–.32）只影响连接筛选/局部再接入，运行选择低于.32必须同时body候选；清扫参考中心和eligible仍按.32+半格构造，`planning_centers`保留参考中心的原跟踪侵蚀，`transit_centers`独立连通。该筛选不是实体碰撞判据，完整轮廓由Nav2执行检查。v12仅gym显式.22/附加0；导航实时cost<99、未知拒绝、原覆盖分母/80%/全原条带要求不变。原路线会根据选定余量与现场代价图重新生成，不能宣称与默认25条字节相同。局部已实走并回充，完整健身结果以收据为准。


C041安全恢复（11月23日）：`navigate_service.xml`、`navigate_service_through_poses.xml`与`navigate_cleaning_connector.xml`的恢复子节点不再调用ClearEntireCostmap，改为官方Wait1秒等待更新后重试；保留原生重规划、Spin/BackUp及次数上限。调用方不把清图后的“自由”当作障碍消失证据，盲区实测障碍可保留；body候选的两个VoxelLayer显式`footprint_clearing_enabled=false`。这是共同BT行为变化；默认圆形导航YAML/地图仍保留。当前只对局部绕行已实跑，全部四项日常及完整清扫以收据为准。


C041权重单位校正：本机Humble `ObstacleFootprintCritic.getScale()`乘costmap分辨率，原BaseObstacle不乘；5cm图需配置`.4`才是原`.02`有效权重。真实评估消息已读到≈`.02`。global候选使用包住全部碰撞体的.40×.36m矩形、local仍真实8点（padding .03），global padding .01；这不使NavFn成为SE2规划器，实际执行由本地完整碰撞检查。其余运动上限、goal checker、硬碰撞拒绝不改，按实跑收据区分局部/整房。


C041显式body候选的local_costmap.global_frame与behavior_server.global_frame配套为`odom`，global_costmap仍`map`；普通任务/轨迹/清扫覆盖与物品输出仍统一map坐标。局部诊断产物按各自header.frame_id和对应TF源戳读取，不能将odom代价图坐标直接当map绘制。保留体素历史不盲清、真实全身轮廓/低障碍门槛；当前连续坐标系改动的覆盖收益待实跑。


C041恢复目标余量：仅在lane_recovery选择端点时，`RoomSweep.recovery_target_valid`按原参考中心与当前free格检查完整.12m到点圆盘（圆与整格方块相交判断，未知/越界拒绝）。旧安全点余量不足时，从既有30秒连续实走轨迹选.4–.85m锚点，仍经原Nav2/停车/实际落点检查；`recent_trace_with_arrival_margin`与`arrival_tolerance_m`写入lane_recoveries。此端点筛选不改变可扫分母、刷宽、原条带、动态mask或goal checker。无有余量轨迹点明确失败，不直接驱动或清图。


C041低趾/观测保留修整候选：local滚动图6×6m纳入2.5m marking范围及相机水平偏置，local padding=.03（半格.025向上取整，左右失败夹具重放交集）、global仍.01。这是保守栅格余量，不能称已测误差上界；原.02高度门槛、160×120采样及.4秒时效不改，低于.02的真实碰撞片不伪造成观测点。较大轮廓须重新验证器械背面通行和双椅安全。最新值优先于上文历史两图同padding的记录。


C041当前body候选局部层：`static_layer.enabled=false`，global静态仍开启，local激光/深度/膨胀层与完整轮廓仍工作。全局历史栅格不再作为局部近身碰撞输入，避免其map→odom定位修正投影在实际机身边缘造成硬拒绝；并未删除底图或清空实测障碍。运行时单层对照的override仅用于a824根独立返航，之后新冷根才用固化配置验后方/双椅/整房。


C041 11月23日最新body显式候选：先执行`bash scripts/build.sh`构建新增`service_navfn`原生适配包（现在共三包）。GridBased为`service_navfn/CellCenteredNavfn`，仍安装NavFn Dijkstra，仅统一floor索引/格心坐标；global laser/depth为`service_navfn/OdomObstacleLayer`和`service_navfn/OdomVoxelLayer`，原生感知/持久记忆存odom后以当前TF投回原map主图。全局静态底图/输出尺寸/膨胀保留；local6m/odom/真实8点/.03/staticFalse仍原值，默认nav2_depth/v6不变。TF失败或超出4m额外射线缓存范围时原生costmap current=false，不能把缺图当自由。仅body cleaning checker .04，general仍.12；lane_start父动作成功另须实测距原请求≤.12，否则adjusted_goal_unreached保留action_status=4及停稳证据，不把替代目标当到达。最新局部/整房结果以HANDOFF与修整收据为准，尚无新整房通过。


C041原条带小缺口：默认主遍历后优先`required_stripe_repair`，针对原中心线实际未扫且当前可达的格构造≥.14m片段，近似段最多2次，再进入普通换向残余区域补扫。仅此原必扫缺口允许低于.04m²增益；未扫/受阻不从原验收集合移除，计划不增覆盖，预算/刷宽/速度不变。日志保存`original_lane_id`和`required_centerline_cells`。此修复的真实整房结果按HANDOFF读取。


11月23日当前body/v12 gym另有`cleaning_turning_radius_m=.321`：只调整实际执行端点，最多沿原条带内收.15m；`execution_segment`保留原参考并记录入口转向判断，短段单点须实测原段全部刷到且停车才完成。原分母/原条带/预算不变。该显式候选最新整房56.615%/8/23未通过，所有试验根已停；Native Lattice仅在results内准备，尚未授权、没有可用产品启动开关，不能将其文档视作已部署。最新事实以HANDOFF和footprint_trial/audit.json为准。


统一导图沿用`POST /api/map/export`：`{}`或`{"view":"cost"}`保留原全局代价导出；新增`occupancy`、`rooms`、`semantic`、`observations`、`clean_trace`、`clean_coverage`、`relations`。每次返回本会话allowlist下PNG/SVG/原始JSON。`map_export_views`为能力清单，旧后台隐藏新下拉。`semantic`/`relations`可用`room`/`category`过滤；`observations`/`clean_*`用本会话`task_id`（省略则当前任务），不能传文件路径或访问其他会话。清扫仅已结束且保存完整原栅格/报告的任务可导出，原partial不改。

`shared_map_view_v1`保留原底图、分区、所选任务和必要源数据；`captured_at`为快照导出时间，图上写“导出”，各源保留原始sim戳/修订/会话，不声称同步采集。语义图M-空间ID和#任务编号分开，历史不作现场库存；观察图灰圈为目标，绿色位姿/方向必须来自有效实测到达及停留数据。清扫原始map/resolution/origin、floor/eligible/covered格数须一致，定位断点不连线，非目标房间不涂成不可达；关系图须核验真实Spark-DSG的家庭—房间、房间—物品边和记忆修订，缺证据返回409。导出不派发运动或重规划，不支持局部odom代价直接当map。

## 明确顺序任务与核验解释（2026-11-24首版）

`POST /api/task`沿用`request_id,command`，导航模式支持“先去厨房统计杯子，再去卧室找书”等2–3项明确顺序句式。整句全部规范化成功才启动任何子任务；子任务使用独立ID、原报告和原自动回充。只有completed且saved才继续，partial/失败/取消/未知控制终态终止余项；停止按钮取消当前任务及余项。相同父request_id与原文重试不重派。`GET /api/state.sequence`给出父ID/原文/状态及子项ID、文字、canonical、kind、state；会话`sequences/<id>.json`保存回执。没有队列断点续作；重启不自动派发未完任务。建图模式仍单条运动控制。真人转写仍先显示文字再由用户提交。

搜索进度和结果新增`memory_trace`：历史只读坐标/时间/状态、真实成功来源task_id/frames、逐点到达/完整驻留/有效投影及本次确认数。它解释记录，不判断物理身份或可靠缺失；计数仍现场完整巡视，不从历史补数。清扫进度新增`cleaning_phases`及当前有限补扫`supplement`的measured_new_cells/elapsed_seconds；受阻条带是曾受阻记录，不等于永久不可达。字段的实际使用及验收范围见DEVELOPMENT_VALIDATION.md。

`POST /api/task/preview`接受同一`request_id,command`，导航模式返回`sequence`及每项`command,parsed`，只读解析、无任务或进程启动。UI对组合指令先展示清单，点击“执行以上任务”才调用原`/api/task`；提交时仍完整重验，文本或会话改变使预览失效。预览不是跨会话可恢复任务。

精确回充在普通深度配置下单独选用`nav2_depth_docking_progress.yaml`，相对`nav2_depth.yaml`只将DockingPath.GoalDist.scale从24改为96，避免终点附近原地小转评分长期优于合法前进；普通FollowPath/CleaningPath与障碍代价、碰撞约束、速度均不变。body_footprint/navigation_safety候选不自动混用此文件。参数加载不等于停靠通过，仍核对独立位置/朝向与停稳。

清扫进度和结果附`coverage_layer`：地图frame/width/height/resolution/origin、原始signed-int8栅格字节SHA256、实际eligible/covered的RLE和格数，以及带时效的原生代价禁行RLE。UI仅在当前底图几何与`data_sha256`一致时叠加；绿色为实刷，黄色为原可扫未扫，红色仅为当前native≥99禁行，过期即不绘制红层。终态标记结束快照，不把曾受阻格称永久不可达，不用场景真值减分母。`cleaning_phases`的增量来自真实covered掩膜，UI由原resolution换算m²；不是规划面积。

离线语音保持原单任务/建图解码；首次文本已满足明确组合句式时，至多追加一次共享目录和顺序词提示的确定性解码（temperature=0、beam_size=10），仅保留仍能完整拆分的真实ASR原文，不替换同音词。识别后仍须预览确认，不按类别真值补全指令。

只读时效验证器`cloud_continuity.py --ready-file <独立文件>`提供派发前就绪握手：至少预热3秒，且10秒内收到新鲜原生时钟/深度云/激光后才写一次ready；未就绪失败退出。原始启动数据和startup_checks单列保留，measurement_started_wall之后所有检查连续计入，后续断流不重新预热。未传该选项时保持原3秒统计方式。调用方必须在ready之后派任务，不能事后使用此模式裁去任务内超界。

该只读观察器可加`--journal <新JSONL文件>`，持续保存原始cloud/scan、startup和全部measurement检查，每秒flush；最终JSON仍保存完整统计。流水用于诊断中断，缺少任务末端/接触完整性时仍不得据部分流水判通过。长时本地验证采用独立唯一validation owner、有界supervisor及接触检查点；进程退出来源未知时保留not_accepted，不推定机器人失败或安全通过。

`--sim-localization-candidate`为显式仿真运动模型校准，只支持普通深度＋precision_docking。session.config记录sim_localization_candidate及实际navigation文件，configuration_snapshots冻结原生OdometryPublisher模型；alpha1–4从.2隔离为.02，alpha5、测量模型、设施位置/地图、控制和验收门槛保留。输入仍是已有/odom与scan，没有新的物品真值或运行时位姿提示；候选是否准入以独立物理停靠及相关任务为准。

本轮最终收据workflow_completion.json区分专项通过与原清扫partial。运行结束的真实缓存DOM已核对；保存任务还通过只读HTTP回放调用原导出方法检查，代价使用记录时的仿真时间并标历史。该回放只是验证工具，不是实时机器人会话或新的可执行任务入口。
