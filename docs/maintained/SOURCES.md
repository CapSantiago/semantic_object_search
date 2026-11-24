# 上游源码与课程资料

2026-11-24显式精确停靠候选继续复用本机Nav2 1.1.20的[RotationShim最终朝向控制](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_rotation_shim_controller/src/nav2_rotation_shim_controller.cpp)与[DWB RotateToGoal](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_critics/src/rotate_to_goal.cpp)。配置5cm执行目标及独立停靠树，由原生组件执行运动/碰撞检查；保留15cm/.15rad独立验收，项目不声称自研新的局部控制算法。历史记忆安全点复用现有RoomSweep栅格几何、SciPy连通/距离与Nav2到点，当前实测无提速收益，默认关闭。

统一地图导出复用现有NumPy/Pillow栅格与中文绘制，关系边读取真实Spark-DSG JSON；当前`view_export.py`负责同底图图层、PNG/SVG双格式和证据封装。清扫原始刷盘/连续标志、观察实测到达及源时间均来自系统原结果；导出不是另一个地图算法，也没有接入场景真值或补造关系。

2026-11-23默认RGB-D清除修复继续复用Nav2 1.1.20的[VoxelLayer射线裁剪与清除](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/plugins/voxel_layer.cpp)及[ObservationBuffer高度过滤](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/src/observation_buffer.cpp)。本项目适配器分离标障和清除流，并仅在已实测验证的Fortress有组织XYZ/CameraInfo/深度正Inf契约下补无回波射线端点；坏深度不生成自由空间，原有限点标障输出在真实捕获上逐字节相同。传感器远裁剪值来自实际相机标定配置，不读取目标位置。没有新建占据超时清除算法或按仿真删除真值擦图；原生未知/静态保护仍测试。实际同会话清除/重新通行与重复计数失败分开留证于[修复收据](../../results/development/demo_two_rooms_20261123/clearing/audit.json)。

C041 11月23日新增`src/service_navfn`复用已安装Nav2 1.1.20的[NavfnPlanner与NavFn求解器](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_navfn_planner/src/navfn_planner.cpp)：统一Costmap2D floor索引与格心输出，Dijkstra/容差/生命周期仍原生；不是新SE2算法。全局障碍适配复用[ObstacleLayer/ObservationBuffer](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/plugins/obstacle_layer.cpp)和[VoxelLayer](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/plugins/voxel_layer.cpp)，原生感知缓冲/光线清除与持久体素置于odom初始化上下文，每轮按当前TF投影真实占据到原map主图；与原生StaticLayer滚动图分支同样采用逆采样。上下文适配/完整范围刷新/只合并占据由本项目实现，实跑收益另验，未直接宣称上游库解决本项目覆盖。

C041只读诊断复用已安装Nav2 1.1.20的[DWB评分/严格更小值选择](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_core/src/dwb_local_planner.cpp)及消息接口；提前终止的评分不当作完整代价，全轨迹非法时默认best_index=0不代表真实选中命令。实跑靠近终点的73个零线速样本有同分前进候选；[原生Twirling](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_critics/src/twirling.cpp)先做离线重评分，CleaningPath独立.1候选整房71.77%未证明收益，已恢复原配置；普通控制器和原约束始终保留。定位诊断调用[原生request_nomotion_update](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_amcl/src/amcl_node.cpp)，只请求下一次激光更新、无位姿提示；一次完整计数2/3失败，不接默认。第三布局场景离线检查使用[Trimesh CollisionManager](https://trimesh.org/trimesh.collision.html)与[python-fcl](https://github.com/BerkeleyAutomation/python-fcl)确认碗/苹果源网格相交；依赖精确版本与碰撞证据在C041收据，未进入感知/搜索决策。

C032重选条带的连接距离估计复用本机SciPy 1.8.0的[稀疏图Dijkstra](https://docs.scipy.org/doc/scipy-1.8.0/reference/generated/scipy.sparse.csgraph.dijkstra.html)：当前安全栅格四邻边等距、单源无向求距，不把穿过障碍的直线距离当绕行代价。它只参与任务选段，实际规划和控制仍由Nav2完成；条带调度与诊断保存属于本项目适配，收益按真实轨迹验证。

2026-11-24 点云传输候选沿用本机ros_gz_bridge **0.244.26** 的原生惰性桥接。核对同版官方[parameter_bridge.cpp](https://github.com/gazebosim/ros_gz/blob/0.244.26/ros_gz_bridge/src/parameter_bridge.cpp)、[YAML解析](https://github.com/gazebosim/ros_gz/blob/0.244.26/ros_gz_bridge/src/bridge_config.cpp)和[BridgeHandle](https://github.com/gazebosim/ros_gz/blob/0.244.26/ros_gz_bridge/src/bridge_handle.cpp)：该版CLI读取lazy却未赋给逐桥配置，因此候选用YAML的lazy字段；实际日志Lazy1、无原云ROS消费者与Gazebo仅发布端均已核对。每次启动仍用三对同戳原生XYZ独立校验深度投影，之后锁定内参/尺寸/帧/编码，复用已有深度图，非自创传感器或导航算法。Fortress半像素投影沿用本项目已验证约定；同源实测最大误差约1微米，不引用场景物品真值。原云native默认保留，image仍为显式候选，标障/清除与运动时效分别验证。证据及原版Apache2源码快照见[候选收据](../../results/development/cloud_continuity_20261122/depth_image_candidate_20261124/audit.json)。

2026-11-21 C031复用本机Nav2 1.1.20原生[PositionGoalChecker](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_controller/plugins/position_goal_checker.cpp)和[PoseProgressChecker](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_controller/plugins/pose_progress_checker.cpp)。前者用于清扫端点，后者认可位移或转角进展；普通目标/充电仍保留完整朝向和独立实测停靠。两份普通恢复树来自同版`nav2_bt_navigator/behavior_trees`，仅显式补general checker ID，随包附[NAV2_LICENSE.txt](../../src/semantic_search/config/NAV2_LICENSE.txt)，不作为原创导航算法。

C031续查同版[ControllerServer](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_controller/src/controller_server.cpp)、[DWB](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_core/src/dwb_local_planner.cpp)及[NavFn](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_navfn_planner/src/navfn_planner.cpp)：到达检查以plan.back为目标、保留其时间戳，DWB用latest变换目标；NavFn容差允许以邻近可达点作为末点。当前候选让局部/behavior共用map并加入StaticLayer；.05m候选迟滞且仍卡住，已恢复原生Position .12m，原始odom/停稳屏障和实际锚点检查保留；副作用与待验证项见C031调查收据，不据上游机制直接归因全部失败。

清扫专用条带控制器复用本机[RotationShimController 1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_rotation_shim_controller/src/nav2_rotation_shim_controller.cpp)包装专用DWB实例，去掉清扫端点RotateToGoal，保留普通完整朝向DWB及全部障碍评分/限速。该版每次setPlan重查方向；采样短于.35m换道，显式退出.1rad；采样/TF/碰撞异常会回落DWB，不承诺始终先旋转。已完成配置/加载及完整实跑，覆盖仍未通过，收益与限制见交接及C031运行收据。

条带保持RoomSweep参考线并直接调用官方[FollowPath接口](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_msgs/action/FollowPath.action)，仍由同一ControllerServer及RotationShim+DWB执行；NavFn继续连接、入房和返航。半栅格路径核验与任务调度由本项目适配，未更换Dijkstra/DWB，不将规划线计入已清扫覆盖。新增FollowPath与原Navigate两动作共18项隔离ROS终态/新鲜停车协议已通过，物理收益继续实跑。

C031队列修复对照已安装[ros_gz_bridge 0.244.26 factory实现](https://github.com/gazebosim/ros_gz/blob/0.244.26/ros_gz_bridge/src/factory.hpp)，使用publisher QoS覆盖和消费端KEEP_LAST(1)，保留可靠性与.4秒停车门槛，未套用新版`qos_profile`配置。集成实例List/GetParameters已核实四类RGB-D输出depth=1，图中depth=0继续标未知。仿真出生/AMCL对照本机[Fortress 6.18.0 OdometryPublisher](https://github.com/gazebosim/gz-sim/blob/ignition-gazebo6_6.18.0/src/systems/odometry_publisher/OdometryPublisher.cc#L307)：当前/odom已包含world位姿，不再加出生平移；按地图绑定的恒定map/world关系应用于最新里程计，模式切换不回写充电位姿。证据集中`results/development/service_motion_20261121/`，短程收益不外推为覆盖或避障全部通过。

2026-11-21 C030停机后核对本机Humble包`nav2-controller/dwb-critics 1.1.20`及同版本官方[SimpleProgressChecker](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_controller/plugins/simple_progress_checker.cpp)、[RotateToGoal](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_critics/src/rotate_to_goal.cpp)、[SimpleGoalChecker](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_controller/plugins/simple_goal_checker.cpp)。进展检查只按XY位移重置，不计算转角；本机配置.5m/10秒与.35m换道/末端转向存在不匹配。当前真实短目标轨迹和失败日志支持该诊断；尚未记录控制器内部状态与同步速度链，不能认定RotateToGoal状态保持或深度过期解释了每一次失败，也未修改控制器参数。

2026-11-21 C028核查[Nav2 Humble StaticLayer源码](https://api.nav2.org/nav2-humble/html/static__layer_8cpp_source.html)，复用其滚动窗口与TF变换处理。局部配置原有static_layer段但未注册到plugins，候选仅补插件列表并通过真实GetParameters服务核实加载；240秒测试预算实跑覆盖19.31%、返航失败，未解决卡点；已逐字节恢复上一候选，旧参数和失败证据保留。本机Humble无rclpy.parameter_client模块，查询改用已有rcl_interfaces/GetParameters服务。

2026-11-21 C027受阻诊断补查Nav2 Humble官方[SimpleGoalChecker源码](https://api.nav2.org/nav2-humble/html/simple__goal__checker_8cpp_source.html)与[DWB RotateToGoal源码](https://api.nav2.org/nav2-humble/html/rotate__to__goal_8cpp_source.html)，核对进入位置窗口后的stateful目标检查和转向阶段状态保持。当前仅作为解释末端漂移/终态差异的后续诊断线索，尚未用本机库对照证实其为根因，也未修改控制器参数。C027任务层实测锚点校验是独立防止错误记账的修复，完整脱困与85%覆盖仍待验证。

2026-11-21 候选水瓶采用本机 [Assimp](https://github.com/assimp/assimp) 5.2.0 C接口转换GLB→OBJ，处理+Y到+Z轴向及嵌入纹理，原资产保留；转换在临时缓存中进行，OBJ漫反射近似不能完整复现PBR材质。部署依赖为Ubuntu的`libassimp5`。

视觉有限对照使用 [Ultralytics YOLO11](https://docs.ultralytics.com/models/yolo11/) 的m分割权重和 [YOLOE](https://docs.ultralytics.com/models/yoloe/) 的11s免提示分割权重，来自官方assets v8.4.0并核对发布页SHA256。权重与恢复入口见`resources/runtime.lock.json`；仅离线候选，未替换默认n。三张冻结原生图上均未解决冰箱漏检及水果混淆，不能据此认定更先进模型必然更适用。

C041追加16份真实视点重放：锁定YOLOE-pf能在两房蓝球的4个视点输出beach ball，另一个视点输出balloon，后者不能合并为sports ball；其他采样视点无球子类输出。这只支持继续验证球类辅助候选，不替代默认COCO检测器或正式误检验收。新增`ball_geometry.py`复用SciPy `least_squares`的鲁棒损失拟合真实掩膜点云球心，不读取场景资产半径/位置；半球噪声、平面拒绝与真实深度重放有证据，在线定位与重复稳定性尚待实跑。免提示模型沿用内建词表，不调用需要额外文本编码器的`set_classes()`。

2026-11-21 语义地图选型：按用户要求先复用有公开代码的方法与组件，再验证项目改进。已查阅下列官方仓库及安装说明；“方法参考”“组件接入”和“完整算法复现”分别记录。

| 上游 | 用途与适配判断 | 当前进展 |
|---|---|---|
| [Spark-DSG](https://github.com/MIT-SPARK/Spark-DSG) / [Python 发行包](https://pypi.org/project/spark-dsg/1.1.3/) | 复用 Hydra 的成熟场景图结构与 Python API，表达家庭、房间、物品及归属；发行包携带许可证 | 已在独立 `.venv-semantic` 安装 `1.1.3`，新增 `scene_graph.py` 适配感知记录；实际库的节点、边、证据与保存重载测试通过 |
| [ConceptGraphs](https://github.com/concept-graphs/concept-graphs) / [关联代码](https://github.com/concept-graphs/concept-graphs/blob/main/conceptgraph/slam/mapping.py) | 重点参考多视角对象融合、几何相似度与 CLIP 外观相似度联合关联；仓库采用 MIT 许可 | 已阅读代码和依赖；后续以现有 YOLO 掩膜、RGB-D 与 TF 对接，先建立适配基线。尚未复现其融合流程或接入 CLIP |
| [ConceptGraphs显示代码](https://github.com/concept-graphs/concept-graphs/blob/main/conceptgraph/scripts/visualize_cfslam_results.py) | 分类/实例着色与关系显示切换 | C041阅读实际代码后借鉴交互组织，现有Canvas按类别着色、区分历史/待核验/当前任务，并提供关系视图；未移植Open3D查看器或其关联算法 |
| [Canopy](https://github.com/Adyansh04/canopy) / [README](https://github.com/Adyansh04/canopy/blob/main/README.md) | SLAM地图上组合房间、物品、相机覆盖等语义图层 | C041借鉴同底图分层呈现，新增语义/运行视图及档案联动；未安装该项目，不宣称已经实现其全部功能或完整相机覆盖层 |
| [Hydra](https://github.com/MIT-SPARK/Hydra) | 参考对象—位置—房间的层级表示及增量场景图；当前主线官方测试环境为 Ubuntu 24.04 / ROS 2 Jazzy | 当前Humble工程已复用Spark-DSG；整套Hydra未接入 |
| [Clio](https://github.com/MIT-SPARK/Clio) | 参考任务相关的对象与区域粒度；官方 README 的在线运行仍采用 catkin / ROS 1，并提示发布版在线表现的回归问题 | 作为任务语义组织的研究参考，暂未安装或复现，不作为当前默认在线链路 |
| [SemExp](https://github.com/devendrachaplot/Object-Goal-Navigation) / [论文](https://arxiv.org/abs/2007.00643) | 参考“语义地图—目标选择—局部规划”的分层目标搜索；官方代码依赖旧 Habitat 与 PyTorch | 借鉴任务组织和对照设计；其策略权重尚未移植到 Gazebo，后续主动观察须在本工程验证 |
| [ETH ASL NBV](https://github.com/ethz-asl/nbvplanner) | Bircher 等 ICRA 2016，按未知区域增益选择下一视点、随观测重新规划；上游是 ROS Indigo/Jade、catkin 和 OctoMap | 参考信息增益与循环重规划思想，当前 `active_observation.py` 采用已有 RGB-D 可见性、占据地图和 Nav2 实现有界地面视点选择；未移植其 RRT、三维地图或飞行控制，实际效果正在验证 |

当前自有代码负责输入适配、持久化、任务与 UI 接入。空间关联仅作为基础对照。后续对照固定视点、语义记忆引导、几何与外观联合关联、覆盖和不确定性驱动补拍，比较漏计/重复计数、错误关联、定位误差、耗时、路程与显存。先验证收益，再确定创新结论；不把第三方算法或数据库保存称为原创方法。

导航保护复用已安装的 [Nav2 Humble CollisionMonitor](https://api.nav2.org/nav2-humble/html/collision__monitor__node_8cpp_source.html)，本机包版本 1.1.20，Apache-2.0。官方组件处理激光/点云的停车与接近减速；Humble 源码对过期来源调用 getData 后继续处理，本机隔离实测会继续放行。项目另加 `velocity_guard.py` 做墙钟、数据、TF 与指令时效检查，不能把其他 ROS 版本的失效停车说明直接当作本机行为。组合隔离测试已通过，当前仅显式候选启用；历史障碍子集已有实测，最终配置按总计划4项日常场景验收。

低矮障碍接触后核对 [ObstacleLayer](https://api.nav2.org/nav2-humble/html/obstacle__layer_8cpp_source.html) 和 [VoxelLayer](https://api.nav2.org/nav2-humble/html/voxel__layer_8cpp_source.html)，候选将激光二维层与深度三维层分开融合，保留 NavFn/DWB。[ObservationBuffer](https://api.nav2.org/nav2-humble/html/observation__buffer_8cpp_source.html) 会在缓存前按高度过滤点；深度 min-height 同时影响后续清除射线，因此障碍移除后的清除能力必须另测，不能从静态绕行通过推定动态恢复通过。

2026-11-21 物品扩展复用 [官方 YCB Object and Model Set](https://ycb-benchmarks.s3.amazonaws.com/index.html) 的 Google 16k 扫描：011_banana、013_apple、017_orange，数据许可 CC BY 4.0。网格、材质、纹理保留原样，项目生成静态 SDF 与署名说明；归档及 SDF 校验值记录在 `resources/assets.lock.json`。资源适配属于工程接入，识别、计数与场景兼容性另行验收。

2026-11-17系统复盘调研（仅查阅，未新增安装或替换运行依赖）：[Nav2 Humble Simple Commander迁移说明](https://docs.nav2.org/rolling/configuration_and_development/migration_guides/galactic/Galactic/)作为组合导航任务的参考；[Open Navigation Coverage](https://github.com/open-navigation/opennav_coverage/blob/main/README.md)提供Fields2Cover覆盖任务服务及Humble相关分支说明，室内覆盖适配仍需验证；[ConceptGraphs论文](https://arxiv.org/abs/2309.16650)及[官方代码](https://github.com/concept-graphs/concept-graphs)提供物体与关系场景图参考；[Goal-Oriented Semantic Exploration](https://arxiv.org/abs/2007.00643)提供语义地图辅助目标搜索的研究依据。本文引用不代表已经复现其算法；本项目的轻量语义记忆、主动核验与清扫观察协同方案见总计划P6，收益待实测。

2026-11-16 必做 cup 类资产适配：因原蓝杯在真实停靠视角下被识别为 bowl，新增 [GoogleResearch / Room_Essentials_Mug_White_Yellow](https://fuel.gazebosim.org/1.0/GoogleResearch/models/Room_Essentials_Mug_White_Yellow) v1，CC BY 4.0；来源、下载归档 SHA-256 与模型校验见 `resources/assets.lock.json` 的 `yellow_mug`，原件位于 `third_party/target_assets/yellow_mug/`，没有扩大类别白名单。当时资产准备总数为9；现已有12项目标来源，旧8/9项数字保留历史含义。Fortress 的 OBJ/MTL 相对纹理适配在临时缓存中完成，保留原版权/材质注释和原文件。

2026-11-16 项目技能：`skills/context-compression` 完整取自 [Agent-Skills-for-Context-Engineering](https://github.com/muratcankoylan/Agent-Skills-for-Context-Engineering) 的 `skills/context-compression/`，固定提交 `9ff0c99108b36d3b886defd825d8ac1f47dba374`；`skills/grill-me` 与必需依赖 `skills/grilling` 完整取自 [mattpocock/skills](https://github.com/mattpocock/skills) 的 `skills/productivity/` 下对应目录，固定提交 `5863806dd4e15c63174bee4e7515a4256792ecfa`。逐文件 SHA-256/Git blob、安装范围与会话使用边界见 `resources/project_skills.lock.json`，仓库许可证副本位于 `skills/upstream-licenses/`。未安装整套插件或其他可选技能；当前会话按已安装 SKILL.md 手工执行流程。

保留的第三方 Git 克隆位于 `third_party/`，可在相应目录中运行 `git log -1`、`git remote -v` 核对来源。上游文件在各自许可证下分发，本项目新增代码位于 `src/semantic_search`。

GitHub 仓库通过 `resources/runtime.lock.json` 与 `scripts/prepare_runtime.py` 恢复这些固定提交，不重复上传嵌套 Git 仓库及大体积资产。Fuel Chair 的原始来源元数据、归档 SHA-256 与署名也保存在该清单中，下载后恢复 `SOURCE.json` 和 `ATTRIBUTION.md`。语音和候选物品分别使用已有锁定清单与准备脚本。克隆部署流程见 [docs/GITHUB_DEPLOYMENT.md](GITHUB_DEPLOYMENT.md)。

2026-11-09 提前资源准备：`third_party/target_assets/` 暂存八个 Fuel 杯/瓶/碗/书/手机模型，逐项作者、版本、许可、URL 和 SHA-256 见 `resources/assets.lock.json` 及各模型 `SOURCE.json`，未加入运行场景。`third_party/realsense_ros_reference` 为 [官方 RealSense ROS 仓库](https://github.com/IntelRealSense/realsense-ros) `ros2-master` commit `7a87097c499e3b0b69518d54a8e7a9ac1924e87c` 的相机描述参考，保留 LICENSE/NOTICE，未构建驱动或替换相机。语音数据来自 [Systran/faster-whisper-small](https://huggingface.co/Systran/faster-whisper-small)，固定 revision 与文件校验值见 `resources/voice_model.lock.json`；在独立语音环境准备，未做录音/推理。详细状态见 `docs/maintained/RESOURCE_PREPARATION.md`。

新增 `third_party/aws_small_house`：[AWS Robotics / Small House World](https://github.com/aws-robotics/aws-robomaker-small-house-world)，`ros2` 分支固定提交 `ff9631ca6d1db9c1ba656498151464b5ab74aafe`。保留其全部原始资产和 LICENSE；根 LICENSE 为 Amazon 的宽松授权文本，而 package.xml 声明 Apache 2.0，两个原文件均保留供核对。不编译其 Classic 包；本工程读取 87 个静态实例，在临时文件中修复 Collada 照片路径、移除 Classic layer 元数据和不完整静态惯性，地板碰撞使用 Fortress 地平面，家具摆放和尺寸保留。来源清单及世界文件校验值见 `scenes/aws_house/source.json`，实例真值仅作评估。

| 本地目录 | 上游 / 分支 / 克隆时提交 | 许可证与项目内用途 |
|---|---|---|
| `third_party/tb3_fortress` | [Onicc/navigation2_ignition_gazebo_turtlebot3](https://github.com/Onicc/navigation2_ignition_gazebo_turtlebot3) · `main` · `115bcc30b0f5dc6bdfb918df64c94b88cf953657` | Apache-2.0。直接读取 Waffle SDF、网格、基础世界与 Nav2 参数。在本项目启动文件中生成派生场景，加 RGB-D、房间和搜索目标，采用当前系统的 `ros_gz_bridge`。原始上游文件保留原样。 |
| `third_party/turtlebot3_simulations` | [ROBOTIS-GIT/turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) · `humble` · `a35a56c8b04877dc89772b598084d8ce648a9023` | Apache-2.0。课程 Gazebo Classic 路线和原模型的对照源码。当前 Fortress 工程不编译这份仓库。 |
| `third_party/yolo_ros` | [mgonzs13/yolo_ros](https://github.com/mgonzs13/yolo_ros) · `main` · `94d2dd500cb26027ee9c019f4d3a02029ac615cc` | 仓库根目录为 GPL-3.0，`yolo_msgs/` 有单独的 MIT 许可证。当前只构建 `yolo_msgs`，复用 DetectionArray 接口。检测节点调用 Ultralytics Python API，避免将上游其余检测组件和当前 ONNX 工具链引入基础环境。 |

新增 `third_party/fuel_chair`：[OpenRobotics / Chair](https://fuel.gazebosim.org/1.0/OpenRobotics/models/Chair)，Fuel 版本 3，CC BY 4.0，作者 Cole Biesemeyer / Open Robotics。用于替换实测漏检的几何椅子。保留原资产；启动时修改实例名称、位置、静态属性和网格路径。下载元数据、SHA-256 和署名见目录内 `SOURCE.json`、`ATTRIBUTION.md`。

`src/semantic_search/config/nav2.yaml` 派生自固定版本 Onicc `waffle.yaml`（Apache-2.0），将目标检查器和 DWB 的 XY 到达容差统一为 0.12 m、朝向容差设为 0.10 rad，以提高相机观察点的可复现性。原上游配置保留。

早期AWS CPU配置使用`yolov8n-seg.pt`并按实际图像校准厨房视点，AWS NMS IoU=.30、基础场景=.50，检测置信度=.35。C013后的一键默认已改为GPU n/640，原CPU保留；最新默认与候选参数见[运行指南](RUNNING.md)，不把旧CPU配置写为当前默认。

Nav2、SLAM Toolbox、ros_gz 使用 Ubuntu 中已安装的 Humble 系统组件。应用通过 Nav2 的 `NavigateToPose` 发出目标，现有 Nav2 配置负责规划和控制。Ultralytics 运行库及权重的许可请以对应发行版本为准，首次下载的权重不在本目录。

C035的`map_resources.py`是项目文件资源适配层，复用现有PyYAML读取地图元数据、标准库SHA-256校验和Nav2 MapServer/MapSaver加载与保存栅格；没有新增建图算法。保留原图与兼容副本，未将文件ID冒充语义图物理身份。公共返航新增2秒发现预备窗口以缓解一次实测目标响应发送超时，保留原Action客户端及停车确认协议；单次复测通过不等于证明DDS根因已消除。

2026-11-17 数据链路适配参考 [ROS 2 Humble 官方 QoS 说明](https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)。本项目自行实现 `sensor_qos.py`，在保持其他 sensor-data 策略不变的前提下提供启动参数 `sensor_reliability`；默认保留 BEST_EFFORT，RELIABLE 候选需发布端支持相应策略。实际端点、同窗对照和候选链路时序分别保存在 `docs/evidence/qos_probe_20261117.json` 及开发验证记录中。没有替换系统 RMW、修改内核缓冲或引入新识别模型，也不把 QoS 配置视为原创感知算法。

课程任务书《设计任务及要求2026》、报告模板及三份《设计教程》位于 [docs/course_materials/](../course_materials/README.md)。课程教程以 Humble、TurtleBot3、Gazebo Classic 为例，第三份教程的任务顺序、YOLO 消息和 RGB-D 定位流程是本项目的设计参考。Fortress 与 Classic 的插件和启动方式不同，所以本项目另写了仿真入口。

研究背景可参考 [WoMAP](https://proceedings.mlr.press/v305/yin25b.html) 和 [OpenIN](https://ieeexplore.ieee.org/document/11091457/)。本项目借鉴语言指定目标和语义地图的任务设定，采用静态场景、固定类别、预设视点及简单跨帧融合；不复现论文算法。


C034复用上述Humble独立ObstacleLayer/VoxelLayer与RotationShim接入默认开发配置，原NavFn/DWB和普通完整朝向检查保留。Fortress接触诊断使用本机world原有Contact系统和ros_gz_bridge/ros_gz_interfaces；对照已安装`/usr/share/sdformat12/1.8/contact.sdf`，topic位于contact子项。首次误放sensor/topic的未收到消息记录保留，发现实际话题后补录实体支架—哑铃接触；最终共享通道以独立夹具校准且夹具移除。诊断不改变碰撞几何，排除持续轮地消息，不接收物品真值用于运行决策。

C037厨房两区巡视复用既有Nav2目标执行、RGB-D同步与逐视点诊断。inspection_views及find结束约束是项目房间配置/任务调度适配，位置与数量仍由实际感知产生；不是Spark-DSG内置巡视或新的导航算法。操作台摆放几何仅用于场景设计和事后评估，原AWS资源保留。


C040床头柜修复复用既有YOLO掩膜、RGB-D/TF与NumPy quantile，对杯/瓶深度做标准IQR异常值过滤后沿用原投影/尺寸先验；没有更换模型或新增识别算法库。提速沿用原NavFn、RotationShim和DWB，只同步开发控制器/平滑器线速度上限；相邻条带及确认停稳后续队列为项目调度实现，不称第三方原生能力或新导航算法。实际收益与限制见C040收据。

C041追加文本候选复用[Ultralytics YOLOE提示嵌入接口](https://docs.ultralytics.com/models/yoloe/#reuse-prompt-embeddings)，本机8.4.207的`save_prompt_embeddings/load_prompt_embeddings`实际用于19个通用类别词的冻结编码。基础模型与MobileCLIPTS来自官方assets v8.4.0并核验SHA，分词源码固定[ultralytics/CLIP提交](https://github.com/ultralytics/CLIP/tree/7ffa84b3bfa40c42ecc1c77147a855e69cb2dd40)。主S/n保持，文本辅助仅补书与球，分数/掩膜不变，其他类别输出忽略以免引入已观察到的苹果误识；同类重叠抑制由本项目适配。该文本分类能力来自上游模型，不等同于已经接入CLIP物理外观关联、完成全部词表类别或证明任务收益。新候选在线复测与原免提示失败分别记录，不直接重标balloon为球。

C041局部通行诊断复用Nav2 1.1.20的[ControllerServer速度限制订阅](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_controller/src/controller_server.cpp)及[SpeedLimit消息](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_msgs/msg/SpeedLimit.msg)：临时上限.12m/s，0用于移除该上限，不改控制器碰撞半径。Fortress 6.18的[Contact系统实现](https://github.com/gazebosim/gz-sim/blob/ignition-gazebo6_6.18.0/src/systems/contact/Contact.cc)仅在接触非空时发布事件，因此不能要求无接触心跳，也不能把空采样当碰撞验证通过。房间级附加跟踪余量开关、连续数据准备检查与证据汇总是项目诊断适配；未实现新导航算法，也未据此通过局部通行或整房覆盖验收。

C041点云连续性修复复用同项目已验证的独立ros_gz_bridge进程与Humble可靠QoS/KEEP_LAST(1)，扩展到普通开发启动；NumPy紧凑XYZ编码使用原clouds.sample_cloud及原采样密度。未更改Gazebo物理/控制算法、感知源时间戳或.25/.4秒门槛，也未宣称精确定位UDP/内核丢包根因；同场景前后时序对照、移动和GUI结果集中cloud_continuity_20261122。

C041定向残余补扫延用项目既有往复式条带、SciPy convolve增益/连通掩码及Nav2 FollowPath；本轮新增方向偏好和连续增益区间拆段，没有引入新覆盖规划库，也不将离线几何变化当实测提效。初次杂物间完整实跑的主路线完成达标，新补扫未触发。


C041全部碰撞体轮廓候选复用本机Nav2 Humble `dwb_critics::ObstacleFootprintCritic`（已安装default_critics.xml确认），按[官方Humble实现](https://api.nav2.org/nav2-humble/html/obstacle__footprint_8cpp_source.html)逐朝向检查轮廓边缘；两代价图统一显式footprint与.01 padding，未把原BaseObstacle的圆心检查当作多边形检查。全碰撞体包络提取、保守8点外包、参考覆盖与连接筛选分离由本项目适配；不是新的导航算法。几何来自实际SDF全部碰撞体，场景网格仅用于事后通道可行性审计，不用于运行选路。局部实走163格/停车回充通过不等同整房或4项日常避障验收。


C041低椅脚修整：深度高度下限在显式body候选从.08改.02，保留实际2–8cm点；原危险目标在现场cost99拒绝。局部诊断另复用官方Humble [PlannerServer::isPathValid完整轮廓分支](https://api.nav2.org/nav2-humble/html/planner__server_8cpp_source.html)校验单个目标朝向，不发动作，不将仅圆心cost<99等同于全身安全。前后对称轮廓/额外padding组合因背面接近及回充失败已撤回，原配置/原失败保留；当前只在原全部碰撞体外包/.01padding上验证低高度修复，未新写全局规划算法。


C041恢复安全修复复用现有Nav2 Wait行为与恢复树：将项目三份服务/连接树的ClearEntireCostmap替换为1秒Wait等待更新，原生重规划/Spin/BackUp与上限保留；不新增BT插件。关闭body候选原生VoxelLayer脚下清理开关，保留其他3D观测与射线清空，仅防止轮廓区域覆盖清除真实近身点。该组合来自实际低椅脚/杠铃接触、原生日志清图顺序与安全距离点云/代价图核对，不称已孤立证明唯一机制或通过全部日常避障。


C041全局/局部模型校准依据[Nav2官方footprint设置说明](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/footprint/setup_footprint/)：NavFn仍按栅格中心/代价工作，不因配置多边形就成为SE2完整轮廓规划器。显式候选global采用覆盖全部真实碰撞体的.40×.36m对称矩形，local保留真实8点完整外包，均padding .01；改变通行引导的保守程度，不缩小本地碰撞体，不把经验校准当路径可行性数学保证。旧同形和前后对称局部候选失败已固定，当前实跑见收据。


C041权重换算直接核实本机Humble头文件与[上游ObstacleFootprint](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_dwb_controller/dwb_critics/include/dwb_critics/obstacle_footprint.hpp)：该critic的getScale乘resolution，BaseObstacle继承基类直接用scale。原`.02`不能照搬为同量级；候选现`.4×.05=.02`，41条实际评估印证。硬拒绝不依此改变，两类原始分数不同，所以只称单位校正和本次实测通过，不称数学等价或保证性能因果。SmacLattice仅做本机安装/原语事实调查，未启动、未进入运行或算法比较，完整轮廓/纯转仍有待验证限制。


C041连续局部图候选沿用[Nav2 1.1.20默认局部odom/全局map配置](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_bringup/params/nav2_params.yaml)。[StaticLayer](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/plugins/static_layer.cpp)支持滚动局部图与静态map异帧；[恢复碰撞检查器](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/src/costmap_topic_collision_checker.cpp)直接按输入坐标查图，故body候选local_costmap与behavior_server必须配套使用odom。原生体素/射线/完整轮廓仍保留。旧map局部图近身历史格持续阻止轨迹已有证据，出生时的定位重影与地板误投尚未证实，不能把此次配置修整称唯一根因。


C041低趾/窗口修整：沿用Nav2原生rolling VoxelLayer（无新增传感器），显式body的local改6m窗口覆盖2.5m有效观测范围，padding .03按半格.025取整并核对左右失败夹具交集；global仍.01。这是保守设计选项，非已测误差上界。001足部实际碰撞网格低于.02、原点云未覆盖接触趾的事后审计见left_chair_sensor_audit；后时图重放不当作接触前验证。实际双椅.35回归通过，器械背面与完整覆盖仍按同配置实跑核对。


C041原地分层对照（static_layer_isolation）：机器人未移动，只禁用body候选local StaticLayer，命中格100→97，global静态与laser/depth保留；独立真实返航49.2秒/11.515m/接触0/停靠通过。现固化local static.enabled=false，以连续odom中的实测层负责近身碰撞，global map负责历史底图规划，复用[Nav2 1.1.20默认分层](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_bringup/params/nav2_params.yaml)（local活动列表不含StaticLayer）；不是删除原地图/清空实测障碍或新规划算法。其他低脚与完整覆盖仍按同配置实跑，不把该返回成功补作清扫覆盖。


11月23日只读候选审查核对Nav2 1.1.20 [Lattice参数/目标量化](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_smac_planner/src/smac_planner_lattice.cpp)、[NodeLattice碰撞采样](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_smac_planner/src/node_lattice.cpp)及安装5cm diff控制集SHA；16heading含cardinal、112原语含32原地，任意yaw与转动中间采样有限制。候选只准备未运行，不能当作本项目已实现或已验证收益。末态停滞重放沿用已安装[ObstacleFootprintCritic](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_critics/src/obstacle_footprint.cpp)的整数轮廓边缘算法；[原生BackUp/DriveOnHeading](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_behaviors/include/nav2_behaviors/plugins/drive_on_heading.hpp)从t=0检查轮廓，因此不能承诺从已重叠安全边缘自动倒出。图和JSON为冻结诊断，实际配置未减小padding。
