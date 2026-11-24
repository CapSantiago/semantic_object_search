# 运行资源与部署状态

更新：2026-11-24。本页维护资源位置、恢复方法和适配状态；功能完成以[验证概览](DEVELOPMENT_VALIDATION.md)为准，优先级以[总计划](DEPLOYMENT_PLAN.md)为准。资源锁是版本与校验值的依据。

## 当前已准备内容

| 内容 | 位置与版本依据 | 已有进展与待验收内容 |
|---|---|---|
| 12项目标资源 | `third_party/target_assets/`；`resources/assets.lock.json` | 杯、瓶、碗、书、手机及YCB水果已入候选布局；正式仍三类，其余按常用区域验收 |
| AWS住宅及原baseline | `third_party/`；`resources/runtime.lock.json` | 家庭场景、传感器与三类基线已实测，原场景/历史地图保留 |
| 文件底图注册与校验 | `config/map_resources.json`、`semantic_search/map_resources.py` | C035统一主图/兼容选择、导航与充电绑定及独立新版本保存；冷建图恢复/感知记忆回归通过，原12份地图未改；7类空间视图及真实DSG关系图统一导出已实现；局部代价图UI仍待接入 |
| 视觉环境与权重 | `.venv`、`.venv-gpu`；`requirements-vision-ubuntu-*.txt`、`runtime.lock.json` | 默认GPU n/640、CPU保留；s与YOLOE免提示已有候选实跑，YOLO11m仅离线，文本YOLOE书/球辅助为显式候选，碗书所选布局通过，更多布局与重复暂缓 |
| 语义场景图库 | `.venv-semantic`；`requirements-semantic.txt` | Spark-DSG1.1.3接入并通过真实任务保存/重载；厨房杯移动单次现场核验及SQLite/真实DSG重载通过；合格查找收益仍未验收 |
| 相机组件与描述参考 | 项目SDF/TF；`resources/camera_reference.lock.json` | 1m/.25rad组件已有回归，原生宽视场为候选；RealSense参考未作为真实硬件驱动启用 |
| 离线语音环境与Whisper small | `.venv-voice`、`resources/models/faster-whisper-small/`；语音依赖及模型锁 | 合成音频已完成转写和任务测试，真人麦克风仍最后联调 |
| 音频库与离线wheels | `resources/native/`、`downloads/voice_wheels/`；对应锁文件 | 支持同平台恢复；新机器需重新创建虚拟环境 |
| 导航与运动保护 | ROS Humble组件及项目配置 | 继续使用Dijkstra+DWB；CollisionMonitor/时效保护为候选，最终配置按4项日常场景验证 |
| 固定充电设施 | `config/charging_station.json`及development内置几何 | 地图双SHA、固定设施/出生/公共返航已接入；五区与建图恢复后回充已有实测。清扫覆盖与条带验收另见验证概览，不与充电设施准备状态混用 |
| 导航比较候选 | 已安装Smac、MPPI、RPP | 算法比较暂缓，当前版本完成后再评估 |

12项资源包含多个同类外观，不等于12个正式通过类别；住宅家具继续复用原资产，16类保留为后续扩展目录，不作答辩前提。源版本、许可及散列见各锁文件和[SOURCES.md](SOURCES.md)，逐类进度和验收区域见[TARGET_CATALOG.md](TARGET_CATALOG.md)。

## 目标资源与适配

- 杯：`plastic_cup`、`blue_mug`、`yellow_mug`；原蓝杯混淆和塑料杯漏检记录保留。
- 瓶：`mustard_bottle`、`water_bottle`；水瓶GLB已通过Assimp5.2.0临时转OBJ、轴向与纹理适配，并在Fortress渲染。原GLB保留，OBJ漫反射近似不能完整复现PBR。
- 碗、书和手机：`bowl`、`patterned_bowl`、`book`、`mobile_phone`；已入候选场景；碗/书所选布局寻找与计数通过，更多变化布局/重复暂缓，手机仍按原候选状态。
- 水果：`ycb_apple`、`ycb_banana`、`ycb_orange`；官方Google16k扫描、派生SDF及CC BY4.0署名已锁定，苹果/橙子混淆仍待处理。
- 原住宅提供椅子、电视、冰箱、沙发、床、餐桌、花瓶和球等。哑铃等后续类别按总计划另行评估。

当前功能联调默认使用v6/service_surfaces_v2：厨房餐桌/操作台、卧室书桌/床头柜；原v4/v5与各布局保留。v5→v6只改观察路线，不改房间几何，复用v5语义namespace；v4历史namespace仍保留。答辩预检显式使用service_bowl_book_v1/v14、S1280/1280×960和书/球辅助；默认配方不自动切换。显式image点云候选每启动独立校准后使用既有深度图，原生点云默认保留，相关时效范围见验证概览。相机、目标、地图和图层共用当前场景版本；各类地图输出按[统一底图方案](DEPLOYMENT_PLAN.md#地图数据与输出视图)派生。

## 检查与恢复

新机器先按[GITHUB_DEPLOYMENT.md](GITHUB_DEPLOYMENT.md)准备系统依赖并执行`bash scripts/deploy.sh`。GitHub保存源码、锁文件和精选证据，虚拟环境、缓存与权重需恢复；当前未发布改动及部署验证版本见交接。

```bash
# 核心资源与已准备的候选模型：只核验，不联网
python3 scripts/prepare_runtime.py --check
python3 scripts/prepare_runtime.py --check --model yolov8s-seg.pt
# 按需要恢复某个布局的资源
python3 scripts/prepare_target_assets.py --layout scenes/aws_house/layout_objects_v3.json
# 独立语义环境
bash scripts/prepare_semantic.sh
# 同平台且已复制离线缓存时，重建语音环境
bash scripts/prepare_voice.sh --offline
```

`prepare_runtime.py --model <锁定模型名>`会恢复所选候选权重，在线启动目前只支持n/s。`prepare_target_assets.py --asset <ID>`可单独准备目标。去掉语音脚本的`--offline`可下载固定来源的缺失文件；离线部署需另行复制缓存，虚拟环境须在目标机器重建。

`scripts/verify_resources.py`用于文件/依赖完整性核验，其准备阶段状态字段不代表当前功能验收。`--check-baseline`对照的是早期准备快照，仅适合追溯当时操作。`scripts/voice.sh`会开始实际录音，本轮文档维护不启动它。

## 历史准备记录

2026-11-09最初准备8项Fuel资源，11月16日加入白黄杯成为9项，11月21日加入3种YCB水果成为12项。`resources/preparation_status.json`及`downloads/voice_dependency_check.log`保留早期准备结论；当时未运行仿真、识别或录音，后续功能实测见验证概览。

早期本机音频下载曾遇TLS故障且无免密sudo，随后从Ubuntu官方软件池下载并在项目内解包，未修改系统音频配置。参考相机源码与课程原件保留，当前不需要编译真实RealSense驱动或切换Gazebo发行版。

C041文本辅助使用官方`yoloe-11s-seg.pt`（27,803,986字节，资源锁SHA256）与仓库内`resources/ball_prompts_v1.npz`（37,093字节、模型绑定与SHA核验）。词表及来源见`resources/vision_prompts_v1.json`，通过Ultralytics官方`save_prompt_embeddings/load_prompt_embeddings`复用。恢复命令为`python3 scripts/prepare_runtime.py --model yoloe-11s-seg.pt`，`--check`仅核验。600MB `mobileclip_blt.ts`用于离线编码开发，已按官方发布SHA校验并单独忽略Git；正式推理直接加载冻结向量，不依赖编码器、CLIP分词包或联网。开发分词包和固定CLIP提交、源码archive散列及过程失败保留在集中证据/recipe；没有改变主Torch或Ultralytics版本，也没有训练新模型。类别词与模型可输出标签不代表已通过端到端验收。


C041显式body候选新增`src/service_navfn`，依赖现有Nav2 Humble的`nav2_navfn_planner`、`nav2_costmap_2d`、`pluginlib`开发头文件/库，不增加模型、场景或地图资产。使用同一`bash scripts/build.sh`构建三包；原Nav2算法和原地图资源保留。原生适配的真实收益以HANDOFF实跑记录为准。


2026-11-23清除修复不增加模型/资产或依赖：复用既有NumPy、sensor_msgs与Nav2体素层。仿真启动从真实相机clip/far显式配置独立清除流，默认关闭无标定补射线；改动后运行同一build.sh。旧标障输入及历史配置/数据保留；当前清除与连续导航通过不代替重复计数稳定性验收，见HANDOFF。
