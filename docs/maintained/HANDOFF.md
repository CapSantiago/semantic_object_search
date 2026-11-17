# 当前项目交接

## 当前可续接摘要（2026-11-17，C011）

**唯一总计划：[DEPLOYMENT_PLAN.md](DEPLOYMENT_PLAN.md)。当前为P3收尾；按最新指令已提前完成P4的一键启动和搜索UI首版。完整统一任务管理、任务一建图闭环和任务二清扫仍未完成。** 用户认可四项功能全部完成后再做语义记忆等创新；UI纳入正式开发。

### 当前目标与已确定方案

- 完成同一家庭机器人系统的语音建图、清扫、寻物、盘点；开发顺序P3→P4/G0→P4a/G1→P4b/G2→优化与创新。
- Ubuntu22.04/Humble/Fortress，NavFn Dijkstra+DWB，CPU YOLOv8n-seg、640输入、置信度.35、AWS IoU .30。不改原基线推进主要功能。
- 首批目标chair/cup/bottle；房间采用人工多边形、类别固定视点。场景真值只供设计和评估。
- UI首版为本地浏览器界面，已由 `bash scripts/start_robot.sh` 统一启动并自动打开；后续逐步接建图、清扫及语义查询。用户本轮明确授权实现后自行启动验证，先前停止实验的边界不再适用于这项工作。
- 语义地图、主动核验、清扫与观察协同是已认可的发展方向；模拟充电的课程表达待正式验收前核对。

### C011：五区配置与标注图

- 用户确认五区并授权配置坐标、观察点、UI、绘图skill图片置于总计划前。当前一键入口/解析/UI已切至 `rooms_aws_house_v4.json`，原四区v3未改；中部以y=1.0拆为下方客厅/上方健身区，其余覆盖范围不变，无新增墙体。
- 新增 `scenes/aws_house/spatial_v4.json`、`tests/test_five_zones.py`，UI从后端配置读取五区轮廓、名称及可选观察点。139测试通过、5项隔离ROS测试跳过；静态坐标检查通过，健身区3个视点Nav2实走通过；浏览器12项通过，证据在 `results/development/five_zones/`。
- 会话 `results/sessions/20261117T105354_590939Z_9f841a29` 已停止且shutdown无残留；健身区找椅子在第2视点途中随会话停止而canceled，不能记为寻物成功，停止信号来源未确定。此前已走通3点的独立导航记录不受影响。尚未完成五区完整感知/覆盖验收。
- 内置imagegen已生成 `docs/assets/house_five_zones.png` 并插入总计划开头，精确坐标图为 `house_five_zones_coordinates.png`。用户指出左侧蓝紫边界偏离挡板：实际模型RoomWall_01_001的y=-1.496180，程序边界y=-1.5（差约4mm），生成图标注偏移已修正，程序坐标保留。
- 修正版图片已置于总计划前，旧图片草稿仅留本地results；RUNNING/接口已更新。验证摘要并入现有 `docs/evidence/oneclick_ui_20261117.json` 的 `five_zone_update`，不新建重复报告。
- 用户要求后续语言准确、简洁、清晰，减少无用运行记录/重复文档，已加入AGENTS。当前正在提交并推送GitHub；实际发布状态以远端main核验为准。

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
- C009为结构维护；C010已按新授权恢复启动/UI集成实验。代码和文档尚未提交/推送。保存摘要不等同于执行宿主原生上下文压缩。

### C010：一键启动与控制台

- 新增 `scripts/start_robot.sh`、`robot_runtime.py`、`robot_monitor.py`、`robot_console.py` 及 `ui/{index.html,app.js,style.css}`；入口无需激活venv，默认打开网页、Gazebo和RViz，绑定127.0.0.1:8765，默认域66，每次独立IGN分区与 `results/sessions/<时间_ID>/`。
- 实际数据驱动就绪判定、map/TF显示、文字/选择寻物与统计、单任务互斥/幂等、取消、结果链接与本次历史已接入。静态地图保留原点朝向，过期位姿不显示为实时定位；不使用场景真值控制搜索。
- 两次GUI冷启动通过；第一会话传感器检查通过（scan约5Hz、RGB-D约8Hz）、厨房找杯子和计数均1且JSON/SVG已保存；第二会话取消后恢复寻找成功。浏览器宽屏/390px、无JS错误、刷新不重发任务已有证据。
- 修复启动后写盘失败遗留子进程、取消等待过长、关闭时未等待任务记录，以及Popen后进程身份暂不可读导致取消无效的竞态。进程清理只处理本会话token/UID/启动标识核验后的进程，浏览器不带此标记。
- 实际浏览器开始/停止按钮也已验证；取消2秒后的193个odom样本速度为0。两次运行均正常清理；结束时最终会话无所属残留进程、8765端口已关闭，普通浏览器可留在断线页面。
- 工程测试134通过、5项ROS客户端测试本轮跳过；没有重复运行P3正式矩阵。本轮证据和原始会话路径统一见 [oneclick_ui_20261117.json](../evidence/oneclick_ui_20261117.json)。原10项基线散列保持不变。
- 首版仍是现有搜索协议的适配器：无跨功能ROS Action/运动仲裁、实时导航步骤、麦克风、建图操作、清扫或跨运行语义历史。请求ID与核心task_id分别保存；不要再从外部终端向同域并发发任务。

俯视截图：`results/screenshots/house_top_20261117/house_top.png`，来自当前白黄杯家庭场景的Gazebo实际窗口；仅调整临时GUI相机，未修改正式场景或房间配置。

### 下一步执行顺序与完成标准

1. 使用一键入口启动当前候选，网页先确认传感器、定位和导航就绪；要跑矩阵时保持网页空闲并串行调用客户端，使用新的输出目录和明确冷热启动说明。
2. 补旧四区矩阵时显式使用v3；五区v4验证独立记录。继续补客厅瓶子5次、卧室15次，核对状态、逐视点数据、数量、JSON/SVG和跨批任务ID。即使通过也分别保留历史27次成功和启动失败，不能称连续45/45。
3. 检查新相机椅子兼容及原baseline/aws_house回归。已准备未运行脚本移为 `scripts/validation/legacy_regression.sh`，从项目根调用；不得覆盖旧输出。
4. 完善遮挡/邻房、冷启动、异常重复与定位标定；通过P3后补统一任务接口、UI阶段反馈和真实语音G0，再推进任务一/G1、任务二及整机/G2。

技能仍在 `skills/`，发现链接 `.agents/skills`保留。沿用context-compression增量维护本摘要、grill-me/grilling审视未决选择；目前没有必须向用户追问的关键决策。
