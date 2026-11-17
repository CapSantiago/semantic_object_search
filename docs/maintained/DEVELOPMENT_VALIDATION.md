# 当前验证概览

本页维护主要结论和证据入口，完整过程放入 [历史开发记录](../archive/validation/DEVELOPMENT_VALIDATION_20261117.md)。当前执行顺序见 [总计划](DEPLOYMENT_PLAN.md)，运行命令见 [RUNNING.md](RUNNING.md)。

## 已有结果

| 验证项 | 结果 | 证据 |
|---|---|---|
| 原baseline与AWS场景 | baseline计数2椅，AWS厨房找1/计数4；建图和传感器已实测 | [基础场景](../archive/validation/VALIDATION.md)、[AWS场景](AWS_HOUSE.md) |
| 三房间桌面首轮 | 原640配置下，杯子寻找和杯/瓶计数共9例通过 | [首轮集合](../evidence/tabletop_single_20261116.json) |
| 厨房双杯 | 5次计数均为2，编号与产物核验通过 | [双杯证据](../evidence/tabletop_two_20261116.json) |
| 卧室v3初轮 | 15次warm任务通过；同位置转向不代表全房覆盖 | [重复记录](../evidence/bedroom_repeats_20261117.json)、[到达复核](../evidence/bedroom_v3_review_20261117.json) |
| 三房间旧45例 | 44通过、1次partial；75次导航成功，包含5次进度恢复 | [矩阵](../evidence/tabletop_matrix_20261117.json)、[导航](../evidence/final_navigation_20261117.json) |
| 有界补等与异常 | 短断流恢复、永久断流反馈、无效深度及恢复已有实测；补等卧室15例通过 | [异常恢复](../evidence/observation_recovery_20261117.json)、[15次重复](../evidence/bedroom_bounded_wait_20261117.json) |
| 可靠订阅候选 | 传感器检查通过；观测后15秒检测120/120与RGB/depth/info匹配 | [链路收据](../evidence/reliable_sensor_timing_20261117.json) |
| 可靠输入任务批次 | 27通过；第28次发送前发现失败，后17例未执行，整体aborted | [失败收据](../evidence/client_discovery_failure_20261117.json) |
| 最新客户端修复 | 发现等待、旧结果拒绝、超时/中断测试通过；工程66项+隔离ROS5项通过 | [修复收据](../evidence/client_discovery_fix_20261117.json) |

完整任务三/四的语音和人工验收仍待完成；当前G0/G1/G2未通过。搜索和计数返回的是本次已确认结果，观察覆盖仍需补齐。

## 当前候选配置

- 单白黄杯布局 `scenes/aws_house/layout_r0_yellow_v1.json`；类别chair/cup/bottle。
- 房间 `src/semantic_search/config/rooms_aws_house_v3.json`；相机1m/.25rad。
- NavFn Dijkstra+DWB及 `nav2_depth.yaml`；CPU YOLOv8n-seg，640/.35/.30。
- detector/search显式 `sensor_reliability:=reliable`；名义停留4秒，零有效帧时最多补等4仿真秒。

不同批次代码和QoS版本见各JSON中的散列，旧结果不回写为新版本通过。

## 后续验证

先补客厅瓶子和卧室20例，再进行新相机椅子兼容、原场景回归、遮挡/邻房、独立冷启动、异常重复与定位标定。预期数量仅用于结果评估；输入感知和搜索的数据不使用场景真值。

最新运行结果仍在 `results/development/20261117_reliable/`；用户手工结果在 `results/development/manual_demo/`。旧已结束批次和日志已压缩，按实验原路径读取的方法见 [归档说明](../../archive/README.md)。精选证据留在 `docs/evidence/`。

正式案例至少重复5次，记录配置、初始位姿、冷热启动和全部失败；不能把客户端协议测试当作机器人感知或语音验证。


## C010：一键启动和UI集成（2026-11-17）

本轮只推进启动/操作入口，沿用原导航与视觉候选。两次独立GUI冷启动均进入实际传感器/定位/感知就绪；Gazebo、RViz桌面窗口及自动打开的Firefox页面可见。首轮10秒传感器检查通过：scan约5Hz、RGB-D约8Hz、有效深度约69.9%，三段TF均存在。厨房找杯子和统计均为1，JSON、观测文件、编号SVG保存并经HTTP读取。

最终版验证单任务忙碌拒绝409、取消回执、取消后恢复寻物；随后从真实浏览器选择“统计客厅瓶子”点击开始，约5秒后点击停止，约0.5秒收到canceled。取消2秒后的193个odom样本线速度和角速度均为0（观察持续至取消后约5.84秒）。这是一轮仿真停车实测，不外推为任意场景安全认证。

Firefox真实宽屏和390px视口检查通过：地图/位姿/标记正常、无JS错误、刷新只发送GET、忙碌禁用新提交、停止按钮可见。两次退出均无会话残留进程；最终网页端口释放。工程测试134通过、5项隔离ROS客户端测试按开关跳过，新增控制台/运行管理/监测相关68项已包含在总数中。根目录无范围pytest曾误收third_party未配置测试；改用项目tests/后通过。10项受保护基线散列一致。

完整收据、参数、源码指纹和会话目录见 [oneclick_ui_20261117.json](../evidence/oneclick_ui_20261117.json)。本轮不修改旧45例/可靠输入27例统计，不宣称全房覆盖、语音G0或建图/清扫验收通过。


C011五区更新：v4静态连通/不重叠检查通过，健身区三个观察点Nav2实走通过，UI五区/标签/观察点及390px显示共12项检查通过，工程139项通过、5项跳过。健身区寻椅在会话停止时取消，未记为识别成功；五区完整感知覆盖仍待验证。摘要并入现有 [一键入口证据](../evidence/oneclick_ui_20261117.json) 的 `five_zone_update`，原四区记录保留。
