# 现成家庭机器人场景调研

部署更新：本页保留最初的候选调研记录；AWS Small House 已下载并接入，最新本机验证和启动指令见 [AWS_HOUSE.md](AWS_HOUSE.md)。下文“尚未接入”描述的是调研时点。

调研日期：2026-11-08。通过用户指定的 GitHub 插件检索仓库、读取 README/世界文件/依赖声明，并查阅 Gazebo Fuel 官方模型元数据及原作者预览。

**结论：优先采用 AWS Small House 的现成住宅布置，参考新 Gazebo 移植版接入当前工程；再增加独立的瓶子、杯子等任务物品。** 如果更重视住宅外观，可备选 Lake House。场景观感评价属于基于上游预览的判断。

本轮只完成检索、源码与预览核查。候选完整场景尚未下载、尚未接入本工程，也未在本机 Fortress 中启动。因此下面的“兼容性”是源码核对结果，不代表已经通过本项目的导航/识别验收。预览均来自上游，不是本项目新生成的场景。

## 候选对比

| 候选 | 已有内容与适用性 | 对当前 Humble / Fortress 的接入判断 |
|---|---|---|
| **AWS RoboMaker Small House / 新 Gazebo 移植版** | 完整住宅，厨房、客厅、卧室、车库；家具、家电、地毯、挂画等资源丰富；模型可独立摆放，适合目标编号和真值记录 | 原版 `ros2` 分支仍依赖 Gazebo Classic。已有 Jazzy/Harmonic 移植可参考，需把世界系统插件、资源路径等适配回 Fortress；推荐使用其世界和资产，继续使用本项目机器人与搜索链路 |
| **OpenRobotics Lake House + rt-net Humble 示例** | 有家具的双层住宅，带门廊、树木、地形，住宅整体展示效果好 | rt-net 的 `humble` 分支明确面向 Ubuntu 22.04 / ROS Humble / Ignition Gazebo 6，世界插件与当前版本一致；仍需接入自己的机器人、处理楼层/可通行区域、重新建图 |
| **RoboCup@Home Gazebo** | 在 AWS 住宅基础上提供 `storing_groceries`、`serve_breakfast` 场景，增加薯片罐、汤罐、芥末瓶、苹果、可乐罐等物品 | README/启动方式是 Classic，不能因分支名为 `ros2` 就认定兼容 Fortress。更适合复用任务物品和摆放设计 |
| **OOMWOO Gazebo living_room** | 面向家用清扫机器人，`living_room.world` 使用 26 个模型，有独立家具和道具资源 | 当前 README 是 Jazzy / 新 Gazebo，需适配 Fortress；作为客厅或家具来源有价值。其新增 `multi_room.sdf` 等采用简单 box/plane，不能仅凭多房间名称认为视觉丰富 |

## 1. 推荐：AWS Small House

- [官方原仓库，资产所在 ros2 分支](https://github.com/aws-robotics/aws-robomaker-small-house-world/tree/ros2)
- [官方世界文件](https://github.com/aws-robotics/aws-robomaker-small-house-world/blob/ros2/worlds/small_house.world)
- [新 Gazebo / TurtleBot 移植实例](https://github.com/pantelis/turtlebot-maze#aws-robomaker-worlds)
- [移植版住宅世界文件](https://github.com/pantelis/turtlebot-maze/blob/main/tb_worlds/worlds/house_world.sdf.xacro)
- [官方室内预览](https://raw.githubusercontent.com/aws-robotics/aws-robomaker-small-house-world/ros2/docs/images/gazebo_01.png)
- [新 Gazebo 移植版预览](https://raw.githubusercontent.com/pantelis/turtlebot-maze/main/media/world_house_gazebo.png)

原作者预览显示已有厨房橱柜、冰箱、灶台、餐桌、餐椅、沙发、地毯、吊灯和装饰；移植版说明有 68 个 `aws_robomaker_residential_*` 模型资源。这是资源类型/目录数量的描述，不能直接当成某一类别的场景实例数。

官方仓库已归档，默认 `archive` 分支只保留归档说明；原资源仍在 `ros1` / `ros2` 分支。`ros2` 分支检查到的提交为 `ff9631ca6d1db9c1ba656498151464b5ab74aafe`，`package.xml` 仍依赖 `gazebo_ros`、`gazebo`、`gazebo_plugins`，故不是当前 Fortress 的即用启动包。

官方 [PR #47](https://github.com/aws-robotics/aws-robomaker-small-house-world/pull/47) 是未合并的 Jazzy/Gazebo 8.9.0 适配，来源分支为 [sethgi / ros2_jazzy](https://github.com/sethgi/aws-robomaker-small-house-world/tree/ros2_jazzy)，检查到的提交为 `f9d16929d4a30a2e5c4d8348b9e4ad8847478a1e`。PR 说明修补了惯性字段和模型 link 名称等。它可作为迁移参考，不能当作已经支持 Fortress 的结论。

建议接入方式：

1. 固定所采用的模型版本和来源，整理本地 `models` 资源，避免演示时联网拉取。
2. 使用本项目已有的 Fortress Physics/Sensors/UserCommands/SceneBroadcaster 等世界系统；将住宅布置和独立模型接入该世界。
3. 继续使用当前 TurtleBot3、雷达、RGB-D 与 ROS 桥接；选择有效、无碰撞的出生点。
4. 核对网格材质、纹理和碰撞体，尤其是从 Classic 迁移的材质脚本。
5. 另加具有明确实例编号和位姿的瓶子、杯子；预训练模型能否正确识别必须看实际 RGB 图像验证。
6. 从真实雷达重新建图，重新划分房间并校准视点，完成任务三/四回归。

不要为获取住宅场景而直接搬入移植示例的整套 Jazzy、Docker、Zenoh、行为树和检测架构；场景资源与本项目机器人/任务代码可以分开复用。

许可记录：官方 `ros2/LICENSE` 给出允许复制、修改和分发的授权文本，但 `package.xml` 标记为 Apache 2.0，两者存在标签不一致。接入时应保留原始 LICENSE，并核对所用资产目录的单独声明，不在来源表中把所有内容简单归为同一种许可。

## 2. 备选：Lake House

- [Gazebo Fuel 模型元数据](https://fuel.gazebosim.org/1.0/OpenRobotics/models/Lake%20House)
- [rt-net Humble 世界文件](https://github.com/rt-net/raspimouse_sim/blob/humble/raspimouse_gazebo/worlds/lakehouse.sdf)
- [rt-net Humble 环境要求](https://github.com/rt-net/raspimouse_sim/blob/humble/README.en.md#requirements)
- [官方预览](https://fuel.gazebosim.org/1.0/OpenRobotics/models/Lake%20House/tip/files/thumbnails/LakeHouse.png)

Fuel 当前版本 3，元数据文件大小为 237,788,357 bytes，约 238 MB；许可证为 CC BY 4.0。rt-net 的 Humble 世界使用 `ignition-gazebo-*` 系统、Ogre2，并以 `0 0 -7.0 0 0 0` 位姿引入模型；这说明不能直接假定模型原点就是室内地面。

已读取官方 SDF：只有一个 model `house`、一个 link `base_link`，多个部件通过同一个 `house.dae` 的子网格引用组织。与 AWS 独立家具模型相比，单独移动、更换现有家具或逐实例记录真值更复杂。双层结构和室外地形也需要限制当前二维导航机器人的工作区域。

因此它适合住宅整体展示，但对于课程项目的“多个独立物品搜寻、编号和计数”，AWS 场景的模型组织更方便。必要时仍应另外放置独立任务目标。

## 3. 任务物品来源：RoboCup@Home

- [仓库](https://github.com/juandpenan/robocup_at_home_gazebo)
- [收纳食品场景](https://github.com/juandpenan/robocup_at_home_gazebo/blob/ros2/worlds/storing_groceries.world)
- [早餐场景](https://github.com/juandpenan/robocup_at_home_gazebo/blob/ros2/worlds/serve_breakfast.world)

源码抽样发现 `001_chips_can`、`002_master_chef_can`、`005_tomato_soup_can`、`006_mustard_bottle`、`007_tuna_fish_can`、`013_apple`、`coke_can` 等独立模型引用，比只有椅子的环境更接近家庭服务任务。

不能假定这些 YCB/食品模型共享仓库根目录的同一授权，也不能假定都能被当前 COCO 类别检测器准确识别；接入时应记录原模型来源、许可，并做逐类识别验证。该项目 README 还提示部分物体惯性参数不一致，当前只适合先评估静态摆放，不据此认定支持可靠抓取。

## 已筛除或降级的资源

- Fuel 中名为 `Apartment` 的模型，本次检查到的是公寓楼外观，不能仅凭名称认为它具有可搜寻的完整室内家具。
- `socrob/mbot_simulation_environments` 是 ERL 公寓，但 README 面向 ROS Kinetic / Gazebo 7，迁移和旧依赖成本较大，未列为首选。
- PAL `home.world` 可用于家具/操作环境参考，但仍有 Classic 世界结构及项目依赖，不比首选路线更直接。

本轮下载的只是元数据、SDF 文本和原作者预览，保存在 `results/scene_research/`；没有下载 238 MB 的 Lake House 完整模型，也没有修改当前运行场景和地图。
