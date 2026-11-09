# 任务输入与结果接口

2026-11-09 更新。语言/语音模块后置；先用文本和结构化任务验证执行层。当前不调用大语言模型，`voice.py` 中的可选 Whisper 适配器尚未验收。

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

解析支持杯子和瓶子不等于这些类别已实测通过；“可乐”目前只能映射为 `bottle`，没有品牌/饮料种类识别。关键词文本解析和结构化 JSON 最终进入同一执行链路。

当前每次执行一个任务，支持取消，没有任务队列。不合法输入、忙碌或导航未就绪会在 ROS 日志中拒绝；当前并非完整的 ROS Action 任务接口，没有逐步反馈或每次拒绝的结构化应答。这些状态接口在完善执行功能时继续补齐。未来语音转写、规则解析或语言模型适配器只负责生成上述任务，导航和避障由执行层负责。

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

`completed` 目前表示配置的观察点执行完，`count` 是已确认的实例数；这不是任意房间“所有物品都已看见”的证明。后续需加入覆盖与遮挡状态，区分完整覆盖、部分覆盖和无法观察。

## 接口验证边界

本轮为结构化解析、字段校验和原文本兼容添加单元测试，连同既有回归共 18 项通过；未接入语音模型，也未据此重做物理场景的整套验收。历史 Gazebo / 任务三四证据见 `AWS_HOUSE.md`。
