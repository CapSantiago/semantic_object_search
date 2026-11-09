# 提前资源部署记录

更新：2026-11-09。用户授权提前下载/部署后续必要资源。本轮进行下载、解包、版本固定、文件校验、软件安装和依赖导入完整性检查，**未运行 Gazebo、导航比较、识别推理、录音或训练实验**。主线开发顺序见 [DEPLOYMENT_PLAN.md](DEPLOYMENT_PLAN.md)。

本页“已准备”描述当前开发机状态。GitHub 只保存源码、锁定清单和精选证据，不包含 `.venv*`、下载缓存、原生库及模型数据。新机器先按 [GITHUB_DEPLOYMENT.md](GITHUB_DEPLOYMENT.md) 恢复运行资源；离线重建需要另行复制 `downloads/`，仅克隆仓库不能直接使用 `--offline`。

## 已准备内容

| 内容 | 位置与版本 | 当前边界 |
|---|---|---|
| 8 个目标模型 | `third_party/target_assets/`；锁定清单 `resources/assets.lock.json` | 已下载、解包、校验归档与 SDF；没有插入现有场景，没有识别/碰撞验收 |
| 官方相机描述参考 | `third_party/realsense_ros_reference/realsense2_description/`；commit `7a87097c499e3b0b69518d54a8e7a9ac1924e87c` | 已取回模型、URDF、LICENSE/NOTICE；没有替换当前相机、没有构建真实相机驱动 |
| CPU 语音环境 | `.venv-voice/`；`requirements-voice-ubuntu-cpu.txt` 全量固定版本 | 与 `.venv` 视觉环境分离；软件依赖可导入，麦克风和识别质量未验收 |
| Whisper small 离线数据 | `resources/models/faster-whisper-small/`；HF revision `536b0662742c02347bc0e980a01041f333bce120` | 约 464 MiB，数据文件校验通过；没有实例化模型或执行推理 |
| 原生音频库 | `resources/native/`，Ubuntu `libportaudio2` 19.6.0-1.1 | 官方 deb 在项目内解包，由 voice 环境加载，系统包数据库未修改 |
| 导航候选插件 | 系统已有 Smac、MPPI、RPP，Nav2 1.1.20 | 只核对安装版本，没有更换 Dijkstra+DWB，也没有比较结果 |
| 语音离线 wheels | `downloads/voice_wheels/` | 缓存固定版本依赖，用于相同 Ubuntu/Python 平台重建 |

语音环境关键版本：faster-whisper 1.2.1、CTranslate2 4.8.5、sounddevice 0.5.6、NumPy 1.26.4。完整闭包以锁定文件为准。语音模型为 `Systran/faster-whisper-small` 的 CTranslate2 数据；来源、固定 revision 和文件 SHA-256 在 `resources/voice_model.lock.json`。其原始模型卡保留在本地模型目录。

本机音频包镜像下载出现 TLS 故障，且没有无密码 sudo；已改为从 Ubuntu 官方软件池下载并在项目中解包。依赖导入检查在实际主机环境通过，未请求管理员密码，未改变系统音频配置。

## 目标模型明细

| 模型目录 | 对应候选类别 | 固定版本 |
|---|---|---|
| `plastic_cup` | cup | OpenRobotics Plastic Cup v3 |
| `blue_mug` | cup | GoogleResearch Cole_Hardware_Mug_Classic_Blue v1 |
| `mustard_bottle` | bottle | Gambit Mustard Bottle v2 |
| `water_bottle` | bottle | iche033 Water Bottle v3，GLB 适配待核对 |
| `bowl` | bowl | OpenRobotics Bowl v4 |
| `patterned_bowl` | bowl | GoogleResearch Cole_Hardware_Bowl_Scirocco_YellowBlue v1 |
| `book` | book | GoogleResearch Eat_to_Live…Book v1 |
| `mobile_phone` | cell phone | OpenRobotics Samsung J8 Black v9 |

8 个模型对应 5 个类别的资产，不是 8 个已可调用类别。现有 AWS 里的椅子、花瓶、电视、冰箱、沙发、床和餐桌可作为其他候选来源，准入顺序见 [TARGET_CATALOG.md](TARGET_CATALOG.md)。Google 扫描模型、GLB 和 Fuel 模型的 URI、尺度、材质及碰撞仍需后续适配；不能把下载完成称作已兼容验收。

## 检查与重建

在项目根目录执行完整性检查，只读取文件和已安装包元数据，不启动功能实验：

```bash
python3 scripts/verify_resources.py
```

本轮额外使用 `--check-baseline` 对照准备前的文件散列，确认原导航参数、场景注册、AWS 房间、机器人启动定义、视觉依赖、既有地图与当前视觉权重未变。该快照只用于本次准备操作的隔离检查；后续有意开发相机时，不应把它当作禁止功能修改的永久规则。

资源缺失时按固定清单重取目标资产：

```bash
python3 scripts/prepare_target_assets.py
```

在已有 Python 3.10、pip 和 ROS 基础系统的同平台机器上，可从已缓存文件重建独立语音环境：

```bash
bash scripts/prepare_voice.sh --offline
```

去掉 `--offline` 可从固定来源补下载缺失资源。虚拟环境绑定机器和路径，换目录/机器时应重建；这里没有缓存整套 Ubuntu/ROS 系统安装包。相机参考源码的固定版本和角色在 `resources/camera_reference.lock.json`，不需要在当前工作空间构建它。

`scripts/voice.sh` 是为 P4 准备的本地离线模型启动入口，执行后会加载识别模型并进入录音流程，**本轮没有执行**。`voice_env.sh` 只作用于调用它的语音终端，不改现有视觉启动方式。

## 完整性记录与后续工作

`resources/preparation_status.json` 保存准备检查结果，`downloads/voice_dependency_check.log` 保留依赖导入记录。检查成功仅表示文件/软件准备就绪，以下工作仍未开展：

- 将新增物品布置到指定房间/支撑面、扩展运行白名单并验收。
- 将参考相机改造成完整仿真传感器组件，验证多点视野、碰撞和覆盖。
- 麦克风录音、真实语音识别和人工端到端任务验证。
- 新导航/视觉算法比较、训练和默认算法切换。

后续按总计划 P1 开始开发，不能跳过当前配置下的任务书完整验收 G0。
