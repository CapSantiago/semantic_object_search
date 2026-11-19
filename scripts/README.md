# 当前工程脚本

在项目根目录调用这些脚本，当前候选的完整顺序见 [运行指南](../docs/maintained/RUNNING.md)。

| 用途 | 入口 |
|---|---|
| 日常一键启动 | `start_robot.sh`，打开本地UI、Gazebo和RViz；Ctrl+C退出 |
| 一键入口内部模块 | `robot_console.py`、`robot_runtime.py`、`robot_monitor.py` |
| 环境与构建 | `env.sh`、`build.sh` |
| GPU视觉环境 | `prepare_gpu.sh`安装独立`.venv-gpu`；一键默认GPU，`--device cpu`回退 |
| 候选仿真/导航/搜索/视觉 | `development.sh`、`development_navigation.sh`、`development_search.sh`、`development_detector.sh` |
| 原场景运行 | `simulation.sh`、`navigation.sh`、`search.sh`、`rviz.sh` |
| 建图和地图保存 | `mapping.sh`、`mapping_navigation.sh`、`save_map.sh`、`drive_waypoints.py` |
| 任务与验收 | `run_search.py`、`run_search_matrix.py`、`validate_sensors.py`、`summarize_tabletop_validation.py` |
| 感知与异常验收 | `validation/capture_observation_views.py`、`validation/visibility_coverage.py`、`validation/audit_search_results.py`、`validation/fault_recovery.py`、`validation/occlusion_recovery.py`；在独立验收批次运行 |
| 抓帧/路线/诊断 | `capture_rgbd.py`、`navigate_route.py`、`record_navigation_state.py`、`validate_spatial_plan.py` |
| 资源与语音准备 | `prepare_runtime.py`、`prepare_target_assets.py`、`prepare_voice.sh`、`voice.sh`等 |
| 原场景回归 | `validation/legacy_regression.sh`，显式调用且使用新输出；不是自动启动步骤 |

旧布局标定、视点评估和固定验收汇总脚本已移至 [archive/tools](../archive/README.md)。后续一次性实验脚本随实验结果保存；通用维护工具才保留在此目录。
