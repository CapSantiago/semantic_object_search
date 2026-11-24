# 当前工程脚本

在项目根目录调用这些脚本，当前候选的完整顺序见 [运行指南](../docs/maintained/RUNNING.md)。

| 用途 | 入口 |
|---|---|
| 日常一键启动 | `start_robot.sh`，打开本地UI、Gazebo和RViz；Ctrl+C退出 |
| 一键入口内部模块 | `robot_console.py`、`robot_runtime.py`、`robot_monitor.py`；`navigation_client.py`共用Navigate/FollowPath终态与新鲜停车屏障 |
| 环境与构建 | `env.sh`、`build.sh` |
| GPU视觉环境 | `prepare_gpu.sh`安装独立`.venv-gpu`；一键默认GPU，`--device cpu`回退 |
| 候选仿真/导航/搜索/视觉 | `development.sh`、`development_navigation.sh`、`development_search.sh`、`development_detector.sh` |
| 原场景运行 | `simulation.sh`、`navigation.sh`、`search.sh`、`rviz.sh` |
| 建图和地图保存 | `mapping.sh`、`mapping_navigation.sh`、`save_map.sh`、`drive_waypoints.py` |
| 任务与验收 | `run_search.py`、`run_search_matrix.py`、`run_cleaning.py`、`run_return.py`、`validate_sensors.py`、`summarize_tabletop_validation.py` |
| 感知与异常验收 | `validation/capture_observation_views.py`、`validation/visibility_coverage.py`、`validation/audit_search_results.py`、`validation/fault_recovery.py`、`validation/occlusion_recovery.py`；在独立验收批次运行 |
| 抓帧/路线/诊断 | `capture_rgbd.py`、`navigate_route.py`、`record_navigation_state.py`、`validate_spatial_plan.py`；`validation/motion_profile.py`只读同步采样速度/guard/TF与失败代价图；`validation/dwb_scores.py`只读记录真实DWB最优/前进候选及各critic评分，标明提前终止的不完整代价；`validation/localization_refresh_trial.py`为一次停稳激光更新诊断，不是默认查找行为 |
| 资源与语义/语音环境 | `prepare_runtime.py`、`prepare_target_assets.py`、`prepare_semantic.sh`、`prepare_voice.sh`；`voice.sh`开始实际录音 |
| 安全保护与多类筛查 | `validation/navigation_safety_isolated.py`、`validation/navigation_safety_trials.py`、`validation/obstacle_avoidance.py`、`validation/survey_objects.py`；专项诊断不等于正式通过 |
| 原场景回归 | `validation/legacy_regression.sh`，显式调用且使用新输出；不是自动启动步骤 |
| 答辩素材打包 | `package_demo_materials.py --output <新目录>`，从现有审计生成8张16:9页、离线目录、原图视频及冻结会话散列；拒绝覆盖，不启动机器人，不能代替任务验收 |

避障按[总计划的4项日常场景](../docs/maintained/DEPLOYMENT_PLAN.md#局部避障验收矩阵)验收，已有专项记录保留；只读物品筛查不写入正式记忆或提升类别准入。

`validation/audit_moving_yield.py`提供已保存原生数据的停让旁证核验函数：排除目标点自然停车，核对新鲜实际路径、占据变化、提前停车与续行；不要求机器人靠近障碍。它不替代任务正确性、接触、全时效或回充检查，真实正反例回归见`tests/test_moving_yield_audit.py`。

旧布局标定、视点评估和固定验收汇总脚本已移至 [archive/tools](../archive/README.md)。后续一次性实验脚本随实验结果保存；通用维护工具才保留在此目录。
