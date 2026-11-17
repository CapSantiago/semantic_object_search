# 历史工具与运行归档

本目录用于保持日常工程目录简洁。当前运行入口在 `scripts/`，现行文档在 [docs/maintained](../docs/README.md)。

## 历史工具

`tools/scripts/`保存6个旧专项工具：布局测量、场景清点、候选相机视点评估、捕获图像评估及两套固定旧验收汇总；`tools/tools/mesh_bounds.cpp`是它们的配套源码。原代码未改写，原路径及散列见 [整理清单](../resources/maintenance/reorganization_20261117.json)。

这些工具可能按原文件层级计算项目根路径。需要复现实验时，先在独立项目副本中恢复到清单的原路径，再按原记录执行；不要直接在归档目录执行或写回旧验证结果。通用抓帧、路线、传感器、矩阵和结果审计工具仍在 `scripts/`。

## 本机运行数据

- `runs/development_completed_20261117.tar.gz`：已结束的开发批次、旧故障注入脚本和观察器。
- `runs/logs_20261117.tar.gz`：整理前的构建及ROS日志。
- 每个包旁的 `.manifest.json`记录所有原始路径、文件SHA-256、符号链接与包校验值；压缩包逐文件验证后才清理原目录。

压缩包是本机归档，不上传Git；精选证据和归档清单随源码保留。其他机器如需完整原始记录，应另行复制这些包。最新 `results/development/20261117_reliable/`、用户 `manual_demo/` 和已跟踪的原场景验收数据继续保留原位。

在项目根目录查看包内容：

```bash
tar -tzf archive/runs/development_completed_20261117.tar.gz
```

读取某一个旧JSON，不需要解压全部数据：

```bash
tar -xOzf archive/runs/development_completed_20261117.tar.gz   results/development/20261117_bedroom/final_matrix/report.json
```

恢复时先解压到单独的临时目录，避免覆盖新结果：

```bash
restore_dir=$(mktemp -d /tmp/semantic_history.XXXXXX)
tar -xzf archive/runs/development_completed_20261117.tar.gz -C "$restore_dir"
printf '%s\n' "$restore_dir"
```

废弃 `build/`、`build_ascii/`和项目Python/pytest缓存已删除，可由构建或测试再生成。当前 `install/`、实际外置构建目录、依赖环境、资产、地图及权重保留。
