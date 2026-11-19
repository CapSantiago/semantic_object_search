#!/usr/bin/env bash
# Prepare a fresh Ubuntu/ROS checkout for CPU operation. System dependencies
# are installed separately so running this script never asks for sudo.
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
if [[ ! -r /opt/ros/humble/setup.bash ]] || ! command -v colcon >/dev/null; then
    echo '请先按 docs/maintained/GITHUB_DEPLOYMENT.md 安装 ROS 2 Humble 和 colcon。' >&2
    exit 1
fi

/usr/bin/python3 scripts/prepare_runtime.py
/usr/bin/python3 scripts/prepare_target_assets.py --layout scenes/aws_house/layout_r0_yellow_v1.json
/usr/bin/python3 -m venv --without-pip --system-site-packages .venv
.venv/bin/python -m pip install --cache-dir downloads/pip_cache -r requirements-vision-ubuntu-cpu.txt
.venv/bin/python -m pip check
bash scripts/build.sh

echo '基础部署完成。可运行 bash scripts/start_robot.sh --device cpu --headless。'
