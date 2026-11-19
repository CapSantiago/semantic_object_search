#!/usr/bin/env bash
set -eo pipefail
export SEMANTIC_ROS_DOMAIN_ID="${SEMANTIC_ROS_DOMAIN_ID:-64}"
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
source "$SEMANTIC_PROJECT_ROOT/scripts/native_gui_env.sh"
export YOLO_CONFIG_DIR="$SEMANTIC_PROJECT_ROOT/results/ultralytics"
semantic_device="${SEMANTIC_DETECTOR_DEVICE:-cpu}"
case "$semantic_device" in
    cpu) semantic_python="$SEMANTIC_PROJECT_ROOT/.venv/bin/python" ;;
    cuda:0) semantic_python="$SEMANTIC_PROJECT_ROOT/.venv-gpu/bin/python" ;;
    *) echo 'SEMANTIC_DETECTOR_DEVICE必须为cpu或cuda:0' >&2; exit 2 ;;
esac
[[ -x "$semantic_python" ]] || { echo "缺少视觉环境：$semantic_python" >&2; exit 2; }
"$semantic_python" -m semantic_search.detector --ros-args \
    -p use_sim_time:=true -p model:="$SEMANTIC_WS/yolov8n-seg.pt" \
    -p device:="$semantic_device" -p iou_threshold:=0.30 -p image_size:=640 "$@"
