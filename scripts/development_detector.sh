#!/usr/bin/env bash
set -eo pipefail
export SEMANTIC_ROS_DOMAIN_ID="${SEMANTIC_ROS_DOMAIN_ID:-64}"
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
source "$SEMANTIC_PROJECT_ROOT/scripts/native_gui_env.sh"
export YOLO_CONFIG_DIR="$SEMANTIC_PROJECT_ROOT/results/ultralytics"
"$SEMANTIC_PROJECT_ROOT/.venv/bin/python" -m semantic_search.detector --ros-args \
    -p use_sim_time:=true -p model:="$SEMANTIC_WS/yolov8n-seg.pt" \
    -p device:=cpu -p iou_threshold:=0.30 -p image_size:=640 "$@"
