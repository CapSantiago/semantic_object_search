#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
source "$SEMANTIC_PROJECT_ROOT/scripts/scene_env.sh" "$@"
if [[ ! -x "$SEMANTIC_PROJECT_ROOT/.venv/bin/python" ]]; then
    echo "Vision environment missing: see HANDOFF.md" >&2
    exit 1
fi
ros2 launch semantic_search search.launch.py \
    scene:="$SEMANTIC_SCENE" project_root:="$SEMANTIC_WS" \
    with_detector:=true detector_python:="$SEMANTIC_WS/.venv/bin/python" \
    model:="$SEMANTIC_WS/yolov8n-seg.pt" "${SEMANTIC_LAUNCH_ARGS[@]}"
