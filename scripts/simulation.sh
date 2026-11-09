#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
source "$SEMANTIC_PROJECT_ROOT/scripts/scene_env.sh" "$@"
source "$SEMANTIC_PROJECT_ROOT/scripts/native_gui_env.sh"
ros2 launch semantic_search fortress.launch.py \
    scene:="$SEMANTIC_SCENE" project_root:="$SEMANTIC_WS" \
    upstream_dir:="$SEMANTIC_WS/third_party/tb3_fortress/turtlebot3" \
    chair_asset:="$SEMANTIC_WS/third_party/fuel_chair" "${SEMANTIC_LAUNCH_ARGS[@]}"
