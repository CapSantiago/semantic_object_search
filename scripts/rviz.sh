#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
source "$SEMANTIC_PROJECT_ROOT/scripts/scene_env.sh" "$@"
source "$SEMANTIC_PROJECT_ROOT/scripts/native_gui_env.sh"
exec ros2 run rviz2 rviz2 -d "$SEMANTIC_WS/src/semantic_search/config/search.rviz" \
    --ros-args -p use_sim_time:=true "${SEMANTIC_LAUNCH_ARGS[@]}"
