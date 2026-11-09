#!/usr/bin/env bash
# Nav2 planning/control with SLAM supplying /map and map -> odom.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
source "$SEMANTIC_PROJECT_ROOT/scripts/scene_env.sh" "$@"
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True \
    autostart:=True use_composition:=False \
    params_file:="$SEMANTIC_WS/src/semantic_search/config/nav2.yaml" \
    "${SEMANTIC_LAUNCH_ARGS[@]}"
