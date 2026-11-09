#!/usr/bin/env bash
# Use instead of AMCL/navigation.sh while building a scene's own map.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
source "$SEMANTIC_PROJECT_ROOT/scripts/scene_env.sh" "$@"
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true \
    slam_params_file:="$SEMANTIC_WS/src/semantic_search/config/slam.yaml" \
    "${SEMANTIC_LAUNCH_ARGS[@]}"
