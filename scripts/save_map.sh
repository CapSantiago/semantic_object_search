#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
source "$SEMANTIC_PROJECT_ROOT/scripts/scene_env.sh" "$@"
if [[ "$SEMANTIC_SCENE" == baseline ]]; then
    # Keep the accepted baseline maps byte-for-byte intact.
    semantic_map_prefix="$SEMANTIC_WS/maps/baseline/rebuilt_$(date +%Y%m%dT%H%M%S)"
else
    semantic_map_prefix="${SEMANTIC_MAP%.yaml}"
fi
mkdir -p -- "$(dirname -- "$semantic_map_prefix")"
ros2 run nav2_map_server map_saver_cli -f "$semantic_map_prefix" \
    --ros-args -p use_sim_time:=true -p save_map_timeout:=15.0
