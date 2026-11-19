#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
source "$SEMANTIC_PROJECT_ROOT/scripts/scene_env.sh" "$@"
semantic_map_prefix=""
for semantic_map_arg in "${SEMANTIC_LAUNCH_ARGS[@]}"; do
    if [[ "$semantic_map_arg" == map_output:=* ]]; then
        semantic_map_prefix="${semantic_map_arg#map_output:=}"
        [[ -n "$semantic_map_prefix" ]] || { echo "map_output requires a file prefix" >&2; exit 2; }
    fi
done
if [[ -z "$semantic_map_prefix" && "$SEMANTIC_SCENE" == baseline ]]; then
    # Keep the accepted baseline maps byte-for-byte intact.
    semantic_map_prefix="$SEMANTIC_WS/maps/baseline/rebuilt_$(date +%Y%m%dT%H%M%S)"
elif [[ -z "$semantic_map_prefix" ]]; then
    semantic_map_prefix="${SEMANTIC_MAP%.yaml}"
fi
mkdir -p -- "$(dirname -- "$semantic_map_prefix")"
# Nav2 represents unknown cells as gray 205 in a trinary PGM. A free
# threshold below 50/255 keeps these cells unknown when the image is loaded.
exec ros2 run nav2_map_server map_saver_cli -f "$semantic_map_prefix" --free 0.19 --occ 0.65 \
    --ros-args -p use_sim_time:=true -p save_map_timeout:=15.0
