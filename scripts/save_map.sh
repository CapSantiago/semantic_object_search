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
semantic_save_args=(--new-output --scene "$SEMANTIC_SCENE")
if [[ -n "$semantic_map_prefix" ]]; then
    semantic_save_args+=(--output-prefix "$semantic_map_prefix")
fi
semantic_map_prefix="$(python3 "$SEMANTIC_PROJECT_ROOT/src/semantic_search/semantic_search/map_resources.py" \
    --root "$SEMANTIC_WS" "${semantic_save_args[@]}")"
mkdir -p -- "$(dirname -- "$semantic_map_prefix")"
# Nav2 represents unknown cells as gray 205 in a trinary PGM. A free
# threshold below 50/255 keeps these cells unknown when the image is loaded.
exec ros2 run nav2_map_server map_saver_cli -f "$semantic_map_prefix" --free 0.19 --occ 0.65 \
    --ros-args -p use_sim_time:=true -p save_map_timeout:=15.0
