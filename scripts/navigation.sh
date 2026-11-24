#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
source "$SEMANTIC_PROJECT_ROOT/scripts/scene_env.sh" "$@"
semantic_nav_args=()
semantic_map_selector=(--yaml-path "$SEMANTIC_MAP")
for semantic_nav_arg in "${SEMANTIC_LAUNCH_ARGS[@]}"; do
    case "$semantic_nav_arg" in
        map:=*) semantic_map_selector=(--yaml-path "${semantic_nav_arg#map:=}") ;;
        map_id:=*) semantic_map_selector=(--map-id "${semantic_nav_arg#map_id:=}") ;;
        *) semantic_nav_args+=("$semantic_nav_arg") ;;
    esac
done
semantic_nav_map="$(python3 "$SEMANTIC_PROJECT_ROOT/src/semantic_search/semantic_search/map_resources.py" \
    --root "$SEMANTIC_WS" "${semantic_map_selector[@]}")"
# Nav2's YAML rewrite also needs ASCII paths with this Humble installation.
ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=True autostart:=True use_composition:=False \
    map:="$semantic_nav_map" \
    params_file:="$SEMANTIC_WS/src/semantic_search/config/nav2.yaml" "${semantic_nav_args[@]}"
