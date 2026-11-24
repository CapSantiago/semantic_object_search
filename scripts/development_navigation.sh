#!/usr/bin/env bash
set -eo pipefail
export SEMANTIC_ROS_DOMAIN_ID="${SEMANTIC_ROS_DOMAIN_ID:-64}"
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
source "$SEMANTIC_PROJECT_ROOT/scripts/native_gui_env.sh"
semantic_nav_args=()
semantic_map_id=aws_house_v1
semantic_map_yaml=''
for semantic_nav_arg in "$@"; do
    case "$semantic_nav_arg" in
        map:=*) semantic_map_yaml="${semantic_nav_arg#map:=}" ;;
        map_id:=*) semantic_map_id="${semantic_nav_arg#map_id:=}" ;;
        *) semantic_nav_args+=("$semantic_nav_arg") ;;
    esac
done
if [[ -n "$semantic_map_yaml" ]]; then
    semantic_map_selector=(--yaml-path "$semantic_map_yaml")
else
    semantic_map_selector=(--map-id "$semantic_map_id")
fi
semantic_nav_map="$(python3 "$SEMANTIC_PROJECT_ROOT/src/semantic_search/semantic_search/map_resources.py" \
    --root "$SEMANTIC_WS" "${semantic_map_selector[@]}")"
setsid ros2 launch semantic_search navigation.launch.py use_sim_time:=True autostart:=False \
    use_composition:=False map:="$semantic_nav_map" \
    params_file:="$SEMANTIC_WS/src/semantic_search/config/nav2_depth.yaml" "${semantic_nav_args[@]}" &
semantic_dev_nav_pid=$!
# Own a separate process group: a background launch can ignore SIGINT, and
# terminating only its Python parent can orphan the lifecycle nodes.
trap 'kill -TERM -- "-$semantic_dev_nav_pid" 2>/dev/null || true; wait "$semantic_dev_nav_pid" || true' EXIT
trap 'exit 130' INT TERM
python3 "$SEMANTIC_PROJECT_ROOT/scripts/start_navigation.py"
wait "$semantic_dev_nav_pid"
