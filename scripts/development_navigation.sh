#!/usr/bin/env bash
set -eo pipefail
export SEMANTIC_ROS_DOMAIN_ID="${SEMANTIC_ROS_DOMAIN_ID:-64}"
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
source "$SEMANTIC_PROJECT_ROOT/scripts/native_gui_env.sh"
setsid ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True autostart:=False \
    use_composition:=False map:="$SEMANTIC_WS/scenes/aws_house/r0_v1/maps/house.yaml" \
    params_file:="$SEMANTIC_WS/src/semantic_search/config/nav2_depth.yaml" "$@" &
semantic_dev_nav_pid=$!
# Own a separate process group: a background launch can ignore SIGINT, and
# terminating only its Python parent can orphan the lifecycle nodes.
trap 'kill -TERM -- "-$semantic_dev_nav_pid" 2>/dev/null || true; wait "$semantic_dev_nav_pid" || true' EXIT
trap 'exit 130' INT TERM
python3 "$SEMANTIC_PROJECT_ROOT/scripts/start_navigation.py"
wait "$semantic_dev_nav_pid"
