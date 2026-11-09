#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
source "$SEMANTIC_PROJECT_ROOT/scripts/scene_env.sh" "$@"
if [[ ! -f "$SEMANTIC_MAP" ]]; then
    echo "Scene map missing: $SEMANTIC_MAP. Build it with scripts/mapping.sh first." >&2
    exit 1
fi
# Nav2's YAML rewrite also needs ASCII paths with this Humble installation.
ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=True autostart:=True use_composition:=False \
    map:="$SEMANTIC_MAP" \
    params_file:="$SEMANTIC_WS/src/semantic_search/config/nav2.yaml" "${SEMANTIC_LAUNCH_ARGS[@]}"
