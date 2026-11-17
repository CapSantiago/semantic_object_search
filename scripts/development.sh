#!/usr/bin/env bash
# Candidate R0 scene only. Keep its ROS domain independent of archived demos.
set -eo pipefail
export SEMANTIC_ROS_DOMAIN_ID="${SEMANTIC_ROS_DOMAIN_ID:-64}"
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
source "$SEMANTIC_PROJECT_ROOT/scripts/native_gui_env.sh"
ros2 launch semantic_search development.launch.py \
    project_root:="$SEMANTIC_WS" "$@"
