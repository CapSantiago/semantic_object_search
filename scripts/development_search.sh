#!/usr/bin/env bash
set -eo pipefail
export SEMANTIC_ROS_DOMAIN_ID="${SEMANTIC_ROS_DOMAIN_ID:-64}"
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
ros2 run semantic_search search --ros-args -p use_sim_time:=true \
    -p scene_id:=aws_house_r0_v1 \
    -p rooms_file:="$SEMANTIC_WS/src/semantic_search/config/rooms_aws_house_v1.json" \
    -p output_dir:="$SEMANTIC_WS/results/development/r0_v1/tasks" "$@"
