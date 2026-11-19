#!/usr/bin/env bash
# Source this file from any directory; keep rosidl paths ASCII on Humble.
SEMANTIC_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
export SEMANTIC_PROJECT_ROOT
# Give each checkout its own ASCII alias. A second clone can coexist with the
# original project without replacing symlinks used by a build or simulator.
SEMANTIC_WORKSPACE_KEY="$(printf '%s' "$SEMANTIC_PROJECT_ROOT" | sha256sum)"
export SEMANTIC_WORKSPACE_KEY="${SEMANTIC_WORKSPACE_KEY:0:16}"
export SEMANTIC_WS="/tmp/semantic_object_search_${UID}_${SEMANTIC_WORKSPACE_KEY}"
if [[ -L "$SEMANTIC_WS" ]]; then
    if [[ "$(readlink -f -- "$SEMANTIC_WS")" != "$SEMANTIC_PROJECT_ROOT" ]]; then
        echo "Workspace alias belongs to another project: $SEMANTIC_WS" >&2
        return 1
    fi
elif [[ -e "$SEMANTIC_WS" ]]; then
    echo "Workspace alias already exists: $SEMANTIC_WS" >&2
    return 1
else
    ln -s -- "$SEMANTIC_PROJECT_ROOT" "$SEMANTIC_WS" || return 1
fi
if [[ ! -r /opt/ros/humble/setup.bash ]]; then
    echo '找不到 ROS 2 Humble。请先按 docs/maintained/GITHUB_DEPLOYMENT.md 安装系统依赖。' >&2
    return 1
fi
source /opt/ros/humble/setup.bash || return 1
# This project runs entirely on this machine. Do not inherit a TurtleBot4
# discovery server from the user's shell; opt in explicitly for remote robots.
if [[ "${SEMANTIC_USE_SYSTEM_DDS:-0}" != 1 ]]; then
    unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
    export ROS_LOCALHOST_ONLY=1
    export ROS_DOMAIN_ID="${SEMANTIC_ROS_DOMAIN_ID:-42}"
fi
if [[ -f "$SEMANTIC_PROJECT_ROOT/install/setup.bash" ]]; then
    source "$SEMANTIC_PROJECT_ROOT/install/setup.bash"
fi
export ROS_LOG_DIR="$SEMANTIC_PROJECT_ROOT/log/ros"
mkdir -p -- "$ROS_LOG_DIR"
