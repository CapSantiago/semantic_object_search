#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
# CMake resolves the binary directory physically, so an ASCII symlink alone
# cannot fix a build directory under a Chinese Desktop path.
SEMANTIC_BUILD_ROOT="${SEMANTIC_BUILD_ROOT:-${XDG_CACHE_HOME:-$HOME/.cache}/semantic_object_search/${SEMANTIC_WORKSPACE_KEY}/build}"
colcon --log-base "$SEMANTIC_WS/log" build \
    --paths "$SEMANTIC_WS/src/semantic_search" "$SEMANTIC_WS/src/service_navfn" "$SEMANTIC_WS/third_party/yolo_ros/yolo_msgs" \
    --build-base "$SEMANTIC_BUILD_ROOT" --install-base "$SEMANTIC_PROJECT_ROOT/install" \
    --symlink-install "$@"
