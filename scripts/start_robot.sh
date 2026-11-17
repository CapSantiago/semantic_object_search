#!/usr/bin/env bash
# One foreground owner for the robot stack and its local browser console.
set -eo pipefail
semantic_launch_domain=66
semantic_launch_args=("$@")
for ((semantic_launch_i=0; semantic_launch_i < ${#semantic_launch_args[@]}; semantic_launch_i++)); do
    case "${semantic_launch_args[$semantic_launch_i]}" in
        --domain)
            semantic_launch_i=$((semantic_launch_i + 1))
            semantic_launch_domain="${semantic_launch_args[$semantic_launch_i]:-}"
            ;;
        --domain=*) semantic_launch_domain="${semantic_launch_args[$semantic_launch_i]#--domain=}" ;;
    esac
done
if [[ ! "$semantic_launch_domain" =~ ^[0-9]+$ ]] || (( 10#$semantic_launch_domain > 232 )); then
    echo "--domain 必须是 0 到 232 的整数" >&2
    exit 2
fi
export SEMANTIC_ROS_DOMAIN_ID="$((10#$semantic_launch_domain))"
export SEMANTIC_USE_SYSTEM_DDS=0
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
source "$SEMANTIC_PROJECT_ROOT/scripts/native_gui_env.sh"
cd -- "$SEMANTIC_PROJECT_ROOT"
exec /usr/bin/python3 "$SEMANTIC_PROJECT_ROOT/scripts/robot_console.py" "$@"
