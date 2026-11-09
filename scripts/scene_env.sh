#!/usr/bin/env bash
# Source after env.sh. ROS-style scene:=... is accepted by every scene launcher.
SEMANTIC_SCENE="${SEMANTIC_SCENE:-aws_house}"
SEMANTIC_LAUNCH_ARGS=()
for semantic_scene_arg in "$@"; do
    case "$semantic_scene_arg" in
        scene:=*) SEMANTIC_SCENE="${semantic_scene_arg#scene:=}" ;;
        *) SEMANTIC_LAUNCH_ARGS+=("$semantic_scene_arg") ;;
    esac
done
export SEMANTIC_SCENE
mapfile -t semantic_scene_fields < <(python3 \
    "$SEMANTIC_PROJECT_ROOT/src/semantic_search/semantic_search/scenes.py" \
    "$SEMANTIC_SCENE" "$SEMANTIC_WS")
if [[ ${#semantic_scene_fields[@]} != 6 ]]; then
    echo "Cannot load scene: $SEMANTIC_SCENE" >&2
    return 1
fi
SEMANTIC_SCENE_LABEL="${semantic_scene_fields[0]}"
SEMANTIC_WORLD="${semantic_scene_fields[1]}"
SEMANTIC_MAP="${semantic_scene_fields[2]}"
SEMANTIC_ROOMS="${semantic_scene_fields[3]}"
SEMANTIC_ROUTE="${semantic_scene_fields[4]}"
SEMANTIC_OUTPUT="${semantic_scene_fields[5]}"
export SEMANTIC_SCENE_LABEL SEMANTIC_WORLD SEMANTIC_MAP SEMANTIC_ROOMS SEMANTIC_ROUTE SEMANTIC_OUTPUT
echo "[$SEMANTIC_SCENE] $SEMANTIC_SCENE_LABEL"
unset semantic_scene_fields semantic_scene_arg
