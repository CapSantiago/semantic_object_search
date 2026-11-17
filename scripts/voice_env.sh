#!/usr/bin/env bash
# Source only for the separately prepared voice environment.
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh" || return 1
if [[ ! -x "$SEMANTIC_PROJECT_ROOT/.venv-voice/bin/python" ]]; then
    echo "Voice environment missing; see docs/maintained/RESOURCE_PREPARATION.md" >&2
    return 1
fi
source "$SEMANTIC_PROJECT_ROOT/scripts/native_gui_env.sh"
source "$SEMANTIC_PROJECT_ROOT/.venv-voice/bin/activate"
SEMANTIC_VOICE_NATIVE="$SEMANTIC_PROJECT_ROOT/resources/native/usr/lib/x86_64-linux-gnu"
if [[ -d "$SEMANTIC_VOICE_NATIVE" ]]; then
    export LD_LIBRARY_PATH="$SEMANTIC_VOICE_NATIVE${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export LIBRARY_PATH="$SEMANTIC_VOICE_NATIVE${LIBRARY_PATH:+:$LIBRARY_PATH}"
fi
unset SEMANTIC_VOICE_NATIVE
