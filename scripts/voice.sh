#!/usr/bin/env bash
# Runtime entry point prepared for P4; preparation does not execute this file.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/voice_env.sh"
semantic_voice_model="$SEMANTIC_WS/resources/models/faster-whisper-small"
if [[ ! -f "$semantic_voice_model/model.bin" ]]; then
    echo "Offline speech model missing; see resources/voice_model.lock.json" >&2
    exit 1
fi
exec "$SEMANTIC_WS/.venv-voice/bin/python" -m semantic_search.voice \
    --model "$semantic_voice_model" "$@"
