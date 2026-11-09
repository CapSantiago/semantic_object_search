#!/usr/bin/env bash
# Recreate the isolated preparation environment; does not record or transcribe.
set -eo pipefail
semantic_voice_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
semantic_voice_offline=0
semantic_voice_pip_args=(--index-url https://pypi.org/simple)
semantic_voice_model_args=()
if [[ "${1:-}" == --offline ]]; then
    semantic_voice_offline=1
    semantic_voice_pip_args=(--no-index --find-links "$semantic_voice_root/downloads/voice_wheels")
    semantic_voice_model_args=(--offline)
elif [[ $# != 0 ]]; then
    echo 'Usage: bash scripts/prepare_voice.sh [--offline]' >&2
    exit 1
fi
python3 -m venv --without-pip --system-site-packages "$semantic_voice_root/.venv-voice"
"$semantic_voice_root/.venv-voice/bin/python" -m pip --isolated install --ignore-installed \
    "${semantic_voice_pip_args[@]}" --cache-dir "$semantic_voice_root/downloads/pip_cache" \
    -r "$semantic_voice_root/requirements-voice-ubuntu-cpu.txt"
python3 - "$semantic_voice_root" "$semantic_voice_offline" <<'PY'
import hashlib,json,subprocess,sys,urllib.request
from pathlib import Path
root=Path(sys.argv[1]);record=json.loads((root/'resources/native_audio.lock.json').read_text())
archive=root/record['archive']
archive.parent.mkdir(parents=True,exist_ok=True)
if not archive.exists():
    if sys.argv[2]=='1':raise RuntimeError('Native audio archive missing in offline mode')
    with urllib.request.urlopen(record['url'],timeout=60) as response:archive.write_bytes(response.read())
if hashlib.sha256(archive.read_bytes()).hexdigest()!=record['sha256']:
    raise RuntimeError('Native audio archive checksum differs from lock')
subprocess.run(['dpkg-deb','-x',str(archive),str(root/'resources/native')],check=True)
link=root/'resources/native/usr/lib/x86_64-linux-gnu/libportaudio.so'
if not link.exists():link.symlink_to('libportaudio.so.2')
PY
"$semantic_voice_root/.venv-voice/bin/python" "$semantic_voice_root/scripts/prepare_voice_model.py" \
    "${semantic_voice_model_args[@]}"
echo 'Voice software/data prepared; audio and inference remain untested.'
