#!/usr/bin/env bash
# Project-local GPU runtime; the existing CPU environment is kept intact.
set -euo pipefail
gpu_project="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd -- "$gpu_project"
python3 -m venv --without-pip --system-site-packages .venv-gpu
mkdir -p downloads/pip_cache
.venv-gpu/bin/python -m pip install --cache-dir downloads/pip_cache -r requirements-vision-ubuntu-gpu.txt
.venv-gpu/bin/python - <<'PY'
import torch, torchvision, ultralytics
if not torch.cuda.is_available():
    raise SystemExit('CUDA不可用；检查NVIDIA驱动。CPU入口：bash scripts/start_robot.sh --device cpu')
x=torch.ones((32,32),device='cuda:0'); y=x@x
torch.cuda.synchronize()
assert y[0,0].item()==32
print(f'GPU已就绪：{torch.cuda.get_device_name(0)}；torch={torch.__version__}，CUDA={torch.version.cuda}')
PY
