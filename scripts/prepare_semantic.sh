#!/usr/bin/env bash
# Spark-DSG graph adapter, isolated from the detector and ROS environments.
set -eo pipefail
export PIP_DEFAULT_TIMEOUT="${PIP_DEFAULT_TIMEOUT:-120}"
export PIP_RETRIES="${PIP_RETRIES:-5}"
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
/usr/bin/python3 -m venv --without-pip --system-site-packages .venv-semantic
.venv-semantic/bin/python -m pip install --cache-dir downloads/pip_cache -r requirements-semantic.txt
.venv-semantic/bin/python - <<'PY'
from importlib.metadata import version
import spark_dsg
graph = spark_dsg.DynamicSceneGraph()
assert graph.num_nodes() == 0
print('Spark-DSG 已就绪：' + version('spark-dsg'))
PY
