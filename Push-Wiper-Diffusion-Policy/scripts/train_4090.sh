#!/usr/bin/env bash
# From any working directory: install the isolated CUDA environment, then train.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "$script_dir/.." && pwd)"
venv_dir="${PUSH_WIPER_VENV:-$project_root/.venv}"
if [[ "$venv_dir" != /* ]]; then
    venv_dir="$PWD/$venv_dir"
fi
export PYTHONNOUSERSITE=1
unset PYTHONPATH
export MPLCONFIGDIR="$project_root/.cache/matplotlib"
PUSH_WIPER_VENV="$venv_dir" bash "$script_dir/install_cuda.sh"
cd -- "$project_root"
exec "$venv_dir/bin/python" -m push_wiper_dp.train \
    --config-name=push_wiper runtime=rtx4090 "$@"
