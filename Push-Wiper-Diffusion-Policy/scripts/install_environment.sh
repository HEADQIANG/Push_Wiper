#!/usr/bin/env bash
set -euo pipefail
runtime="${1:?Expected cpu or cuda}"
case "$runtime" in cpu|cuda) ;; *) printf 'Expected cpu or cuda\n' >&2; exit 2 ;; esac
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
venv_dir="${PUSH_WIPER_VENV:-$project_root/.venv}"
python_command="${PUSH_WIPER_PYTHON:-python3.10}"
pypi_index="${PUSH_WIPER_PIP_INDEX:-https://pypi.org/simple}"
export PYTHONNOUSERSITE=1
unset PYTHONPATH
export PIP_CACHE_DIR="$project_root/.cache/pip"
export MPLCONFIGDIR="$project_root/.cache/matplotlib"
export PIP_DISABLE_PIP_VERSION_CHECK=1
if [[ ! -x "$venv_dir/bin/python" ]]; then
    "$python_command" -m venv "$venv_dir"
fi
"$venv_dir/bin/python" -c 'import sys; assert sys.version_info[:2] == (3, 10), "Python 3.10 is required"; assert sys.prefix != sys.base_prefix, "Refusing to install outside a virtual environment"'
"$venv_dir/bin/python" -m pip install --index-url "$pypi_index" pip==24.3.1 setuptools==75.6.0 wheel==0.45.1
"$venv_dir/bin/python" -m pip install --index-url "$pypi_index" -r "$project_root/requirements/common.lock"
"$venv_dir/bin/python" -m pip install -c "$project_root/requirements/common.lock" -r "$project_root/requirements/$runtime.txt"
"$venv_dir/bin/python" -m pip install --no-deps --no-build-isolation -e "$project_root/third_party/diffusion_policy" -e "$project_root"
"$venv_dir/bin/python" -m pip check
"$venv_dir/bin/python" "$project_root/scripts/check_environment.py" --device "$runtime" --report "$project_root/outputs/environment_$runtime.json"
printf 'Environment ready. Activate with:\nsource %q/bin/activate\nunset PYTHONPATH\nexport PYTHONNOUSERSITE=1\n' "$venv_dir"
