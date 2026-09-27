#!/usr/bin/env bash
# Source this script from any directory; do not execute it in a child shell.
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo '使用方式：source install/activate_airdc.sh' >&2
    exit 1
fi

if ! command -v conda >/dev/null 2>&1; then
    echo '找不到 conda，请先初始化 Miniconda 的 shell。' >&2
    return 1
fi
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate airdc
