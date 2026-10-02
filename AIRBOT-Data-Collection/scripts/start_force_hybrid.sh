#!/usr/bin/env bash
# Start the standalone AIRBOT + Kunwei/LFS force-position controller.
#
# The script is intentionally separate from the Push-Wiper policy launcher.
# It only starts the controller selected below; the AIRBOT server remains a
# separately managed process so its CAN interface and permissions stay explicit.

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "$script_dir/.." && pwd)"
workspace_root="$(cd -- "$project_root/.." && pwd)"

usage() {
    cat <<'EOF'
用法：
  scripts/start_force_hybrid.sh [run] [选项]
  scripts/start_force_hybrid.sh validate [选项]
  scripts/start_force_hybrid.sh sensor-check [选项]
  scripts/start_force_hybrid.sh sensor-read [选项]
  scripts/start_force_hybrid.sh drag-preview [选项]

动作：
  run           启动独立力位混合控制（默认）
  validate      只校验配置和 NPZ，不连接硬件
  sensor-check  只检查当前配置的力传感器，不移动机械臂
  sensor-read   只读当前配置的力传感器，不发送硬件清零命令
  drag-preview  连接 AIRBOT 并预览拖拽位姿，不连接力传感器

环境变量：
  FORCE_HYBRID_CONFIG   配置文件路径，默认 airbot_ie/configs/force_hybrid_push.json
  FORCE_HYBRID_PYTHON   Python 可执行文件；未设置时优先使用当前 python，回退到仓库 .venv-airdc/bin/python

动作后面的选项会原样传递给 Python 入口，例如：
  scripts/start_force_hybrid.sh drag-preview --preview-hz 30

也可以直接覆盖配置文件：
  scripts/start_force_hybrid.sh validate --config path/to/force_hybrid.json
EOF
}

mode="run"
if [[ $# -gt 0 ]]; then
    case "$1" in
        run|validate|sensor-check|sensor-read|drag-preview)
            mode="$1"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
    esac
fi

config_path="${FORCE_HYBRID_CONFIG:-airbot_ie/configs/force_hybrid_push.json}"
if [[ -n "${FORCE_HYBRID_PYTHON:-}" ]]; then
    python_bin="$FORCE_HYBRID_PYTHON"
elif command -v python >/dev/null 2>&1; then
    # Prefer the currently activated environment (for example airdc).
    python_bin="python"
else
    python_bin="$workspace_root/.venv-airdc/bin/python"
fi

# Resolve a user-provided relative interpreter before changing directories.
if [[ "$python_bin" == */* && "$python_bin" != /* ]]; then
    python_dir="$(dirname -- "$python_bin")"
    if [[ -d "$python_dir" ]]; then
        python_bin="$(cd -- "$python_dir" && pwd)/$(basename -- "$python_bin")"
    fi
fi

# Consume --config here so the preflight check and the Python entry point use
# the same file. All other options are forwarded unchanged.
forwarded_args=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --config)
            if [[ $# -lt 2 ]]; then
                echo "--config 需要一个文件路径。" >&2
                exit 2
            fi
            config_path="$2"
            shift 2
            ;;
        --config=*)
            config_path="${1#--config=}"
            shift
            ;;
        *)
            forwarded_args+=("$1")
            shift
            ;;
    esac
done

if [[ "$config_path" != /* ]]; then
    config_path="$project_root/$config_path"
fi
if [[ ! -f "$config_path" ]]; then
    echo "找不到力位控制配置：$config_path" >&2
    exit 2
fi

if [[ "$python_bin" == */* ]]; then
    if [[ ! -x "$python_bin" ]]; then
        echo "找不到可执行 Python：$python_bin" >&2
        echo "可设置 FORCE_HYBRID_PYTHON，或先创建仓库 .venv-airdc 环境。" >&2
        exit 2
    fi
else
    if ! command -v "$python_bin" >/dev/null 2>&1; then
        echo "找不到 Python 命令：$python_bin" >&2
        exit 2
    fi
fi

cd -- "$project_root"
export PYTHONPATH="$project_root${PYTHONPATH:+:$PYTHONPATH}"

controller_args=(--config "$config_path")
case "$mode" in
    validate)
        controller_args+=(--validate)
        ;;
    sensor-check)
        controller_args+=(--sensor-check)
        ;;
    sensor-read)
        controller_args+=(--sensor-read)
        ;;
    drag-preview)
        controller_args+=(--drag-preview)
        ;;
    run)
        ;;
    *)
        echo "未知动作：$mode" >&2
        usage >&2
        exit 2
        ;;
esac

echo "独立力位混合控制：mode=$mode"
echo "配置：$config_path"
echo "Python：$python_bin"
if [[ "$mode" == run ]]; then
    robot_port=$("$python_bin" -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8")).get("robot", {}).get("port", 50051))' "$config_path")
    echo "请确认 AIRBOT 服务已在端口 $robot_port 启动，并确认工具悬空。"
fi

exec "$python_bin" -m airbot_ie.scripts.force_hybrid_push "${controller_args[@]}" "${forwarded_args[@]}"
