PYTHON_VERSION=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
PYTHON_MAJOR=$(echo "$PYTHON_VERSION" | cut -d. -f1)
PYTHON_MINOR=$(echo "$PYTHON_VERSION" | cut -d. -f2)

if [ "$PYTHON_MAJOR" -eq 3 ] && [ "$PYTHON_MINOR" -lt 12 ]; then
    python3 -m pip install pyrealsense2 -i https://pypi.mirrors.ustc.edu.cn/simple
else
    python3 -m pip install pyrealsense2-beta -i https://pypi.mirrors.ustc.edu.cn/simple
fi
