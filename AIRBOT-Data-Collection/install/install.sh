# Install common used dependencies for AIRDC

set -e

if [ $# -eq 0 ]; then
    set -- sudo apt install -y
fi

# TODO: is gcc necessary?
"$@" pip python3 libturbojpeg gcc
python3 -m pip install --upgrade pip -i https://pypi.mirrors.ustc.edu.cn/simple
python3 -m pip install -e ."[all,airbot]" -i https://pypi.mirrors.ustc.edu.cn/simple
