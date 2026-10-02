# 本机数据采集环境

使用独立的 Conda 环境 `airdc`，目录为 `/home/wp/miniconda3/envs/airdc`，
Python 版本为 3.10.21，面向 AIRDC 默认的 MCAP 采集流程。
已有的 `clean`、`discoverse` 等环境保持不变。

## 激活

```bash
cd /home/wp/yuelk_project/Push_Wiper/AIRBOT-Data-Collection
conda activate airdc
python --version
python -m pip check
python scripts/check_collection_env.py
airdc --help demonstrators=test
```

移动仓库目录后，环境中的 editable 安装路径不会自动更新。需在新仓库根目录、激活环境后执行：

```bash
python -m pip install --no-deps --no-build-isolation -e .
```

这只重新注册当前源码路径，不更换依赖版本。Push-Wiper 分段采集入口与复位流程见
[专用采集文档](push_wiper_collection.md)。

若当前终端尚未初始化 Conda，可使用 `source install/activate_airdc.sh`。
本环境通过 `conda env config vars` 设置 `PYTHONNOUSERSITE=1`、空 `PYTHONPATH` 和环境
自身的 `LD_LIBRARY_PATH`，避免用户目录中的 protobuf 和系统 ROS 包覆盖环境依赖。
退出使用 `conda deactivate`，Conda 会恢复原来的环境变量。
上一步创建的 `.venv-airdc` 不再作为运行环境。
若终端仍显示 `(.venv-airdc)`，请先执行
`if declare -F deactivate >/dev/null; then deactivate; fi`，再激活 `airdc`。
旧虚拟环境的激活脚本保留了搬迁前的目录，可能导致“找不到命令 python”；完整恢复命令见
[RealSense 视频预览的常见问题](realsense_preview.md#常见问题)。

## 重建环境

```bash
conda create -n airdc python=3.10 pip libjpeg-turbo -y
conda activate airdc
conda env config vars set PYTHONNOUSERSITE=1 PYTHONPATH= "LD_LIBRARY_PATH=${CONDA_PREFIX}/lib"
conda deactivate
conda activate airdc
python -m pip install -c install/airdc.constraints -e '.[all,rs,image]' \
  /home/wp/下载/airbot_py-5.1.6-py3-none-any.whl \
  -i https://pypi.mirrors.ustc.edu.cn/simple
```

在其他机器重建时，请将 SDK wheel 路径替换为当地保存的位置。

## 无硬件自检

`scripts/check_collection_env.py` 验证 AIRDC、AIRBOT SDK、RealSense、V4L2 模块导入，
执行 JPEG 编解码，并将双臂关节/夹爪、两路彩色视频和 uint16 深度写入 MCAP 后读回。
仅使用合成数据，不连接或控制机械臂；临时文件在检查后自动清理。
`airdc --help demonstrators=test` 使用已有模拟配置检查命令行和 Hydra 配置加载，不启动采集。

## 依赖选择

- AIRBOT Python SDK：本机 `/home/wp/下载/airbot_py-5.1.6-py3-none-any.whl`，
  对应已有 `airbot_server` 的 5.1.6 服务启动脚本。
- 原项目锁定的 `mcap-data-loader==0.1.2` 已无法从公开安装源获取，改用相邻的 0.1.3，
  并将 `StrEnum`、`DataStamped`、`DictDataStamped` 的导入调整到其公开的 `basis` 模块。
- NumPy、OpenCV、PyTurboJPEG 的环境约束见 `install/airdc.constraints`。
  PyTurboJPEG 使用 AIRDC 所需的旧版 API，动态库由 Conda 的 `libjpeg-turbo` 提供。
- 普通 RGB 相机使用 V4L2，RealSense 使用 `pyrealsense2`。
- 本环境使用 MCAP 保存。LeRobot 导出需要额外安装 LeRobot 并验证其独立依赖组合。

## 系统与硬件前提

本机已识别内置 USB 摄像头 `/dev/video0`。2026-09-21 已检测到 RealSense D435IF，
序列号为 `247122070601`，连接速率为 USB 2.1；当前连续取帧仍超时，尚未验证视频流成功。
预览脚本、运行步骤与排查方法见 [RealSense 视频预览](realsense_preview.md)。机械臂尚未连接验证。
Python 环境验证不能代替真机验证。
`airbot_server` 是调用 Docker 的脚本；还需要安装并启动 Docker 后才能启动机械臂服务。
系统安装需要在本机终端输入 sudo 密码，不能通过聊天提供密码。

需要运行真机服务时，在本机终端完成：

```bash
sudo apt update
sudo apt install -y docker.io v4l-utils can-utils
sudo systemctl enable --now docker
sudo install -m 0644 ../.airdc-system-packages/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

USB 权限规则已从 RealSense 官方仓库 v2.58.2 下载到上级目录，安装规则后重新插拔相机。
Docker 启动和镜像拉取需另外验证；Conda 环境创建本身不表示机械臂服务已就绪。

### 启动时报 `docker: 未找到命令`

本机 `/usr/local/bin/airbot_server` 实际通过 `docker run` 启动
`registry.cn-shanghai.aliyuncs.com/discover-robotics/airbot-runtime:5.1.6`。
该错误表示尚未能调用系统 Docker；安装 Python 包不能代替 Docker 引擎。
若尚未安装，在本机终端执行（sudo 密码仅在本机输入）：

```bash
sudo apt update
sudo apt install -y docker.io
sudo systemctl enable --now docker
sudo docker info
```

`sudo docker info` 能正常显示服务端信息后，再在两个独立终端分别启动：

```bash
# 终端 1：示教臂
sudo env -u LD_LIBRARY_PATH /bin/bash /usr/local/bin/airbot_server -i can_lead -p 50050
```

```bash
# 终端 2：执行臂
sudo env -u LD_LIBRARY_PATH /bin/bash /usr/local/bin/airbot_server -i can_follow -p 50051
```

首次启动若本地没有上述镜像，Docker 会自动下载，需要等待。镜像下载失败需单独检查网络与仓库访问。
这里通过 sudo 访问 Docker，无需修改 Docker socket 为全员可写，也无需先配置 docker 用户组。

`/bin/bash: .../envs/airdc/lib/libtinfo.so.6: no version information available`
是系统 Bash 加载 Conda 动态库时的警告，与 Docker 缺失是两个问题。
上述 `env -u LD_LIBRARY_PATH` 仅对机械臂服务启动命令清除该变量；保留
`airdc` 环境本身的配置，以继续为相机 JPEG 编解码提供动态库。
Python 遥操脚本和 `airdc` 采集程序仍在已激活的 Conda 环境中用普通用户运行。

连接相机后，可只读枚举设备：

```bash
conda activate airdc
python scripts/list_cameras.py
python -c 'import pyrealsense2 as rs; print("RealSense:", len(rs.context().query_devices()))'
```

相机枚举脚本已使用 linuxpy 0.25 的 `frame_types` 接口读取分辨率和帧率。

## 采集配置

默认 `airdc` 使用尚未生成的 `airbot_ie/configs/demonstrators/setup.yaml`。
环境安装完成后仍需按实际机械臂角色、CAN 接口、相机序列号配置设备。
不要把默认启动失败误认为 Python 环境缺失。

当前 `setup.py` 的保存逻辑仍使用旧的 `demonstrator.instance` 层级，而默认参考 YAML
使用 `demonstrator` 层级；生成真机配置前需要处理该不一致。
RealSense 的旧相机模板引用不存在的 `RealSenseConcurrent`，应参考
`airbot_ie/configs/demonstrators/realsense.yaml` 中的 `IntelRealSenseCamera`。
