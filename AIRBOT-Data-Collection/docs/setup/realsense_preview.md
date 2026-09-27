# RealSense D435 连续视频预览

脚本 `scripts/preview_realsense.py` 直接使用 RealSense SDK 读取相机，默认连续显示
640×480、30 FPS 的彩色视频，不保存录像。支持 D435 系列（包括本机 D435IF）。

## 在本机运行

连接相机，在图形桌面的终端执行：

```bash
cd /home/wp/yuelk_project/Push_Wiper/AIRBOT-Data-Collection
source install/activate_airdc.sh
python scripts/preview_realsense.py
```

也可以在已经初始化 Conda 的终端用 `conda activate airdc` 激活环境。
本机系统 `python3` 没有安装 `pyrealsense2`，需要先激活项目环境。
依赖安装与环境重建见 [本机环境说明](local_collection_environment.md)。

在视频窗口按 **q / Esc**、点击关闭按钮，或在终端按 **Ctrl+C** 退出。
退出时脚本释放相机；开始 AIRDC 采集前请先退出预览，避免占用相机。
画面左上角显示实际接收帧率，每两秒更新一次，启动时暂为 0。

## 同时显示彩色和深度视频

```bash
python scripts/preview_realsense.py --depth --fps 15
```

左侧为彩色图，右侧为深度伪彩色图。深度颜色由 SDK 自动映射，仅用于观察，
没有固定的颜色到米数刻度，也没有执行深度到彩色的像素对齐。

本机相机当前枚举为 USB 2.1（USB 2.0 速率），双流建议先用 15 FPS。
如需更高分辨率或帧率，使用支持 USB 3.0 的数据线并直接连接 USB 3.0 接口。

## 可选参数

以下命令均在仓库根目录、已激活的 `airdc` 环境内运行。

```bash
# 多台相机时指定序列号；序列号可从脚本启动输出中查看
python scripts/preview_realsense.py --serial YOUR_CAMERA_SERIAL

# 设置彩色流分辨率和帧率，组合必须受当前连接的相机支持
python scripts/preview_realsense.py --width 640 --height 480 --fps 15

# 无图形桌面时验证连续读取，接收 90 帧后自动退出
python scripts/preview_realsense.py --headless --frames 90

# 验证彩色和深度双流
python scripts/preview_realsense.py --depth --fps 15 --headless --frames 90

# 查看所有参数
python scripts/preview_realsense.py --help
```

不传 `--serial` 时使用 SDK 枚举的第一台 RealSense；不传 `--frames` 时持续运行。
`--headless` 不显示图像，但仍读取所选视频流并打印接收帧率。

## 常见问题

- **提示 `(.venv-airdc)`，但报“找不到命令 python”**：旧虚拟环境的激活脚本仍指向
  搬迁前的 `/media/wp/新加卷/yuelk_project/claen_wipe/Push_Wiper/.venv-airdc`，
  提示符出现环境名不代表解释器可用。本项目已改用 Conda 的 `airdc` 环境。
  在当前终端执行以下命令，退出旧虚拟环境后再启动预览：

  ```bash
  cd /home/wp/yuelk_project/Push_Wiper
  if declare -F deactivate >/dev/null; then deactivate; fi
  source AIRBOT-Data-Collection/install/activate_airdc.sh
  command -v python
  python AIRBOT-Data-Collection/scripts/preview_realsense.py
  ```

  `command -v python` 应输出 `/home/wp/miniconda3/envs/airdc/bin/python`。
  不要仅将命令替换成系统 `python3`，系统解释器没有本项目所需的相机依赖。
- **缺少依赖**：先激活 `airdc`。在其他 Python 环境只运行本脚本时，可在仓库根目录
  执行 `python -m pip install -c install/airdc.constraints pyrealsense2 opencv-python numpy`。
- **找不到相机或无权限**：检查 `lsusb` 是否有 Intel RealSense；USB 权限规则安装步骤见
  [本机环境说明](local_collection_environment.md) 的“系统与硬件前提”，安装后重新插拔相机。
- **启动失败、帧超时或画面卡顿**：退出 RealSense Viewer、其他预览/采集程序，
  检查数据线，尝试 `--fps 15`，或先去掉 `--depth` 只预览彩色流。
  SDK 不支持的分辨率/帧率会报错退出，不会静默改用其他配置。
- **没有窗口或 Qt 显示错误**：在本机图形桌面终端运行；无桌面环境使用 `--headless`。
  需要安装带 GUI 的 `opencv-python`，不能只安装 `opencv-python-headless`。

## 本机验证记录（2026-09-21）

- 已检测到 `RealSense D435IF`，序列号 `247122070601`，固件 `5.15.0.2`，USB `2.1`。
- 脚本通过 Python 语法、Ruff 代码和格式检查；OpenCV 图形窗口检查通过。
- 真机彩色流在 640×480、30 FPS 和 15 FPS 下均取帧超时；单独读取深度流、
  直接通过 V4L2 读取彩色流也超时，因此当前还没有成功验证连续视频画面。
  已通过 SDK 软重启相机并确认重新连接，重试后仍取帧超时。
- 当前系统没有安装 RealSense USB 权限规则，安装需要在本机终端输入 sudo 密码。
  先执行以下命令，再重新插拔相机；如仍超时，更换 USB 3.0 数据线/接口后重试。
  缺少权限规则是已发现的配置问题，尚不能断定它就是取帧超时的原因。

```bash
cd /home/wp/yuelk_project/Push_Wiper/AIRBOT-Data-Collection
sudo install -m 0644 ../.airdc-system-packages/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
# 重新插拔相机后：
source install/activate_airdc.sh
python scripts/preview_realsense.py --headless --frames 90
python scripts/preview_realsense.py
```
