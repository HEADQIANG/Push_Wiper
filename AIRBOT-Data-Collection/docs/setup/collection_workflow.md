# 本仓库数据采集操作流程

本文按当前代码整理，主要示例为 AIRBOT Play 一台示教臂控制一台执行臂，加一路 USB 环境相机。双臂遥操、拖动示教和 RealSense 的差异见后文。配置示例必须按实际设备修改；无硬件检查通过不代表机械臂和相机已经联调通过。

无夹爪海绵工具、腕部 RealSense、分段记录和按键精确复位，请使用
[Push-Wiper 专用采集流程](push_wiper_collection.md)。本页通用命令不包含这些分段功能，不能与专用控制器同时运行。

## 1. 激活本机环境

后续命令均从仓库根目录运行。新开 Python 终端也需要激活环境。

```bash
cd /home/wp/yuelk_project/Push_Wiper/AIRBOT-Data-Collection
source install/activate_airdc.sh
python scripts/check_collection_env.py
airdc --help demonstrators=test
```

自检使用合成数据验证模块导入、JPEG 编解码和 MCAP 写入读回，不连接机械臂。`--help` 仅查看配置，不启动采集。环境缺失或重建方法见 [本机环境说明](local_collection_environment.md)。

真机还需要 AIRBOT SDK、可用的 `airbot_server`、Docker 服务、CAN 及相机权限。检查：

```bash
command -v airbot_server
systemctl is-active docker
ip -brief link
```

如果 Docker 尚未安装，按 [本机环境说明](local_collection_environment.md) 中的安装与启动步骤完成系统依赖安装，并确认 `sudo docker info` 能显示服务端信息，再继续。出现 `docker: 未找到命令` 时不能直接进入遥操步骤。

## 2. 连接并确认设备

一控一情况下，拔掉机械臂数据线，再依次连接示教臂、执行臂。首次绑定执行：

```bash
sudo bash ./airbot_ie/scripts/bind_can_udev.sh --target can_lead can_follow
```

这里显式使用 `sudo bash` 读取脚本。若此前直接执行脚本提示“找不到命令”，先确认当前目录为仓库根目录，并运行 `ls -l ./airbot_ie/scripts/bind_can_udev.sh` 检查文件是否存在；然后使用上面的命令。若 Bash 也提示文件不存在，应先修正路径或补齐仓库文件。

拔插两条机械臂数据线后，运行 `ip -brief link`，应能看到 `can_lead`、`can_follow`。已有正确绑定时不用重复绑定。接口名称与后续服务启动命令必须一致。

连接相机，枚举设备：

```bash
python scripts/list_cameras.py
```

假设实际采集相机为 `/dev/video2`，可预览：

```bash
python scripts/multi_capture.py 2 -ff MJPEG
```

`2` 必须替换为真实设备号，分辨率和格式必须为相机支持的组合。确认画面能覆盖夹爪和操作物体，随后退出预览，释放相机。

## 3. 手工建立采集配置

当前根配置 `airbot_ie/configs/config.yaml` 默认加载 `demonstrators: setup`，但本仓库尚无 `setup.yaml`。不要直接执行裸 `airdc` 并把配置缺失误判为依赖问题。

旧文档中的自动配置命令 `python airbot_ie/scripts/setup.py` 暂不能直接作为可靠路径：其保存代码读取 `config["demonstrator"]["instance"]`，而当前参考配置使用 `demonstrator`。因此本流程使用手工配置。

创建 `airbot_ie/configs/demonstrators/push_wiper.yaml`，内容如下。若已有同名配置，先检查并保留原内容。

```yaml
# @package _global_

defaults:
  - basis

demonstrator:
  auto_control:
    groups: []
  components:
    groups: ["/", "/", "/"]
    names: [lead, follow, env_camera]
    roles: [l, f, o]
    instances:
      - _target_: airbot_ie.robots.airbot_play.AIRBOTPlay
        port: 50050
        components: [arm, eef]
      - _target_: airbot_ie.robots.airbot_play.AIRBOTPlay
        port: 50051
        components: [arm, eef]
      - _target_: airdc.common.devices.cameras.v4l2.V4L2Camera
        camera_index: /dev/video2
        width: 640
        height: 480
        fps: 30
        rgb_camera:
          pixel_format: MJPEG
```

其中 `l` 为示教臂，`f` 为执行臂，`o` 为相机等观察设备。四个列表必须逐项对应且长度相同。`basis` 提供 `GroupedDemonstrator` 类型。`auto_control.groups: []` 表示使用后面的独立遥操脚本。

根据实际设备修改 `camera_index`、分辨率、帧率和机械臂服务端口。多相机时建议用枚举输出中的 USB bus 信息代替易变的 `/dev/videoN`；该相机实现支持包含 `usb` 的 bus 字符串。没有夹爪的机械臂设置 `components: [arm]`，并核对遥操脚本对末端的处理。

仅检查配置组合是否成功：

```bash
airdc --help demonstrators=push_wiper
```

这一步不会打开设备，也不验证实际连接。

## 4. 分别启动机械臂服务和遥操

本机用 sudo 启动 Docker 包装脚本，并仅对该命令清除 Conda 的 `LD_LIBRARY_PATH`，避免系统 Bash 的 `libtinfo` 警告。Python 遥操和采集仍使用普通用户及已激活的 Conda 环境。首次启动可能需要等待 Docker 下载机器人运行镜像。

终端 1，示教臂服务：

```bash
sudo env -u LD_LIBRARY_PATH /bin/bash /usr/local/bin/airbot_server -i can_lead -p 50050
```

终端 2，执行臂服务：

```bash
sudo env -u LD_LIBRARY_PATH /bin/bash /usr/local/bin/airbot_server -i can_follow -p 50051
```

等待两边服务就绪。终端 3，在仓库根目录激活 `airdc` 环境后启动遥操：

```bash
python airbot_ie/scripts/task_follow.py -lp 50050 -fp 50051
```

该脚本启动时可能把执行臂移动到示教臂的关节位置，随后将示教臂设为重力补偿模式、执行臂设为伺服模式。启动前留出机械臂运动空间。脚本内有固定夹爪比例 `0.072 / 0.0471`，更换夹爪型号时需要核对适配情况。

先小幅操作示教臂，确认执行臂和夹爪跟随正确。两个服务和遥操脚本在整个采集期间保持运行。

## 5. 启动采集

终端 4，在仓库根目录激活环境后执行：

```bash
airdc demonstrators=push_wiper \
  dataset.directory=push_wiper/session_001 \
  update_rate=20 \
  sample_limit.size=2000
```

这里目标采样率为 20 Hz，单条轨迹上限 2000 帧，按目标频率约 100 秒。根配置原默认值为 1000 帧，约 50 秒；实际耗时受采样速度影响。默认 `SelfManager` 达到上限会自动保存当前轨迹，因此上限应覆盖完整任务。

默认使用 OpenCV 预览和 `pynput` 键盘监听，建议在本机图形桌面运行；纯 SSH/无显示环境可能无法使用这套默认交互方式。

等设备初始化及 `Warming up...` 完成，确认预览正常。程序刚启动只是在预览，按空格才开始记录一条轨迹。

## 6. 每条轨迹的操作

1. 将物体、工具和机器人摆到本次任务的初始状态。
2. 按 **空格** 开始记录。
3. 操作示教臂完成一次完整任务，保持动作自然、连续，物体和夹爪交互始终在画面内。
4. 成功则按 **s**，等待终端打印 `Saved to ...`。
5. 复位场景，再按空格开始下一条。复位过程放在两条轨迹之间。

| 按键 | 作用及适用时机 |
| --- | --- |
| 空格 | 待采集状态下开始新轨迹 |
| s / Enter | 正在采集时保存本条轨迹并返回待采集状态 |
| q / Shift | 正在采集时放弃本条轨迹 |
| r | 待采集状态下删除上一条已保存轨迹，不是撤销键 |
| i | 重新打印按键说明 |
| Esc / z | 待采集状态下退出程序 |

按键以当前代码 `airdc/managers/keyboard.py` 和默认状态机为准。退出前先按 `s` 保存或 `q` 放弃；不要把 `Esc` 或 `Ctrl+C` 当作保存操作。达到帧数上限自动保存后，需重新按空格才会记录下一条。

按键不生效时检查是否处于对应状态、桌面键盘监听是否可用；如果误按 F2 锁定了管理器，再按 F2 解锁。采集用的是键盘监听，应避免在其他窗口打字时误触快捷键。

## 7. 数据位置与质量检查

上述命令默认写入仓库下：

```text
data/push_wiper/session_001/
├── 0.mcap
├── 1.mcap
└── ...
```

每个文件是一条 episode。内容由配置决定，包括示教/执行臂关节与末端观测、相机图像、时间戳以及设备信息。默认彩色图像以 H.264 视频附件保存在 MCAP 中；深度需另外启用。输出不是直接可训练的 LeRobot 数据集。

默认 `sample_limit.start_round=-1` 根据现有文件数量确定起始编号，不是取最大文件编号再加一。建议每次采集使用新的 `session_002` 等目录；存在编号缺口的目录应人工核对并显式设置一个未使用的 `sample_limit.start_round`，避免覆盖。

先录一条短轨迹，回看确认质量，再批量采集。安装了独立 MCAP CLI 时执行：

```bash
mcap info data/push_wiper/session_001/0.mcap
mcap list attachments data/push_wiper/session_001/0.mcap
mcap doctor data/push_wiper/session_001/0.mcap
```

这些命令检查话题、附件和文件完整性；仍需回看画面、关节运动和时间对应关系。AIRBOT MCAP Data Viewer 的使用见 [可视化说明](../visualize/airbot.md)，查看器需另行获取。Foxglove 不能直接播放默认 MCAP 视频附件，需按 [Foxglove 说明](../visualize/foxglove.md) 提取视频。

全部采集完成后，先在待采集状态退出 `airdc`，再用 Ctrl+C 停止遥操脚本，最后依次停止机械臂服务。

## 8. 其他硬件组合

### 单臂拖动示教

只连接一条执行臂，服务启动命令为：

```bash
sudo env -u LD_LIBRARY_PATH /bin/bash /usr/local/bin/airbot_server -i can_follow -p 50050
```

CAN 未绑定时可按实际接口使用 `can0`。不运行 `task_follow.py`。上述配置的 `components` 改为以下内容，相机参数仍按实际修改：

```yaml
components:
  groups: ["/", "/"]
  names: [follow, env_camera]
  roles: [l, o]
  instances:
    - _target_: airbot_ie.robots.airbot_play.AIRBOTPlay
      port: 50050
      components: [arm, eef]
    - _target_: airdc.common.devices.cameras.v4l2.V4L2Camera
      camera_index: /dev/video2
      width: 640
      height: 480
      fps: 30
      rgb_camera:
        pixel_format: MJPEG
```

这里被人拖动的机械臂在采集器中用 `l` 角色，即使名字是 `follow`。启动采集程序后，再通过臂上按钮按设备操作说明进入重力补偿模式，按空格录制并直接拖动该臂完成任务。退出后通过臂上按钮退出重力补偿模式，否则下次可能无法正常启动。

### 双臂遥操（二控二）

按左示教、左执行、右示教、右执行的顺序连接并首次绑定：

```bash
sudo bash ./airbot_ie/scripts/bind_can_udev.sh --target can_left_lead can_left can_right_lead can_right
```

拔插确认后，分别在四个终端启动服务：

```bash
sudo env -u LD_LIBRARY_PATH /bin/bash /usr/local/bin/airbot_server -i can_left_lead -p 50050
sudo env -u LD_LIBRARY_PATH /bin/bash /usr/local/bin/airbot_server -i can_left -p 50051
sudo env -u LD_LIBRARY_PATH /bin/bash /usr/local/bin/airbot_server -i can_right_lead -p 50052
sudo env -u LD_LIBRARY_PATH /bin/bash /usr/local/bin/airbot_server -i can_right -p 50053
```

这四条是四个独立终端各执行一条，不是在同一终端顺序等待。遥操命令为：

```bash
python airbot_ie/scripts/task_follow.py -lp 50050 50052 -fp 50051 50053
```

配置前四个组件的 `groups` 为 `[left, left, right, right]`，`names` 为 `[lead, follow, lead, follow]`，`roles` 为 `[l, f, l, f]`，`instances` 对应四个端口。每增加一个相机，四个列表各追加一项；相机组可用 `/`，角色用 `o`，名称须能区分环境相机及左右腕部相机。

### RealSense 彩色与深度

只读枚举序列号：

```bash
python -c 'import pyrealsense2 as rs; print([(d.get_info(rs.camera_info.name), d.get_info(rs.camera_info.serial_number)) for d in rs.context().query_devices()])'
```

将对应相机的实例替换为以下内容，序列号保留引号：

```yaml
- _target_: airdc.common.devices.cameras.intelrealsense.IntelRealSenseCamera
  camera_index: "替换为真实序列号"
  width: 640
  height: 480
  fps: 30
  enable_depth: true
  align_depth: true
```

不需要深度时设置 `enable_depth: false`、`align_depth: false`。使用当前 `IntelRealSenseCamera` 类，不要沿用旧模板中的 `RealSenseConcurrent`。也可单独运行 `airdc demonstrators=realsense dataset.directory=camera_test` 测试相机采集，但这不会记录机械臂。

## 9. 验证范围

本流程依据当前配置、键盘管理器、状态机、遥操脚本和 MCAP 采样器整理。环境自检与配置解析不涉及真机运动；真实 CAN 通信、机械臂跟随、相机支持的格式和持续采集质量需要接入实际设备后逐项确认。
