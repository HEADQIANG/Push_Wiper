# AIRBOT Play + LFS-6D65 力位混合控制

该入口支持固定 XY、拖拽后 Y 轴往返和 NPZ 平面轨迹，并统一使用 Z 方向法向力保持，
用于验证 LFS-6D65、Airbot 位姿伺服和二阶导纳控制。

## 安装

在 AIRBOT-Data-Collection 目录执行：

```bash
cd /home/wp/yuelk_project/Push_Wiper/AIRBOT-Data-Collection
source install/activate_airdc.sh
python -m pip install --no-build-isolation -e '.[force_control]'
```

确认 Airbot SDK 已安装，并确认 LFS-6D65 串口属于当前用户：

```bash
python -c 'import airbot_py; print(airbot_py.__file__)'
ls -l /dev/ttyUSB0
```

传感器协议使用 115200、8N1、Modbus 从站地址 1。关闭传感器原上位机，避免串口被占用。

## 启动 Airbot 服务

根据实际 CAN 名称启动执行臂服务。默认配置使用端口 50051：

```bash
sudo env -u LD_LIBRARY_PATH /bin/bash /usr/local/bin/airbot_server \
  -i can_follow -p 50051
```

保持该终端运行。

## 配置与无硬件检查

修改 [`airbot_ie/configs/force_hybrid_push.json`](../../airbot_ie/configs/force_hybrid_push.json) 中的：

- `sensor.port`：LFS-6D65 实际串口；
- `sensor.timeout_s`：Modbus 串口响应超时，默认 1 s；
- `sensor.zero_settle_s`：硬件清零后等待传感器恢复的时间，默认 0.1 s；
- `standalone.fixed_xy`：固定测试位置，单位米；
- `standalone.surface_z_m`：已知桌面高度，未知时填 `null`，程序使用当前高度向下一个接近深度作为初始估计；
- `force.target_force_n`：初始建议 5 N；
- `force.force_sign`：向下按压时原始 FZ 为负时保持 `-1`；
- `admittance`：二阶导纳质量、阻尼、刚度和限幅；
- `control.max_tangential_speed_m_s`：平面切向跟踪的速度上限。
- `trajectory.mode=drag_y_sweep`：启动时进入重力补偿，手动拖拽到初始位置并按 Enter
  确认工具接近桌面但不要压住桌面，然后沿基座 Y 正方向以 `trajectory.y_speed_m_s` 移动
  `trajectory.y_distance_m`，再返回初始 Y 位置；Z 方向仍由接触检测和导纳力控负责。
- `drag.enabled` 和 `drag.prompt`：拖拽示教开关及提示语。`align_tool_z=false` 时，轨迹
  始终复用拖拽确认时的末端四元数，只改变位置。

`trajectory.mode=fixed_xy` 时，程序在接触前直接沿表面法向以
`standalone.approach_speed_m_s` 下压；接触后才切换到二阶导纳位移。这样
`APPROACH` 不会把法向目标误当成导纳修正量而停在安全高度。

拖拽往返模式的配置示例：

```json
{
  "trajectory": {
    "mode": "drag_y_sweep",
    "y_distance_m": 0.10,
    "y_speed_m_s": 0.01,
    "surface_z_m": null,
    "align_tool_z": false
  },
  "drag": {
    "enabled": true
  }
}
```

正式运行时程序会先进入 `GRAVITY_COMP`，此时可以手动拖动机械臂；按 Enter 后退出
重力补偿并记录末端位姿。轨迹使用机器人基座坐标：`y0 → y0+0.10 → y0`，其中
`0.10` 单位为米，`y_speed_m_s=0.01` 表示往返轨迹的峰值速度为 1 cm/s；平滑加减速
时总轨迹时长自动为 30 秒。`align_tool_z=false` 表示沿途保持拖拽确认的姿态不变。
`surface_z_m=null` 时先从拖拽确认的当前 Z 上抬 `safe_clearance_m`，再以
  `standalone.approach_speed_m_s` 做有界接近，当前 1 mm/s、最大 50 mm 接近深度对应
  `approach_timeout_s=60`；检测到接触后将
实际接触 Z 写入轨迹参考。

不连接硬件即可检查配置：

```bash
python -m airbot_ie.scripts.force_hybrid_push \
  --config airbot_ie/configs/force_hybrid_push.json \
  --validate
```

`drag_y_sweep` 的 `--validate` 只检查配置，不会进入拖拽模式；真机运行时才会等待
Enter。若使用 NPZ 轨迹，将 `trajectory.mode` 改为 `npz` 并提供具体的 `sample.npz`
路径即可，原有 NPZ 流程不变。

## 拖拽位姿实时预览

调试拖拽方向时，可以只连接 Airbot，不连接 LFS-6D65。预览模式进入重力补偿，实时
显示末端 SDK 参考点的 X/Y/Z 曲线；窗口标题显示相对于开始位置的位移，关闭窗口或
按 `Ctrl+C` 后自动退出重力补偿。需要图形桌面和 `matplotlib`：

```bash
python -m airbot_ie.scripts.force_hybrid_push \
  --config airbot_ie/configs/force_hybrid_push.json \
  --drag-preview \
  --preview-csv data/drag_preview.csv
```

其中曲线单位为毫米，坐标方向是机器人基座坐标系；如果想提高刷新率，可增加
`--preview-hz 30`。此模式不会执行接近、力控或 X/Y 自动轨迹。

正式启动前建议先只检查力传感器，不连接也不移动 Airbot。工具保持悬空，关闭
LFS-6D65 原有 GUI 或其他串口程序后执行：

```bash
python -m airbot_ie.scripts.force_hybrid_push \
  --config airbot_ie/configs/force_hybrid_push.json \
  --sensor-check
```

成功时会打印六维力的悬空均值。若出现 `No communication with the instrument`
或 `LFS-6D65 read failed on ...`，先检查 `sensor.port`、USB 转串口连接、串口权限、
传感器供电和从站地址 1，并确保没有其他程序占用该串口；此时不要启动真机力控。

## 真机运行

开始前确认工具悬空、传感器没有接触桌面。控制器启动时会执行硬件清零，并采集约 2 秒软件零偏。

```bash
python -m airbot_ie.scripts.force_hybrid_push \
  --config airbot_ie/configs/force_hybrid_push.json
```

状态流程为：`PRECHECK → MOVE_SAFE → APPROACH → CONTACT_SETTLE → FORCE_HOLD → RETRACT`。

向下接触时，传感器原始 `FZ` 应为负，控制器将：

```text
F_normal = -(FZ_raw - bias_FZ)
```

并把 `F_normal` 控制到目标值。输出日志默认保存到：

```text
data/force_hybrid_push.csv
```

按 `Ctrl+C` 会停止控制并执行抬升。若传感器超过 0.2 秒无新数据、法向力超过 8 N、导纳位移达到 20 mm 或机器人反馈失败，程序进入故障处理。

## 法向位置与导纳积分

检测到接触后，固定 XY、拖拽 Y 往返和 NPZ 平面轨迹都将当时的实测 Z 保存为参考高度，
并清零导纳位移和速度。力保持阶段的法向目标为：

```text
z_command = z_reference - delta_n
```

`delta_n` 是相对参考高度的累计位移，正值表示下压；它不会再次叠加到当前实测位置。
因此 `delta_n` 保持 1 mm 时，目标始终在参考高度下方 1 mm；当它减小或回到零时，
目标会向参考高度回退。切向运动仍按原配置限制步长和速度。

二阶导纳采用后向欧拉积分，避免原积分方式在默认 100 Hz 参数下持续振荡。
首个力控周期使用配置周期，后续使用单调时钟测得的实际周期；接触稳定阶段结束后重新
开始周期调度，耗时超期时不集中补发过期周期。现有配置项、依赖和启动命令均无需修改。

速度上限约束导纳位移的变化率，位移上限约束相对轨迹参考的法向目标偏移；它们不代表
机器人反馈误差或物理位移的独立硬限位。有正虚拟刚度时，恒定力误差的平衡位移为
`delta_n = force_error / stiffness_n_m`，该模型本身不保证接触力稳态误差为零。

CSV 中 `position_x_m/y_m/z_m` 记录本周期发送命令前的实测位置，`command_z_m`
记录发送的目标 Z，`delta_n_m` 记录累计导纳位移。排查漂移时可对比实测位置、目标位置
和导纳位移；旧版本在接近及力保持阶段曾将目标位置写入实测位置列。

## 后续接入 Push-Wiper 平面轨迹

后续将 `trajectory.mode` 改为 `npz`，并提供现有导出目录中的
`trajectory.npz`、`duration_s` 和 `replay_yaw`。`surface_z_m` 可以显式填写；留空
时优先按样本中的 `work_height_m` 设置；没有该字段时，程序从当前机器人位姿向下
`approach_depth_m` 做有界接近，检测到接触后再把当前实测 Z 写入轨迹参考。
`capture_reference_pose[2]` 是拍摄参考高度，不会被当作桌面高度。NPZ 的 `x/y` 按仓库
`ACTION_DEFINITION` 解释，`delta_yaw` 相对 `capture_reference_pose` 叠加。NPZ 模式
默认 `align_tool_z=true`，根据基座偏航生成 TCP Z 轴朝向 `-normal` 的姿态；只有为
兼容旧回放才关闭它。轨迹模块输出期望位置、姿态和表面法向，导纳控制器无需修改。

论文的平面执行结构是“切向位置跟踪 + 法向二阶导纳修正”：论文中 TCP 的 Z 轴应
对准 `-normal`。拖拽往返模式在确认拖拽姿态后，默认自动生成 TCP Z 轴向下的姿态；
固定 XY 模式仍锁定配置姿态，要求开始测试前人工确认工具 Z 轴已经朝向桌面。当前
平面提供器使用平滑的 X 往返轨迹，执行层使用切向步长和速度上限抑制突变；真机接入
完整 ASPI 时仍需增加严格的梯形速度参数化、曲面法向和姿态生成。

## 测试

运行纯算法和轨迹接口测试（若环境未安装 pytest，可使用后面的 unittest 命令）：

```bash
cd /home/wp/yuelk_project/Push_Wiper/AIRBOT-Data-Collection
python -m pytest tests/test_force_hybrid_control.py \
  tests/test_admittance_dynamics.py tests/test_force_hybrid_timing.py -q
```

```bash
python -m unittest tests.test_force_hybrid_control \
  tests.test_admittance_dynamics tests.test_force_hybrid_timing -v
```

若当前环境只有 `python3`，将命令中的 `python` 替换为 `python3`。测试覆盖三种平面
轨迹在连续控制及反馈滞后时不重复累加法向位移、双向限幅、默认参数及变周期下的导纳
收敛、接触后的周期调度，以及实测位置与命令位置的日志区分；测试不连接硬件。

无硬件测试不能替代现场验证。真机按“传感器读取 → 空中移动 → 1–2 N → 5 N → 完整轨迹”的顺序逐步进行。
