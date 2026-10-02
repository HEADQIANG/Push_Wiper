# AIRBOT Play + 坤维/LFS 力位混合控制

该入口支持固定 XY、拖拽后 Y 轴往返和 NPZ 平面轨迹，并统一使用 Z 方向法向力保持，
默认验证坤维 KWR75、Airbot 位姿伺服和二阶导纳控制；仍保留 LFS-6D65 回退。

默认配置中的 `sensor.driver=kunwei` 使用 460800 波特率连续帧，启动时执行
1 秒本地 tare，工具必须悬空静止。切换旧传感器时设置 `sensor.driver=lfs6d65`，
其顶层 `slave_id`、`baudrate=115200` 和硬件清零配置继续有效。

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

## 从零开始的现场启动流程

下面按三个终端执行。默认配置的 `trajectory.mode` 是 `drag_y_sweep`，即先拖拽示教
起点，再沿基座 Y 轴往返；导纳控制只负责接触后的 Z 方向法向力。

### 终端 1：启动 AIRBOT 机械臂服务

先确认 CAN 接口名称。默认执行臂接口是 `can_follow`，服务端口是 `50051`：

```bash
sudo env -u LD_LIBRARY_PATH /bin/bash /usr/local/bin/airbot_server \
  -i can_follow -p 50051
```

这个终端保持运行。若实际接口不是 `can_follow`，只替换 `-i` 参数；配置文件中的
`robot.port` 必须与 `-p` 一致。该命令启动的是机械臂服务，不会启动导纳控制。

### 终端 2：准备控制环境并检查配置

```bash
cd /home/wp/yuelk_project/Push_Wiper/AIRBOT-Data-Collection
source install/activate_airdc.sh
python -m pip install --no-build-isolation -e '.[force_control]'
scripts/start_force_hybrid.sh validate
```

`validate` 不连接机械臂和力传感器，只检查 JSON 及轨迹参数。

### 终端 3：检查坤维力传感器

工具保持悬空，确认没有其他程序占用 `/dev/ttyUSB0`，执行：

```bash
cd /home/wp/yuelk_project/Push_Wiper/AIRBOT-Data-Collection
source install/activate_airdc.sh
scripts/start_force_hybrid.sh sensor-check
```

看到 `kunwei communication OK` 后才进入真机控制。该步骤执行本地 tare 并读取传感器，
不会连接或移动机械臂。

如果这里无响应，先使用只读诊断：

```bash
scripts/start_force_hybrid.sh sensor-read
```

判断方法：

- `sensor-read` 也无响应：检查 `/dev/ttyUSB0`、USB 供电/接线、串口占用和
  `sensor.kunwei.baudrate`（默认 460800）。
- 使用 LFS 时，才检查顶层 `slave_id`、`baudrate` 和硬件清零寄存器。

关闭硬件清零时必须先确认工具完全悬空，并且 `sensor-read` 输出稳定；这只是诊断和临时
运行选项，不能替代传感器厂商规定的硬件清零流程。

### 启动导纳控制并操作拖拽模式

仍在终端 3 执行：

```bash
scripts/start_force_hybrid.sh run
```

启动后控制器依次使用这些模式和状态：

1. `PRECHECK`：连接 AIRBOT，硬件清零并采集约 2 秒软件零偏。
2. `GRAVITY_COMP`：仅在 `drag_y_sweep` 模式出现。看到中文提示后，手动把工具拖到
   起始位置，保持工具悬空且姿态朝向正确，然后按一次 Enter。
3. `PLANNING_POS`：退出重力补偿，机械臂不再允许手动拖动。
4. `MOVE_SAFE`：移动到起始 XY 上方的安全高度。
5. 如果 `sensor.rezero_after_preposition=true`，在安全高度且工具仍悬空时再次执行硬件清零和
   软件零偏采集。姿态改变会改变工具重力在 FZ 轴上的投影；这一步可以避免把姿态引起的
   静态力误判成接触。
6. `SERVO_CART_POSE` + `APPROACH`：以配置的 `approach_speed_m_s` 缓慢下压，直到去偏后的
   `Fz` 绝对值达到 `contact_threshold_n`（当前配置为 5 N）并持续 `contact_confirm_s`
   （默认 0.1 s）。控制台会按 `logging.force_display_interval_s`（默认 0.2 s）显示
   `Fz_raw`、`Fz_bias_corrected`、`|Fz_bias_corrected|` 和姿态投影得到的 `normal_force`；
   完整数据同时写入 CSV。
   这个持续时间用于滤除单个传感器尖峰；此阶段还没有启用导纳位移。
7. `CONTACT_SETTLE`：保持接触约 `settle_s`（默认 0.5 秒），清零导纳位移和速度。
8. `FORCE_HOLD`：启动二阶导纳。切向按轨迹移动，Z 方向根据 `target_force_n`（默认
   5 N）修正，保持控制使用去偏后 `Fz` 的绝对值。姿态投影得到的 `normal_force` 仍会
   写入日志供诊断。
9. `FORCE_HOLD` 结束后，控制器持续重发最终轨迹位姿并检查实际水平 XY 位置；误差小于
   `force_hold_xy_tolerance_m`（当前 20 mm）即通过验收。最多等待
   `force_hold_xy_feedback_timeout_s`（当前 60 s），超时会记录警告并继续执行下一阶段。
10. `RETRACT`：沿法向抬升 `safe_clearance_m`（默认 20 mm），然后回到安全保持状态。

以上按时间采样和最终 XY 补发描述适用于独立轨迹复现入口。模型在线入口
`push_wiper_policy_force` 改为直接逐点追踪原始 16 个 XY，不插值，所有点包含最后一点
都在同一导纳闭环中验收到位，再抬升；不执行第 9 步的非力控补发。
每点误差阈值由 `safety.waypoint_xy_tolerance_m` 控制（2 mm），单点超时由
`safety.waypoint_timeout_s` 控制（15 秒）；启动、离线检查及日志说明见
`docs/setup/push_wiper_policy_force.md`。独立入口的运行命令和轨迹模式不变。

在 `GRAVITY_COMP` 阶段只允许手动拖动；按 Enter 后不要再接触机械臂。运行中按
`Ctrl+C` 会先停止伺服并抬升。传感器超时、超过 `max_force_n`（默认 8 N）、达到导纳
位移上限或位姿命令失败时，控制器也会进入抬升故障流程。

### 固定 XY 模式

如果不需要拖拽示教，把配置中的：

```json
"trajectory": {"mode": "fixed_xy"}
```

并填写 `standalone.fixed_xy`。此时运行命令不等待 Enter，程序直接读取当前姿态，移动到
配置的固定 XY，再执行 `APPROACH → CONTACT_SETTLE → FORCE_HOLD → RETRACT`。

## 独立启动入口

力位混合控制与 Push-Wiper 模型推理解耦，使用仓库内的启动脚本单独运行。脚本会固定在
`AIRBOT-Data-Collection` 目录执行，自动设置 `PYTHONPATH`，因此不受当前终端所在目录影响。
默认优先使用当前 shell 已激活环境中的 `python`，否则回退到仓库根目录的
`.venv-airdc/bin/python`；也可以设置 `FORCE_HYBRID_PYTHON` 指定解释器。

在本节其余命令所在的 `AIRBOT-Data-Collection` 目录执行：

```bash
scripts/start_force_hybrid.sh validate
```

确认配置后，按下面顺序分别执行。AIRBOT 服务仍需在另一个终端单独保持运行：

```bash
# 只检查 LFS-6D65 通信和悬空零偏，不移动机械臂
scripts/start_force_hybrid.sh sensor-check

# 可选：只连接 AIRBOT，进入重力补偿并预览拖拽轨迹
scripts/start_force_hybrid.sh drag-preview --preview-hz 30

# 正式启动独立力位控制
scripts/start_force_hybrid.sh run
```

启动其他配置时使用 `FORCE_HYBRID_CONFIG`，路径相对于
`AIRBOT-Data-Collection` 目录解析；使用其他 Python 时设置 `FORCE_HYBRID_PYTHON`：

```bash
FORCE_HYBRID_CONFIG=airbot_ie/configs/force_hybrid_push.json \
FORCE_HYBRID_PYTHON=python \
scripts/start_force_hybrid.sh run
```

脚本不会自动启动 `airbot_server`，也不会启动相机、Diffusion Policy 或旧的采集入口；
这样可以把 CAN 服务、传感器自检和力位控制分成明确的启动阶段。

## AIRBOT 服务参数

机械臂服务的启动命令和终端安排见上面的“从零开始的现场启动流程”。根据实际 CAN
名称替换 `-i` 参数，并保持服务终端持续运行；控制器通过配置中的 `robot.url` 和
`robot.port` 连接该服务。

## 配置与无硬件检查

修改 [`airbot_ie/configs/force_hybrid_push.json`](../../airbot_ie/configs/force_hybrid_push.json) 中的：

- `sensor.port`：传感器实际串口；
- `sensor.driver`：`kunwei`（默认）或 `lfs6d65`；
- `sensor.kunwei.baudrate`：坤维波特率，默认 460800；
- `sensor.kunwei.tare_duration_s`：坤维本地 tare 时长，默认 1 s；
- `sensor.kunwei.stale_s`：坤维数据最大允许陈旧时间，默认 0.2 s；
- `sensor.slave_id`：Modbus 从站地址，默认 1；
- `sensor.baudrate`：Modbus 波特率，默认 115200；
- `sensor.timeout_s`：Modbus 串口响应超时，默认 1 s；
- `sensor.zero_settle_s`：硬件清零后等待传感器恢复的时间，默认 0.1 s；
- `sensor.hardware_zero`：是否启动时发送硬件清零，默认 `true`；
- `sensor.rezero_after_preposition`：完成 NPZ 分阶段预定位后，是否在工具悬空时再次硬件清零并
  采集软件零偏，默认 `false`。NPZ 姿态改变明显时建议设为 `true`；如果工具已经接触桌面，
  不要启用此项。
- `trajectory.move_to_capture_pose`：NPZ 模式是否先移动到 `capture_reference_pose`，默认 `false`；
- `trajectory.staged_move_to_trajectory`：NPZ 是否分阶段移动到高位 XY、平滑切换姿态并下降到安全高度，默认 `false`；
- `trajectory.orientation_transition_s`：NPZ 在高位切换到首点姿态的 SLERP 时间，默认 1 s；
- `trajectory.safe_descent_s`：NPZ 保持首点姿态从高位下降到安全高度的时间，默认 1 s；
- `trajectory.high_z_m`：NPZ 分阶段预定位时 `MOVE_HIGH_XY` 和 `MOVE_ORIENTATION` 使用的高位高度，单位米。当前配置为 `0.3`；留空时沿用 `capture_reference_pose[2]`。程序会保证该高度不低于安全下降高度 `safe_z`。
- `trajectory.replay_raw`：是否分两步回放 NPZ 原始姿态。启用后，`MOVE_ORIENTATION` 先将
  配置的工具法向轴（当前为 TCP X 轴）对准竖直向下，再单独用 SLERP 调整到 NPZ 的原始
  `orientations_xyzw`；当前配置为 `true`。
- `trajectory.replay_full_orientation`：`replay_raw` 的兼容旧名称；使用 `orientations_xyzw`
  的完整姿态并用 SLERP 回放，默认 `false`。如果同时配置两个字段，以 `replay_raw` 为准；
- `trajectory.tool_normal_axis`：当 `align_tool_z=true` 时，指定哪根 TCP 轴指向桌面法向，取值为 `x`、`y` 或 `z`，默认 `z`。当前 NPZ 工具的采集首点显示 TCP X 轴才是工具工作方向，因此配置为 `x`；此时 TCP X 轴竖直向下，TCP Z 轴保持水平。
- `standalone.fixed_xy`：固定测试位置，单位米；
- `standalone.surface_z_m`：已知桌面高度，未知时填 `null`，程序使用当前高度向下一个接近深度作为初始估计；
- `standalone.contact_threshold_n`：去偏后的 `Fz` 绝对值接触阈值，当前配置为 5 N；
- `standalone.contact_confirm_s`：`Fz` 绝对值超过接触阈值后需要持续的确认时间，默认 0.1 s；
- `force.target_force_n`：初始建议 5 N；
- `force.force_sign`：传感器内向法向投影的符号修正，当前传感器保持 `-1`；该投影用于
  `normal_force` 记录，接触确认和 `FORCE_HOLD` 使用去偏后 `Fz` 的绝对值；
- `safety.pose_feedback_timeout_s`、`safety.pose_position_tolerance_m`、
  `safety.pose_orientation_tolerance_deg`：姿态伺服结束后的实际反馈校验参数。AIRBOT 的
  `servo_cart_pose` 是异步接口，调用返回不代表已经到位；控制器会读取 `get_end_pose()`，
  并在等待期间持续重发最终目标位姿，校验误差后才进入 `MOVE_SAFE`/`APPROACH`。
- `safety.orientation_feedback_timeout_s`：`MOVE_ORIENTATION` 最终位姿反馈验收的最长等待
  时间，当前为 30 s；等待期间持续重发目标位姿，超时后进入现有故障处理。
- 力控适配器连接 AIRBOT 后会显式设置本次力控伺服参数，不调用 SDK 的
  `SpeedProfile.DEFAULT`：`scale.linear=0.1`、`scale.rotational=0.3`、
  `scale.joint=0.1`、最大速度缩放 `0.5`、最大加速度缩放 `0.1`。这会覆盖服务端上一次
  运行遗留的自定义速度参数。
- `safety.vertical_axis_tolerance_deg`：`MOVE_ORIENTATION_VERTICAL` 阶段 TCP X 轴相对目标
  竖直方向的允许夹角，当前为 12°。该阶段只验收 TCP X 轴方向，完整姿态由后续
  `MOVE_ORIENTATION_RAW` 阶段验收。
- `safety.force_hold_xy_tolerance_m`：`FORCE_HOLD` 结束时实际水平 XY 位置相对最终轨迹
  目标的允许误差，当前为 0.02 m；
- `safety.force_hold_xy_feedback_timeout_s`：`FORCE_HOLD` 最终 XY 验收的最长等待时间，
  当前为 60 s。超时后记录警告并继续 `RETRACT`。
- `logging.force_display_interval_s`：`APPROACH` 阶段控制台显示力传感器读数的周期，单位秒，
  默认 `0.2`；设为更小的值会增加日志输出量。
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
scripts/start_force_hybrid.sh validate
```

`drag_y_sweep` 的 `validate` 动作只检查配置，不会进入拖拽模式；真机运行时才会等待
Enter。若使用 NPZ 轨迹，将 `trajectory.mode` 改为 `npz` 并提供具体的 `sample.npz`
路径即可，原有 NPZ 流程不变。

如果要复现 Push-Wiper 导出的 XY JSON 轨迹，将 `trajectory` 配置为：

```json
{
  "trajectory": {
    "mode": "xy_json",
    "json": "../xy_trajectory_30s/trajectory_xy.json",
    "move_to_capture_pose": true,
    "staged_move_to_trajectory": true,
    "orientation_transition_s": 2.0,
    "safe_descent_s": 2.0,
    "high_z_m": 0.3,
    "duration_s": 30.0,
    "surface_z_m": null,
    "align_tool_z": true,
    "tool_normal_axis": "z"
  }
}
```

命令从 `AIRBOT-Data-Collection` 目录启动时，`json` 路径相对于当前工作目录解析。
适配器读取 `control_points`（没有时使用 `samples`）中的 `t_s`、`x_m`、`y_m`，并将其
转换为现有 `HybridRunner` 使用的平面轨迹。导出的 JSON 同时保存
`capture_reference_pose` 和 `surface_z_m`，因此可以像 NPZ 一样执行采集位姿预定位；也可以
在 `trajectory.capture_reference_pose` 和 `trajectory.surface_z_m` 中覆盖。`duration_s` 为空时
沿用 JSON 的时间长度；填写数值时会按比例缩放时间轴。该模式不读取或重放 yaw，姿态固定为
采集参考姿态，Z 高度由 JSON/配置或接触阶段的实际高度确定，法向力仍由二阶导纳控制。

先只验证 JSON 轨迹和配置：

```bash
scripts/start_force_hybrid.sh validate
```

确认传感器和 AIRBOT 服务后，使用原有启动命令进入导纳控制：

```bash
scripts/start_force_hybrid.sh run
```

## 拖拽位姿实时预览

调试拖拽方向时，可以只连接 Airbot，不连接 LFS-6D65。预览模式进入重力补偿，实时
显示末端 SDK 参考点的 X/Y/Z 曲线，并在右侧三维窗口实时显示 TCP X/Y/Z 轴与基座
X/Y/Z 轴的方向；窗口标题还会显示三根 TCP 轴在基座坐标系中的分量。关闭窗口或按
`Ctrl+C` 后自动退出重力补偿。需要图形桌面和 `matplotlib`：

```bash
scripts/start_force_hybrid.sh drag-preview \
  --preview-csv data/drag_preview.csv
```

其中曲线单位为毫米，坐标方向是机器人基座坐标系；三维图中的彩色实线箭头是 TCP
轴，半透明箭头是平移到当前 TCP 位置的基座轴。CSV 除位置外还会保存
`tcp_x_b*`、`tcp_y_b*`、`tcp_z_b*` 三组方向分量。如果想提高刷新率，可增加
`--preview-hz 30`。此模式不会执行接近、力控或 X/Y 自动轨迹。

正式启动前建议先只检查力传感器，不连接也不移动 Airbot。工具保持悬空，关闭
LFS-6D65 原有 GUI 或其他串口程序后执行：

```bash
scripts/start_force_hybrid.sh sensor-check
```

成功时会打印六维力的悬空均值。若出现 `No communication with the instrument`
或 `LFS-6D65 read failed on ...`，先检查 `sensor.port`、USB 转串口连接、串口权限、
传感器供电和从站地址 1，并确保没有其他程序占用该串口；此时不要启动真机力控。

## 真机运行

开始前确认工具悬空、传感器没有接触桌面。控制器启动时会执行硬件清零，并采集约 2 秒软件零偏。

```bash
scripts/start_force_hybrid.sh run
```

NPZ 启用分阶段预定位时，状态流程为：
`PRECHECK → MOVE_CAPTURE → MOVE_HIGH_XY → MOVE_ORIENTATION_VERTICAL → MOVE_ORIENTATION_RAW → MOVE_SAFE → APPROACH → CONTACT_SETTLE → FORCE_HOLD → RETRACT`（启用 `replay_raw` 时）。
未启用时仍使用单个 `MOVE_ORIENTATION` 阶段。
其中 `MOVE_HIGH_XY` 保持采集参考位姿的高 Z，只移动到轨迹首点的 X/Y；
启用 `replay_raw` 时，`MOVE_ORIENTATION_VERTICAL` 在高位先将 TCP X 轴对准竖直向下，
随后 `MOVE_ORIENTATION_RAW` 单独切换到轨迹首点的原始姿态；两个阶段都持续重发目标并执行
反馈验收。未启用时，`MOVE_ORIENTATION` 直接切换到轨迹首点姿态；
`MOVE_SAFE` 保持该姿态连续下降到 `work_height_m + safe_clearance_m`，完成后才启动接触检测。
如果 AIRBOT 拒绝从当前位姿直接规划到 `capture_reference_pose`，程序会自动先抬升到高位、
再移动到参考 XY，最后用笛卡尔伺服平滑切换参考姿态。
若配置了 `sensor.rezero_after_preposition=true`，`MOVE_SAFE` 完成后会在进入
`APPROACH` 前重新清零并采集偏置；这一步不改变上述运动状态，只更新后续接触判断使用的
`bias_tool`。如果没有启用分阶段预定位，仍使用单次 `PLANNING_POS` 安全移动；未启用
`move_to_capture_pose` 时省略 `MOVE_CAPTURE`。

`normal_force` 记录值会把传感器三维力从工具坐标转换到基座坐标。
设 `R(q)` 是当前末端姿态，`n` 是基座表面外法向，工具坐标中的内向法向为
`R(q)^T(-n)`，控制器计算：

```text
F_normal = force_sign · dot(F_tool - bias_tool, R(q)^T(-n))
```

`APPROACH` 的接触确认使用去偏后 `Fz` 的绝对值，因此当前配置下 `|Fz_bias_corrected|`
需要达到 5 N 并持续确认时间。
输出日志默认保存到：

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
开始周期调度，耗时超期时不集中补发过期周期。现有配置项和依赖保持不变，启动时使用前面的独立入口即可。

速度上限约束导纳位移的变化率，位移上限约束相对轨迹参考的法向目标偏移；它们不代表
机器人反馈误差或物理位移的独立硬限位。有正虚拟刚度时，恒定力误差的平衡位移为
`delta_n = force_error / stiffness_n_m`，该模型本身不保证接触力稳态误差为零。

CSV 中 `position_x_m/y_m/z_m` 记录本周期发送命令前的实测位置，`command_z_m`
记录发送的目标 Z，`delta_n_m` 记录累计导纳位移。排查漂移时可对比实测位置、目标位置
和导纳位移；旧版本在接近及力保持阶段曾将目标位置写入实测位置列。

## 后续接入 Push-Wiper 平面轨迹

后续将 `trajectory.mode` 改为 `npz`，并提供现有导出目录中的
`trajectory.npz`、`duration_s` 和 `replay_yaw`。设置 `move_to_capture_pose=true` 后，
程序会先使用 NPZ 的 `capture_reference_pose` 移动到采集时的初始观测位置，再在该观测高度
移动到轨迹起点 XY，在高位平滑切换轨迹姿态，最后保持姿态下降到轨迹安全高度；观测位姿只
用于预定位，不会被当作桌面高度。
`surface_z_m` 可以显式填写；留空
时优先按样本中的 `work_height_m` 设置；没有该字段时，程序从当前机器人位姿向下
`approach_depth_m` 做有界接近，检测到接触后再把当前实测 Z 写入轨迹参考。
`capture_reference_pose[2]` 是拍摄参考高度，不会被当作桌面高度。NPZ 的 `x/y` 按仓库
`ACTION_DEFINITION` 解释，`delta_yaw` 相对 `capture_reference_pose` 叠加。NPZ 模式
启用 `replay_full_orientation=true` 时，优先使用导出的 `orientations_xyzw` 并按时间做四元数
SLERP，适合当前“复现已有 NPZ 示教轨迹”的模式；模型只输出 `(x,y,delta_yaw)` 时应保持
该选项为 `false`，走姿态重建路径。
代码默认 `align_tool_z=true`，并按 `tool_normal_axis` 生成法向姿态。当前配置文件使用
`tool_normal_axis="x"`：NPZ 首点的 TCP X 轴接近竖直向下，直接把 TCP Z 轴对准向下会让
刮擦工具横转，所以不能只看 TCP Z 轴判断工具是否竖直。姿态伺服完成后日志会同时输出
目标和实际 TCP X/Z 轴；如果反馈误差超过阈值，程序会在进入下降前报错。

论文的平面执行结构是“切向位置跟踪 + 法向二阶导纳修正”：论文中 TCP 的 Z 轴应
对准 `-normal`。拖拽往返模式在确认拖拽姿态后，默认自动生成 TCP Z 轴向下的姿态；如果工具工作轴是 X/Y 轴，设置同名 `tool_normal_axis`。固定 XY 模式仍锁定配置姿态，要求开始测试前人工确认工具工作轴已经朝向桌面。当前
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
