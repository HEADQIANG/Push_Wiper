# Push-Wiper 模型推理与平面力位控制

入口是 `airbot_ie.scripts.push_wiper_policy_force`，配置示例是
`airbot_ie/configs/push_wiper_policy_force.json`。在线流程会先让 AIRBOT 到固定观察位 O，等待稳定并获取新相机帧，分割污渍，启动 DP 预测 16 个动作点，忽略 `delta_yaw` 后直接逐点追踪原始 XY，不做 Hermite 或线性插值，再调用当前的预定位、接触检测、二阶导纳和撤离状态机。当前配置最多执行三段，每段完成后回到 O 重新观察，达到污渍阈值即结束。当前实现只支持水平面，法向固定为 `[0, 0, 1]`，不包含曲面适配和后处理动作。

模型和机器人使用两个 Python 环境。在线入口会用 `Push-Wiper-Diffusion-Policy/.venv` 自动启动
Diffusion Policy 子进程；AIRBOT 主进程使用 AIRBOT 环境。checkpoint 已放在
`../Push-Wiper-Diffusion-Policy/checkpoints/best.ckpt`，并由服务启动时校验 EMA 权重和动作形状。

先在 `AIRBOT-Data-Collection` 目录检查配置和绑定：

```bash
cd /home/wp/yuelk_project/Push_Wiper/AIRBOT-Data-Collection
source /home/wp/yuelk_project/Push_Wiper/.venv-airdc/bin/activate
python -m airbot_ie.scripts.push_wiper_policy_force \
  --config airbot_ie/configs/push_wiper_policy_force.json --validate
```

模型服务可以单独做 checkpoint 验证：

```bash
cd /home/wp/yuelk_project/Push_Wiper/Push-Wiper-Diffusion-Policy
./.venv/bin/python -m push_wiper_dp.policy_service \
  --checkpoint checkpoints/best.ckpt --device cpu --validate
```

离线 dry-run 不连接相机、机器人或力传感器。传入导出样本 NPZ（其中必须有 `mask`）即可检查服务协议、动作安全边界和轨迹构造。没有 CUDA 时显式使用 CPU：

```bash
cd /home/wp/yuelk_project/Push_Wiper/AIRBOT-Data-Collection
MASK=$(find data/push_wiper_export_assisted_complete_20260922 -name sample.npz | head -1)
python -m airbot_ie.scripts.push_wiper_policy_force \
  --config airbot_ie/configs/push_wiper_policy_force.json \
  --device cpu --dry-run-mask "$MASK"
```

如果要直接检查采集得到的初始图片，可以使用 `--offline-image`。该入口会加载配置中的
`mask_config`，生成与在线部署相同的 mask，然后启动策略服务预测 16 点轨迹。它不会连接
相机、机器人或力传感器，也不会裁剪不安全动作：

```bash
cd /home/wp/yuelk_project/Push_Wiper/AIRBOT-Data-Collection
python -m airbot_ie.scripts.push_wiper_policy_force \
  --config airbot_ie/configs/push_wiper_policy_force.json \
  --device cpu \
  --offline-image data/push_wiper/task_20260929_172852_a15cb1db6970/segment_20260929_172854_b2b7fcce3774/before.png \
  --offline-output-dir data/push_wiper_policy_force/offline_initial_20260929
```

`--offline-output-dir` 必须是不存在的新目录。当前 `plane.replay_yaw=false`，因此保持工具姿态不随
策略的 `delta_yaw` 变化。轨迹只保留原始 16 个水平点；`plane.duration_s=30.0` 仅用于报告中的
名义时间标记，不再限制真实执行时间，也不会强制每点停留两秒。输出包括：

- `input.png`：输入图片；
- `mask.png`：黑色为污渍、白色为干净区域；
- `mask_preview.png`：红色叠加污渍区域；
- `trajectory.png`：16 个策略点、表示执行顺序的连线和工作空间；连线不是下发的插值轨迹；
- `trajectory_xy.json`：原始 16 个控制点和 16 个同位置样本，以及 `tracking_mode=waypoints`、`execution_timing=feedback`；
- `prediction.json`：完整动作、mask 统计、动作安全指标和违规原因。

命令返回码为 `0` 表示动作和水平目标均通过安全检查；返回码为 `2` 表示预测已生成但
不能执行。返回码为 `2` 时仍会保留全部报告，不能把输出直接发送给机器人。`--seed` 可以
覆盖配置中的策略随机种子，以便复现同一张图片的预测。

如果已有策略报告而不需要重新启动 DP，可以直接从其中的 `actions` 构造当前配置的 XY
轨迹：

```bash
cd /home/wp/yuelk_project/Push_Wiper/AIRBOT-Data-Collection
python -m airbot_ie.scripts.push_wiper_policy_force \
  --config airbot_ie/configs/push_wiper_policy_force.json \
  --offline-prediction data/push_wiper_policy_force/live_20260930_170031/segment_0000/prediction.json \
  --offline-output-dir /tmp/push_wiper_waypoints_check
```

该命令只输出 `trajectory_xy.json` 和 `trajectory.png`，不会连接机器人或力传感器。

## 16 点追踪与导纳保持

- `FORCE_HOLD` 按点 0 到点 15 顺序直接发送对应绝对 XY；不插值，也不把目标限制在反馈位置前方 0.3 mm。
- 每个控制周期仍读取力、检查力上限和传感器时效、更新导纳，再发送 `Z参考 - 导纳位移`。
- 实际 XY 与当前点的距离不超过 `safety.waypoint_xy_tolerance_m`（默认 2 mm）后才切换下一点。
- 每点最多等待 `safety.waypoint_timeout_s`（默认 15 秒）；超时进入 `FAULT` 并执行现有安全抬升，不跳点或补发终点。
- 最后一点也处在同一个导纳闭环中，到位后直接进入 `RETRACT`；不调用旧的非力控终点补发函数。
- `control.max_tangential_step_m` / `max_tangential_speed_m_s` 的 Python 小步目标限制不用于此逐点模式。
  点间运动由 AIRBOT 的位置伺服及其速度、加速度限制完成；应用层不保证点间实际路径或 30 mm/s 实际速度。
  真机运行前复核服务端速度参数及 `safety.max_action_step_m`，先在低速、可安全撤离的工位验证，不要同时运行拖拽预览。

运行命令不变：先执行 `--validate` 和离线 16 点报告检查，再按照下面的真机步骤启动。
`force.csv` 新增 `waypoint_index`（从 0 开始）、`target_x_m`、`target_y_m`、`xy_error_m`、
`command_x_m`、`command_y_m`、`fz_magnitude_n`，用于对照目标、真实反馈和持续力控状态。

不连接硬件的逐点控制回归测试：

```bash
cd /home/wp/yuelk_project/Push_Wiper/AIRBOT-Data-Collection
source install/activate_airdc.sh
PYTHONPATH=. python -m unittest discover -s tests -p 'test_force_hybrid_waypoints.py' -v
```

测试使用模拟反馈和虚拟时钟，覆盖 16 点顺序、到位切换、最后一点导纳保持、
单点超时、力超限、传感器失效及命令拒绝，不会连接或移动真机。

2026-09-30 离线验证：配置检查和历史 `segment_0000/prediction.json` 的报告重建通过，
控制点与样本均为原始 16 点，没有插值点。相关测试通过；原有
`tests/test_force_hybrid_timing.py` 中四项测试在修改前已因模拟接触力未达到接触阈值而
出现 `Contact approach timed out`，本次未修改这些测试或降低真实接触阈值。
以上仅为离线验证，不代表已完成真机速度、接触力或轨迹跟踪验收。

如果模型动作越过配置的 XY 工作空间，在线执行路径会拒绝整段并保留报告，不会裁剪动作。`max_action_step_m`、`max_action_yaw_rad` 和 `max_abs_yaw_rad` 为正数时分别限制 XY 单步距离、偏航单步和绝对偏航，设为 `null` 时禁用对应检查；当前配置将这三项设为 `null`。动作中的 `x,y` 是基座坐标系下相对于训练参考的绝对位置；`delta_yaw` 是相对于 O 姿态基座 Z 轴偏航增量，单位为弧度。当前 `replay_yaw=false`，因此轨迹构造会忽略该增量并保持固定姿态；`plane.tool_normal_axis="x"` 与当前工具安装方向一致，`align_tool_z=true` 会让该工具轴朝向平面内部。

在线执行还启用了与 JSON/NPZ 复现一致的预定位：先回到 `capture_reference_pose`，再在高位移动到轨迹首点 XY，完成姿态切换后下降到安全高度，最后进入接触和导纳控制。相关参数位于 `trajectory` 配置段。

真机运行前确认 references 中的 O 姿态、工作高度、相机序列号和 AIRBOT endpoint 与当前设备一致，并把配置中的工作空间、力阈值、传感器端口按实际工位复核。先启动 AIRBOT 服务和力传感器，确认相机没有被其他程序占用，再运行：

```bash
# 终端 1：保持 AIRBOT 服务运行（根据实际 CAN 接口替换 can_follow）
sudo env -u LD_LIBRARY_PATH /bin/bash /usr/local/bin/airbot_server \
  -i can_follow -p 50051

# 终端 2：可选的力传感器通信检查
cd /home/wp/yuelk_project/Push_Wiper/AIRBOT-Data-Collection
source install/activate_airdc.sh
scripts/start_force_hybrid.sh sensor-check
```

然后在终端 2 中启动在线总流程：

```bash
cd /home/wp/yuelk_project/Push_Wiper/AIRBOT-Data-Collection
source install/activate_airdc.sh
python -m airbot_ie.scripts.push_wiper_policy_force \
  --config airbot_ie/configs/push_wiper_policy_force.json
```

每次运行会创建独立目录：

```text
data/push_wiper_policy_force/live_YYYYMMDD_HHMMSS/
  segment_0000/
    input.png
    mask.png
    mask_preview.png
    prediction.json
    trajectory_xy.json
    trajectory.png
    force.csv
  run_summary.json
```

`prediction.json` 保存 16 点动作、污渍统计、动作安全检查、水平目标工作空间检查、追踪模式和最终控制器状态；`trajectory_xy.json` 的点只包含 `t_s`、`x_m`、`y_m`，不包含姿态 yaw，`t_s` 是名义标记而不是点切换条件。`gathering.frame_timeout_s` 控制等待新图像的超时，`observation_settle_s` 控制到达 O 后的稳定时间。

配置中的 `sensor.rezero_after_preposition=true` 会在姿态切换和高位 XY 预定位完成后、接触前重新采集力传感器零偏，避免工具姿态变化带来的重力投影被误判为接触力。

力控适配器每次连接 AIRBOT 时会显式设置力控伺服参数，不调用 SDK 的
`SpeedProfile.DEFAULT`：线速度缩放 `0.1`、旋转速度缩放 `0.3`、关节速度缩放 `0.1`、
最大速度缩放 `0.5`、最大加速度缩放 `0.1`。这些设置会覆盖服务端上一次运行遗留的参数。

`clean_threshold` 和 `max_segments` 必须在配置中显式填写。当前 `max_segments=3`；单段真机验证时可临时改为 `1`。流程会在每段完成并回到 O 后重新拍摄、推理和执行，直到达到污渍比例阈值或最大段数。动作、水平点到位超时、姿态、力传感器或导纳故障都会停止流程；`HybridRunner` 会先执行现有撤离逻辑，随后关闭设备。
