# Push-Wiper 分段采集

本文按当前 O/Z 参考登记流程更新。所有命令都在下面的实际仓库路径执行：
`/home/wp/yuelk_project/Push_Wiper/AIRBOT-Data-Collection`。
不再使用此前无法访问的 `/media/wp/新加卷/...` 路径。

本程序支持双臂主从（默认）和单臂拖动，两种方式均使用无夹爪海绵工具、腕部 D435IF 彩色流。
单臂启动与操作差异见下节；后文原有双臂步骤继续适用于默认模式。动作使用 SDK 原有末端参考点，
单位为米、弧度，四元数为 XYZW；不要求海绵 TCP、手眼或相机重新标定。
仅适用于工具安装和拍摄视角固定、机器人基座 Z 轴与桌面法向一致的平面任务。

自动测试和模拟采集不能代替真机验收；复位精度、保持行为和恢复跟随需要现场验证。
主从与复位必须由同一控制器执行，不能同时启动旧的 `task_follow.py`。
仅做通用主从、不使用本采集入口时，旧脚本可通过
`python airbot_ie/scripts/task_follow.py -lp 50050 -fp 50051 --no-eef` 跳过所有夹爪调用。
该脚本保留原有启动对齐行为，与本入口“启动不自动移动”的行为不同。

默认配置：相机 640×480、30 FPS，仅彩色；采集目标 20 Hz；复位使用低速位置规划。
不检查主从臂之间的关节差、SDK 静止速度或复位前的近位范围。
复位仍检查两臂各自相对各自目标的误差：位置 2 mm、姿态 1°、各关节 0.01 rad，
持续满足至少 0.5 秒才允许拍照，超时 10 秒。上述参数须真机验证。
海绵真实离桌距离不由程序计算，按 R 前必须人工抬起并返回无接触的观察附近区域。

## 单臂拖动模式

只连接装有海绵和腕部相机的执行臂。单臂模式使用 `robot.follow_url` / `robot.follow_port`
（默认 localhost:50051），不连接示教臂。启动该臂服务，CAN 名称按实际设备替换：

```bash
sudo env -u LD_LIBRARY_PATH /bin/bash /usr/local/bin/airbot_server -i can_follow -p 50051
```

按后文相机检查步骤确认连续取帧成功后，在另一个终端运行：

```bash
cd /home/wp/yuelk_project/Push_Wiper/AIRBOT-Data-Collection
source install/activate_airdc.sh
python -m airbot_ie.push_wiper.collect \
  --config airbot_ie/configs/push_wiper_collection.json \
  --mode single_drag \
  --references data/push_wiper_single_drag_references.json \
  --serial 247122070601
```

相机序列号仍须按实际设备修改。也可在 JSON 的 `robot` 中设置 `"mode": "single_drag"`；
不指定模式时仍为 `teleop` 双臂主从。`--mode` 优先于 JSON。其他采样、相机、复位、
输出目录、导出参数均保持原值。不启动 `task_follow.py` 或其他机械臂控制程序。

- `G`：执行臂进入重力补偿，界面显示 dragging，直接用手拖动该臂。
  拖动期间程序只读反馈，不发送位置跟随指令。
- `O`：登记这条臂的观察姿态并精确到位；`Z`：记录这条臂的 SDK 工作高度。
- `R`：退出拖动，以原有低速规划回到 O；仍要求原有位置、姿态、关节误差和稳定时长达标，
  界面显示 observing 后才可拍照。按 R 前先手动抬离桌面、返回观察附近并松手。
- 两次空格仍仅划定有效推动区间；第二次空格后继续处于拖动模式，手动抬起、清洁、返回再 R。
- `N/C/S/Q/T/Esc` 的条件、保存规则与原流程一致；退出或故障时尝试保持唯一机械臂的实测位置。

首次登记：`G → 手动到观察位 → O → observing → G → 手动贴桌 → Z → 抬起返回 → R`。
已有 O/Z 的采集顺序：
`R → observing → N → C → G → 手动贴桌 → 空格 → 拖动推动 → 空格 → 抬起/清洁/返回 → R → observing → C → S`。
同一任务的后续段从 C 开始，完成任务按 T。

两种模式使用独立参考文件；切换模式后首次重新登记 O/Z，避免误用双臂参考。
单臂原始记录只有 `follow` 机械臂数据，不生成虚构的 `lead` 数据。
数据中的 `following` 状态表示已开启手动操作，单臂界面显示为 dragging；
任务/分段配置中的 `robot.mode` 标记采集模式。动作坐标系、O 偏航基准、工作高度与导出格式不变，
继续使用本文第 12 节的导出命令。

无硬件验证与练习：

```bash
python -m airbot_ie.push_wiper.collect --mode single_drag --check-config
python -m airbot_ie.push_wiper.collect --mode single_drag --demo --output data/push_wiper_single_demo
python -m airbot_ie.push_wiper.export --input data/push_wiper_single_demo --output data/push_wiper_single_demo_export --allow-simulated
python -m airbot_ie.push_wiper.collect --mode single_drag --mock \
  --references data/mock_single_drag_references.json --output data/mock_single_drag
```

模拟窗口中，方向键、`[`/`]`、`,`/`.` 改为移动虚拟执行臂。模拟检查不能代替真机重力补偿、
工具负载、复位与保持行为的现场验证。
现有不含 mode 的双臂参考文件仍可在 teleop 模式加载；单臂参考只绑定实际连接的执行臂和相机。
仅检查当前硬件配置（不会连接设备）时，在上述启动命令末尾追加 `--check-config`。

## 1. 连接硬件与准备环境

固定双臂基座、海绵支架、腕部相机和桌面，接好 CAN 与相机 USB 线。
桌面先保持干净，完成 O/Z 登记后再布置本次采集的污渍。
不需要连接夹爪；当前入口不读取或控制夹爪，也不读取力传感器。

新开一个采集终端，进入项目并激活环境：

```bash
cd /home/wp/yuelk_project/Push_Wiper/AIRBOT-Data-Collection
source install/activate_airdc.sh
python -m airbot_ie.push_wiper.collect --config airbot_ie/configs/push_wiper_collection.json --check-config
```

`--help`、`--check-config` 和 `--demo` 不连接任何真机。
若首次安装或移动过项目目录，先执行一次以下命令，再检查配置。不需要每次采集执行：

```bash
python -m pip install --no-deps --no-build-isolation -e .
```

检查 Docker 和 CAN 接口：

```bash
systemctl is-active docker
ip -brief link
```

Docker 应为 active，接口列表中应存在 `can_lead` 和 `can_follow`，且对应正确的实体机械臂。
若实际只有 `can0`，不能直接用 `-i can_follow` 启动。先确认设备与接口映射，再处理命名。
已有正确绑定时不要重复运行绑定脚本；首次绑定见 [通用采集流程](collection_workflow.md)。
若新旧 udev 规则同时存在，应统一规则，避免重插后名称变化。

## 2. 启动双臂服务

在两个独立终端分别启动服务，并保持终端运行。已经正常运行的服务不要重复启动。

终端 1：示教臂，端口 50050。

```bash
sudo env -u LD_LIBRARY_PATH /bin/bash /usr/local/bin/airbot_server -i can_lead -p 50050
```

终端 2：执行臂，端口 50051。

```bash
sudo env -u LD_LIBRARY_PATH /bin/bash /usr/local/bin/airbot_server -i can_follow -p 50051
```

CAN 名称按实际设备修改，端口应与采集配置一致。
**不要另外启动 `task_follow.py` 或旧的 `airdc demonstrators=push_wiper`。**
本入口统一管理主从与复位；相同端点使用进程锁避免两个本项目控制器同时启动。

## 3. 验证相机并启动采集

本机此前使用的 D435IF 序列号为 `247122070601`。若更换相机，先查询并替换后续命令中的序列号：

```bash
python - <<'PY'
import pyrealsense2 as rs
for device in rs.context().query_devices():
    print(device.get_info(rs.camera_info.name),
          device.get_info(rs.camera_info.serial_number))
PY
```

在已激活环境的采集终端中，先连续读取 300 帧：

```bash
python scripts/preview_realsense.py --serial 247122070601 --headless --frames 300
```

看到 `预览结束，共接收 300 帧。` 后，再启动采集。若未找到相机或取帧超时，先处理相机连接。
退出其他预览或相机程序，避免占用设备。

```bash
python -m airbot_ie.push_wiper.collect \
  --config airbot_ie/configs/push_wiper_collection.json \
  --serial 247122070601
```

必须指定真实序列号，不自动选第一台相机。历史文档记录 D435IF 曾取帧超时，
目前代码验证不能替代 USB 连接、权限、持续视频和机械臂运动的现场验证。
RealSense 使用现有驱动的 BGR8 彩色流，与 PNG、OpenCV、MCAP 视频编码的颜色顺序一致。

单击视频窗口，使其获得焦点；O/Z/G/R 等按键都在这个窗口中按，不是在终端中输入。
字母大小写均可，每次按下后松开，尤其不要长按空格。

## 4. 首次登记 O 观察姿态和 Z 工作高度

启动后为 idle，不自动移动。在视频窗口中按 G 进入主从，不再要求两臂关节差小于某个阈值。
跟随指令从执行臂当前实测关节位置开始，按 `reset.follow_joint_speed_rad_s` 平滑接近示教臂，
默认每关节指令变化率不超过 0.5 rad/s；这是指令限速，不是主从误差限制。
首次就位与手动模式遵循设备操作说明；G 后执行臂会向示教臂当前关节位置移动。

| 顺序 | 操作 | 应看到的提示或状态 |
| --- | --- | --- |
| 1 | 按 G 开启主从，操作示教臂，将海绵抬离桌面，让相机看清完整工作区 | following |
| 2 | 人工停稳，按 O 登记观察姿态 | `Registered observation reference`，随后进入 observing |
| 3 | 等 observing 后按 G，遥操作让海绵以规定姿态和压缩程度贴合桌面 | following |
| 4 | 人工停稳，按 Z 保存工作高度 | `Registered work height: ... m (follower SDK Z)` |
| 5 | 人工抬起并返回观察位附近，按 R | resetting，随后 observing |

O 分别保存两臂各自的实测关节角、SDK 末端位置和四元数；按 O 还会启动一次复位。
之后 R 让两臂各自返回自己的目标。O 本身不保存训练图片，图片由每段的 C 操作保存。

O 保存的执行臂观察姿态是论文中的固定拍摄参考 `p_cap`，也是动作偏航增量的零点。
首次需要登记 O 观察姿态和 Z 工作高度。Z 只记录执行臂 SDK 末端参考点在基坐标系下的
Z 坐标，单位米，不是海绵下表面或桌面本身的几何高度；它不会移动机械臂、触发有效记录，
也不会按高度或倾斜筛选数据。P 按键不执行操作。
历史参考文件中的 contact 字段会被忽略，已有 O 可以继续使用，无需删除原始数据。
如果旧文件还没有 work_height，先按 G 贴桌、Z 补登记一次，再返回观察位开始新任务。
O 和 Z 均保存到默认 `data/push_wiper_references.json`（可用 `--references` 改路径），
后续任务及程序重启自动复用。界面显示保存的工作高度和当前执行臂 SDK Z，供人工参考。
更换海绵、改变工具倾斜、压缩状态、安装或桌面高度后，需重新登记并人工验证工作高度。

参考姿态保存到 `--references` 指定的 JSON。相机设置、ROI、设备端点或模拟/真机类型变化时，
程序拒绝复用不匹配的参考文件。修改支架、海绵厚度或桌面后，应使用新参考文件重新登记。
若参考文件记录的机器人序列号与当前设备不同，也会在启动控制线程之前拒绝运行。
O/Z 只允许在完整任务之间执行；任务开始后固定使用当时的参考副本，按 T 结束后才能更新。
程序不重新配置 SDK TCP，也不做手眼标定。
SDK 速度反馈仍保存在原始记录中，但不会再触发 `Wait until both arms are stationary`。
程序更新后需要退出旧采集进程，再用原命令启动。安装和工作区域未改变时，O/Z 可直接复用。
旧的 `near_*`、`aligned_joint_rad`、`register_joint_rad`、`still_velocity_rad_s` 配置项已删除；
使用自定义配置时请移除这些项。默认模板已更新，启动命令不变。
自定义配置中的 `contact_z_tolerance_m`、`contact_tilt_tolerance_deg` 也需删除；
默认配置已同步删除。退出旧采集进程后，用原命令重启即可使用简化流程。

## 5. 后续启动时复用参考

若界面显示 `O observation=ready`，并且 `Z work height` 显示具体数值，说明参考已经加载。
无需重复 O/Z；只更换污渍分布也无需重新登记。

1. 程序重启后处于 idle，不自动恢复跟随。
2. 如果两臂已经在观察位附近且回程无接触，按 R，等待 observing。
3. 如果需要先移动，按 G，通过示教臂人工抬起海绵、返回观察位附近，再按 R。
4. 在观察保持状态布置好当前污渍，按下面流程创建新任务。

旧文件若仅缺 Z，先按 G 人工贴桌、按 Z，再抬起、返回、R；已有 O 不用重录。
如果要采用另一套固定参考，可以在启动命令中增加
`--references data/push_wiper_references_table2.json`，之后始终使用该路径启动。

## 6. 完成一个推动段

“任务”指从一次初始污渍分布开始的完整清洁过程；一个任务可含多个“推动段”。
开始任务前，先在 observing 状态布置污渍，确认画面中的区域完整可见。

| 顺序 | 操作 | 结果及确认方式 |
| --- | --- | --- |
| 1 | 按 N | 出现 `New task created`，新建完整任务 |
| 2 | 按 C | 出现 `Captured image: approach`，保存推动前图，开始原始记录 |
| 3 | 按 G | 进入 following，恢复主从 |
| 4 | 人工移动到选定起点上方，再下降贴桌 | 参考界面的已保存工作高度与当前 SDK Z，人工确认接触 |
| 5 | 尚未水平推动时，按第一次空格 | 出现 `Stroke state: pushing`，有效动作开始 |
| 6 | 遥操作完成一段水平推动 | 此时的执行臂实际轨迹进入动作标签 |
| 7 | 抬起前按第二次空格 | 出现 `Stroke state: return`，有效动作结束 |
| 8 | 人工抬起、清洁海绵、返回观察位附近 | 主从仍继续；这些动作只进入原始记录 |
| 9 | 按 R，等待界面显示 observing | 暂停跟随、双臂复位并保持 |
| 10 | 按 C | 出现 `Captured image: review`，保存推动后图 |
| 11 | 按 S | 出现 `Saved: segment_...`，接受并提交本段 |

`Command completed: reset` 只表示复位请求已经处理；应以界面状态 **observing** 为拍照条件。
第二次空格既不停止主从，也不提交文件；本段正常保存的最后一步是 S。
原始记录从第一次 C 开始，持续到 S/Q 或中断处理；训练动作只截取两次空格之间的数据。

按第一次空格之前应已贴桌、尚未水平推动。不要在空中提前按空格，也不要已经擦掉一部分
污渍才开始有效记录。可以从污渍边缘外的干净位置贴桌，再开始推动，减少进场对污渍的影响。
Z 只是保存的工作高度，不会自动下降、自动判断接触或自动触发空格；当前程序也没有力位混合控制。

## 7. 下一段、下一任务与退出

- **同一任务继续推动：**S 后仍在 observing，保持当前污渍分布，直接从 C 拍下一段起始图开始。
  不再按 N，也不用重新 O/Z。完整循环为
  `C → G → 人工贴桌 → 空格 → 推动 → 空格 → 抬起/清洁/返回 → R → observing → C → S`。
- **拒绝当前段：**按 Q，仍会保留原始记录并标记 rejected。Q 不停机械臂；若还在 following，
  人工抬起并返回后按 R，等 observing 再用 C 拍当前状态，开始新段。失败后也不能沿用旧起始图。
- **结束完整任务：**先 S 接受或 Q 拒绝当前段，再按 T，看到 `Task completed`。
  换一批污渍或重新摆放场景后，按 N 创建新任务，继续 C 开始采集。
- **退出程序：**完成 S/Q、T 后按 Esc。程序尝试保持双臂当前位置并退出；若有未提交段，
  则尝试按 incomplete 保存。下次启动可以复用 O/Z，但会创建新任务，不自动续接上次任务。

## 8. 按键速查

| 按键 | 生效条件及行为 |
| --- | --- |
| O | 任务之间登记观察姿态并复位 |
| Z | 任务之间保存执行臂当前 SDK 工作高度，可重复登记更新 |
| N | O 观察参考和 Z 工作高度已登记、没有其他任务时，新建任务 |
| R | 未在有效推动中且没有未完成的进场/审核段时，低速精确复位；不检查近位范围或静止速度 |
| C | observing 下拍起始图并开始原始记录；有效推动结束且再次复位后拍结束图 |
| G | idle/observing 下恢复主从，不限制两臂之间的关节差；按 C 后用它进入推动准备 |
| 空格 | 第一次开始有效推动，第二次结束有效推动；主从继续工作 |
| S | 起始图、起止标记和结束图齐全时接受该段 |
| Q | 拒绝当前段并保留原始数据；不会直接停机械臂 |
| T | 没有待处理段时结束当前完整任务 |
| Esc | 退出，未完成段保存为 incomplete，尝试保持两臂当前位置 |

已有 O/Z 的标准顺序：`R → observing → N → C → G → 人工贴桌 → 空格 → 推动 → 空格 → 人工抬起/清洁/粗返回 → R → observing → C → S`。
所有按键仅在视频窗口获得焦点时生效，大小写均可；界面显示状态、位姿误差与当前段采样数。
界面显示 O 是否 ready，以及 Z 保存的工作高度。若 N 提示 `Missing reference: observation (O)`，
先按 G 移到观察位、按 O 登记，等待 observing 后再按 N、C。
若提示 `Missing work height (Z)`，先按 G 人工贴桌、按 Z，再抬起并返回、按 R；
已有 O/Z 时，每个新任务无需重新登记，直接按正常 N/R/C 流程操作。
只有看到 `New task created` 才表示 N 成功。
每个按键必须单独按下释放，按住重复空格可能产生过短推动段；导出会检查时间与样本数。

## 9. 常见提示、故障恢复与现场验收

| 提示或现象 | 处理方式 |
| --- | --- |
| `Missing reference: observation (O)` | 尚未登记 O，任务未创建；在任务之外登记观察姿态 |
| `Missing work height (Z)` | 已有 O 但缺工作高度；G 人工贴桌、Z，抬起返回后 R |
| `Create a task with N first` | 前一次 N 未成功，先补齐参考，再按 N 确认创建成功 |
| `Reference changes are only allowed between tasks` | 完成本段 S/Q，再 T；然后才能重新 O/Z |
| `Press R and wait for precise observation hold` | 尚未 observing，先结束推动或处理当前段，再按 R 并等到位 |
| `Observation hold outside tolerance` / settling | 观察位反馈暂时超差，暂停拍照；松手等待重新稳定，或按 G 开启手动操作、按 R 重新复位 |
| `Observation hold drifted out of tolerance; recovery timed out` | 在原有 `reset.timeout_s` 内未重新稳定，程序停止并尝试保持实测位置；根据报错中的位置/姿态/关节误差检查工具负载、外力和 O 登记姿态 |
| `Control command timed out` | G/R/O 的 SDK 调用未在命令时限内返回；报错包含命令、耗时、最后状态和反馈年龄，检查对应 airbot_server 终端及 CAN 通信 |
| `Control loop is unresponsive` | 非命令执行期间超过反馈时限未更新状态；报错包含状态、状态年龄和时限，检查 SDK、服务进程及主机负载 |
| `Failed to get CAN interface index` | 对照 `ip -brief link` 检查服务命令使用的接口是否存在 |
| `Another controller owns ...` | 退出仍在运行的旧采集或跟随程序，避免两个控制器同时工作 |
| `getcwd` 或相对路径不存在 | 重新 cd 到本文开头的实际项目目录 |

复位使用低速位置规划，按键后两臂会运动。由于不再拦截远离目标的复位请求，操作者须先抬离桌面、
人工返回观察位附近，并确认回程无接触或障碍。失败后不自动恢复主从，程序尝试将仍可通信的臂
切到关节伺服并保持实测位置，然后退出。断连或服务无响应时不能保证软件保持成功，应检查真机状态。
SDK 5.1.6 的异步反馈缓存通过消息更新监测停滞，采集时间仍属于主机读取时间而不是硬件同步。
自动复位必须现场验证模式切换能中止规划、两臂保持有效，且 G 恢复后无跳动。

观察位保持检查仍使用原有 2 mm、1°、0.01 rad 阈值。旧版一帧超差就退出；
现在先进入 settling 并立即禁止 C 拍照，在原有 10 秒超时内，误差连续满足原有
0.5 秒稳定时长后才恢复 observing，并要求相机取得恢复后的新帧。
等待期间不重新规划、不修改 O，不额外发送运动指令；持续超差或通信故障仍会停止。
如需拖动，应先按 G 等待 dragging（双臂为 following），不要在 observing 下直接拉动机械臂。
`hold failures: {}` 仅表示故障后的保持请求没有报告失败，不代表观察位误差为零。
启动命令、登记步骤和参数均不变；退出旧程序后按原命令重启。

控制线程的普通反馈超时仍为 `robot.feedback_timeout_s`（默认 0.5 秒）。SDK 的模式切换、
速度配置和复位请求会同步等待服务返回，即使复位使用 `blocking=False` 也要等待请求被接受。
G/R/O 执行中改用独立的 `robot.command_timeout_s`（默认 10 秒），界面显示等待命令，
期间暂停采样和其他操作，不把旧反馈重复写为新样本；命令返回并读取新反馈后才恢复。
按键请求排队期间也暂停写样本，命令完成时重新读取已发布的状态，避免使用界面本轮早先的快照。
待处理但尚未执行的按键不会延长普通反馈时限。反馈真正停滞和命令超时仍会退出，
命令前后的采样空隙保留在真实时间轴上，不补造数据。
旧配置未填写 `command_timeout_s` 时自动使用默认值；原启动命令和 O/Z 文件可继续使用。
若 SDK 调用一直不返回，保持请求也可能无法及时执行，应以真机状态为准。
退出会立即标记停止并取消未执行的控制命令；阻塞调用返回后先尝试保持，不再继续后续复位运动。

若旧版程序报 `Non-monotonic collection clock`，表示新样本时间戳不大于上一条。
旧版使用可能被 NTP 或手动校时调整的系统时间；修复版统一用进程启动时对齐 Unix 时间的
单调时钟记录样本、机械臂反馈、相机新帧、控制事件和推动边界，运行中的系统校时不再打乱采集顺序。
相机仅在收到新帧时打时间戳，重复读取不更新时间；旧帧超时检查继续生效。
启动命令、按键、采样频率、O/Z 参考和导出方式不变，退出旧进程后按原命令重启即可，
无需关闭系统时间同步或重新登记相同设备的 O/Z。若之前已有未完成段，检查对应 `meta.json`
的 status 与 reason；异常退出会尝试保留 incomplete 段，该段不会默认导出，需重新采集。
已保存数据不会被改写。若修复后仍出现此错误，新报错会包含前后时间戳以供排查。

正式扩量前做至少 20 次“人工粗定位—R—C”的重复定位检查，再试采约 10 个完整任务。
检查相机连续画面、实际采样频率、结束图、掩码与轨迹质量；不把模拟测试通过当作真机验收通过。
达到单段 180 秒上限会结束记录并标为 incomplete，不把超时段自动当成成功样本；主从仍需人工操作。

若退出原因是 `Camera ... frame is stale`，表示最后一帧已超过 `camera.stale_s`（默认 0.5 秒），
不能据此把旧帧继续写成新样本。错误会显示相机序列号、帧龄和阈值；程序请求机械臂保持，
已有未完成段会按 incomplete 保存。先退出占用相机的程序，检查腕部运动是否拉扯 USB 线、
接口是否松动，重新连接电脑 USB 3 接口，再运行本页的 `preview_realsense.py --headless --frames 300`。
若预览提示未找到相机，先恢复设备枚举；连续取帧成功并退出预览后再启动采集。
Wayland 的 `Ignoring XDG_SESSION_TYPE` 提示不是本次无新帧错误的判定依据。

## 10. 无硬件练习与自动检查

使用模拟设备跑两个任务、四个推动段，再导出到一个不存在的新目录：

```bash
python -m airbot_ie.push_wiper.collect --demo --output data/push_wiper_demo
python -m airbot_ie.push_wiper.export --input data/push_wiper_demo --output data/push_wiper_demo_export --allow-simulated
```

也可以在桌面练习完整按键流程：

```bash
python -m airbot_ie.push_wiper.collect --mock \
  --references data/mock_references.json --output data/mock_collection
```

模拟模式下，方向键改变虚拟 X/Y，`[`、`]` 改变虚拟 Z，`,`、`.` 改变偏航，只有 G 后生效。
按同样的 O/Z 登记、N/C/空格/S 流程练习。模拟数据默认不会进入正式训练导出。

运行自动测试：

```bash
python -m unittest discover -s tests -p 'test_push_wiper.py' -v
```

测试使用没有夹爪接口的模拟机械臂，覆盖模式切换、取消静止/主从差/近位限制、各臂到位判定、
跟随指令限速、重复复位、超时、单臂故障、
旧图像拒绝、有效段边界、保存失败、四元数符号/跨 ±π、真实 MCAP 读写和按任务划分数据。
单臂模式另外检查拖动期间无位置指令、只连接执行臂、模式切换失败、复位/保持、
独立参考文件复用与模式隔离，以及完整分段记录到训练数据的导出。
时钟回归测试模拟系统时间前跳/回拨，检查采集到导出仍正常、缓存相机帧仍会过期、
显式传入的乱序时间戳仍被拒绝。
观察位检查覆盖短暂超差等待、连续稳定后恢复、超时误差诊断、G/R 主动操作、
等待期间断连，以及恢复后必须取得新相机帧才能拍照。
线程测试覆盖慢模式切换等待、命令截止时间、旧反馈禁止采样、排队按键不能掩盖线程停滞，
以及退出后取消尚未执行的复位运动。

## 11. 确认保存成功与数据约定

正常按 S 提交后，终端应显示 `Saved: segment_...`，默认目录结构为：

```text
data/push_wiper/
  task_<时间和唯一编号>/
    task.json
    segment_<时间和唯一编号>/
      before.png
      after.png
      raw.mcap
      samples.jsonl
      meta.json
```

检查推动前后图片是否对应本次操作；`meta.json` 中正常段应为 `status: accepted`，
`sample_count` 应大于 0，`push_start_ns` 和 `push_end_ns` 应存在且结束时间大于开始时间。
只看到参考文件或任务目录，不代表已经保存完整推动段；`.partial` 目录也不表示提交成功。

每个任务含 `task.json`；每段独立保存 `raw.mcap`、`before.png`、`after.png`、
`samples.jsonl` 和 `meta.json`。JSONL 是与 MCAP 同源的低维记录，方便离线导出、检查时间戳，
不是额外控制信号。记录采用进程内统一的单调时钟，启动时对齐 Unix 时间；
属于主机读取时间，不是硬件同步时间。新段 `timestamp_basis` 标明此时钟来源，
旧版数据继续按原有时间戳读取。
未完成目录使用 `.partial` 后缀；只有 MCAP 和元信息写完才提交最终目录。
拒绝、超时和异常退出的段保留原始数据，但不会进入默认训练导出。
记录的 `eef/pose` 表示 SDK 末端参考点，不代表夹爪，也不代表海绵中心。
任务和每段元信息中的 `references.work_height` 保存 `z_m`、反馈时间戳 `t_ns`、
坐标系 `follower_base`、参考点 `sdk_end_reference` 和单位 `m`，随参考版本一起固定保存。

## 12. 离线导出

淡色、反光污渍漏检时，可使用 [人工掩码复查工具](push_wiper_mask_review.md) 补画、擦除并逐图确认，再使用 `--annotations` 严格导出已确认的前后图。

导出前可使用 [HSV 滑块调参工具](push_wiper_hsv_tuning.md)，实时对照原图、掩码和红色叠加图，并保存供导出使用的阈值配置。

激活项目环境后运行（输出目录必须是不存在的新目录，且位于原始数据目录之外）：

```bash
python -m airbot_ie.push_wiper.export --input data/push_wiper --output data/push_wiper_export
```

可复制 `airbot_ie/configs/push_wiper_mask.json` 修改颜色阈值，再通过
`--mask-config airbot_ie/configs/push_wiper_mask.json` 使用。采集参数模板为
`airbot_ie/configs/push_wiper_collection.json`；两份模板已纳入版本控制范围。

默认 HSV 掩码选择有颜色的污渍，要求固定的浅色背景和正确 ROI；必须检查 `mask_preview.png`。
支持 `--mask-config` 指定 HSV/Lab/灰度阈值 JSON。掩码数值在 NPZ 中为污渍 0、干净 1，
PNG 预览为 0/255。模型输入为起始掩码和拍摄位姿；标签为 `actions`（16×3）。
`actions_full`、`positions_full`、`orientations_xyzw` 和时间戳保留完整有效段。
动作定义版本为 2：`x,y` 是执行臂基坐标系下 SDK 末端参考点的绝对位置；
`delta_yaw = yaw(R_t) - yaw(R_cap)`，其中 `R_cap` 来自该段元信息中的
`references.observation.follow.orientation`（O），不是推动首帧。
论文 III-B 明确偏航增量相对固定拍摄姿态，本实现将旋转约定明确为基坐标系 ZYX 偏航：
`yaw(R)=atan2(R[1,0],R[0,0])`，即 SDK 末端 X 轴在基坐标 XY 平面上的投影航向。
角度先归一化到 `[-pi,pi]` 并沿有效段展开，再按时间重采样，不将每段首个角度强制置零。
若该轴竖直导致偏航不可定义，导出会排除并报告该段。
`capture_reference_pose` 保存固定 O 参考 `[x,y,z,qx,qy,qz,qw]`，作为模型的 `p_cap` 输入；
`capture_pose` 继续保存起始图片时的实际执行臂位姿，用于检查复位偏差。
样本 NPZ 的 `action_definition_version`、样本 meta.json 和 manifest.json 的 `action_definition`
都标明新定义。已有原始段只要保留 O 和完整四元数，就能重新导出；无需仅因偏航定义变化重采。
请勿混用旧版以 P 为零点的动作数组。重新导出到一个新目录，例如：

```bash
python -m airbot_ie.push_wiper.export --input data/push_wiper --output data/push_wiper_export_pcap_v2
```

复位、转移和清洁动作不会导出为标签。单个任务的数据全部放在同一个 split。
导出不再要求接触参考，也不按接触高度或倾斜筛选；完整 z 和四元数仍保留。
登记过 Z 的新数据会在 `sample.npz` 中附带标量 `work_height_m`，并在导出元信息中标记
`work_height_available=true`，供后续部署读取；动作仍为 `(x,y,delta_yaw)`。
旧数据若未登记工作高度，仍可导出，但没有 `work_height_m` 字段，且标记为 false；
程序不会用当前高度或旧 P 参考倒填历史数据。此版本只增加高度记录，不实现自动下降或力控。
有效动作范围完全由两次空格确定，不自动判断是否接触桌面。
采样间隙、相机/机器人读取时间差超限的段默认排除；在检查后才使用
`--include-review`。模拟段默认排除，仅测试时增加 `--allow-simulated`。
`trajectory.png` 单独显示 SDK 参考点的基坐标 XY 轨迹，没有叠加到污渍图上。
