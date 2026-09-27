# Push-Wiper 原始数据之后的复现步骤

原始数据检查日期：2026-09-22；训练适配更新日期：2026-09-23。论文依据：工作区根目录 `Push-Wiper.pdf`，III-B、III-C、IV-A、IV-B。

现已提供 [人工掩码复查与严格导出](push_wiper_mask_review.md)：可在自动分割初稿上补画、擦除，只有前后图都人工确认的段才进入 `--annotations` 导出。

## 1. 当前数据与已完成检查

原始目录：`data/push_wiper`。

- 16 个任务、49 段记录；48 段 accepted，1 段 rejected；元信息全部标为真实数据、ketchup、single_drag。
- 每段都有 before.png、after.png、samples.jsonl、raw.mcap、meta.json；每任务有 task.json。
- 使用现有导出器和默认掩码试导出到 `data/push_wiper_export_audit_20260922`：48 段导出、1 段排除。
- 按完整任务划分：训练集 13 个任务/40 段，验证集 3 个任务/8 段。
- 有效推动时长 3.76–9.49 秒，采样频率约 20 Hz；最大采样间隙 0.103 秒，最大相机/机器人读取时间差 0.034 秒，均通过默认导出阈值。这不是硬件同步精度证明。
- 48 段均保存固定拍摄参考和工作高度；工作高度约 0.098716 m，指执行臂基坐标系下 SDK 末端参考点高度，不是海绵接触面高度。
- 检查使用 JSONL、元信息和图片；导出器仅检查 MCAP 文件存在，没有逐帧验证 MCAP 内容。

**试导出数据仅供检查，暂不应直接作为正式训练集。**

## 2. 最优先：修正污渍掩码

抽查 16 段起始图的掩码叠加预览，发现默认 HSV 阈值漏检部分明显污渍，并选中画面边缘的红色物体。比如：

`task_20260922_174838_802a3c29d898/segment_20260922_175050_2a8e1efc52fc`

该段起始图片中有明显污渍，但导出只识别了 189 个污渍像素。非空掩码检查通过不代表分割正确；自动生成的 simple/complex 分类也暂不可作为可信标注。

操作顺序：

1. 确定覆盖全部有效推动区域的固定 ROI，排除桌面之外、线缆、海绵或工具边缘；不要裁掉合法动作对应的区域。
2. 调整 HSV/Lab/灰度组合阈值，核对所有 before/after 图，兼顾淡色残留和反光。
3. 导出器的 ROI 来自每段 meta.json；目前这些记录的 ROI 都为 null。现有 `--mask-config` 只能改颜色规则，不能覆写 ROI。历史数据裁剪需要补充非破坏性的导出选项，不要直接批量改写原始元信息。
4. 掩码修正后导出到新的目录。污渍=0、干净=1；训练和部署应保持同一 ROI、图像尺寸与分割方法。

现有可运行命令（从采集仓库目录运行）：

```bash
cd /home/wp/yuelk_project/Push_Wiper/AIRBOT-Data-Collection
source install/activate_airdc.sh
python -m airbot_ie.push_wiper.export --help
```

修改并保存掩码配置后，可用下列命令重新导出；输出目录必须不存在。此命令本身不解决 ROI 问题：

```bash
python -m airbot_ie.push_wiper.export \
  --input data/push_wiper \
  --output data/push_wiper_export_v2 \
  --mask-config airbot_ie/configs/push_wiper_mask.json
```

查看 `quality.json`、各样本的 `mask_preview.png`、`trajectory.png`。不要用 `--include-review` 掩盖质量问题。

## 3. 建立策略训练模块

已在并列目录 `Push-Wiper-Diffusion-Policy/` 建立独立图像 Diffusion Policy 训练工程，提供 NPZ Dataset、训练、EMA 验证、断点恢复和离线 16 点轨迹生成。策略执行闭环仍需另行实现。安装、CPU 验证及 RTX 4090 迁移见[训练运行文档](../../../Push-Wiper-Diffusion-Policy/docs/usage.md)。

- 一段有效推动作为一个监督样本；输入为 `mask` 与 `capture_reference_pose`，标签为 `actions`，形状 `(16, 3)`。
- `capture_reference_pose` 是固定拍摄参考 p_cap；`capture_pose` 是实际拍摄位姿，用于检查复位误差。
- 动作为基坐标系下 SDK 参考点绝对 x、y（米），以及相对固定拍摄姿态的 delta_yaw（弧度）。当前版本是 action_definition_version=2；部署必须采用一致的坐标、参考点和旋转约定。
- `after_mask` 用于效果分析，不作为部署时不可获得的模型输入。
- 独立工程复用固定版本的官方 Dataset 接口、归一化、卷积式 Diffusion Policy 和训练 Workspace，适配训练/验证、checkpoint 和离线采样；统计只取训练集。
- 对齐论文 IV-B：DDIM，sample prediction，训练 diffusion steps=100，推理 steps=10，每次生成完整 16 点推动段。学习率、batch size、图像分辨率、网络宽度等未在该节完整公开，应查作者配置或明确记录自行选择的值。
- 先在少量干净样本上验证能过拟合，再训练全部数据并检查验证任务的轨迹位置、方向和范围。训练环境建议独立于机器人采集环境。

当前正式数据为 `data/push_wiper_export_assisted_complete_20260922`：47 段，训练39段/13任务、验证8段/3任务。它与上文保留的旧试导出统计不同。

安装独立训练环境后，从训练工程目录运行：

```bash
cd /home/wp/yuelk_project/Push_Wiper/Push-Wiper-Diffusion-Policy
bash scripts/install_cpu.sh
source .venv/bin/activate
export PYTHONNOUSERSITE=1
unset PYTHONPATH
python -m push_wiper_dp.train --config-name=push_wiper runtime=cpu_smoke
```

CPU 短训练仅验证流程。正式训练迁移到4090后使用 `scripts/install_cuda.sh` 和 `runtime=rtx4090`；不要把训练依赖装入采集环境。

## 4. 补数据与几何标定

论文数据为 150 个清洁任务（番茄酱和花生酱各 75 个；每种 25 个简单、50 个复杂任务）、448 条原始推动轨迹，联合仿射增强到 2688 条。当前正式导出为47条，可先用于打通流程，但不足以据此宣称论文效果已复现。

补采不同位置、形状、数量和推动方向的污渍，以及花生酱；先确认元信息的 stain 与实际材料一致。测试任务独立采集；先按 task 划分，再对训练集增强。

当前标签是机器人基坐标米制位置，不能只旋转图像而不变换轨迹。联合几何增强前应建立桌面图像与机器人坐标的映射（如平面单应性），并明确 SDK 参考点到海绵 TCP 的外参；旋转标签也须同步变换。图像缩放/裁剪后的映射必须一致更新。

## 5. 先实现平面执行闭环，再扩展论文完整系统

平面闭环：回拍摄位 → 分割 → 推理完整 16 点推动段 → 生成带速度限制的连续轨迹 → 接近并建立接触 → 执行 → 抬起/刮海绵 → 回拍摄位重新观察。

- 工作高度记录不等于自动下降、接触检测或力控。当前采集控制器没有实现这些策略执行功能。
- 平面实验按论文可关闭 ASPI，但仍需处理轨迹平滑和接触控制。
- 论文使用 UR7e、六维力传感器和 100 Hz 混合力位控制，目标法向力 20 N。AIRBOT Play 的适配要确认实际力反馈、控制接口和工具能力；关节 effort 不可直接视为 TCP 法向力，20 N 也不能未经适配直接照搬。
- 可先做固定高度的平面概念验证，但应记录这是简化执行方案。验证顺序为离线轨迹检查、悬空执行、接触实验。
- 曲面复现还需表面高度/法向估计、ASPI、B 样条与梯形速度规划、SLERP、法向力控制。
- 完整后处理包括蘸取残留、刮擦、冲洗、挤压和最终全覆盖擦拭。

## 6. 效果评估

实现 CS = (1 - N_after/N_before) × 100%，明确 N_before=0 的处理，记录运行时间、推动次数、失败案例；单次推动的面积变化不代表完整清洁任务得分。

论文的停止条件是污渍面积小于 100 像素；应用前需要固定分辨率与 ROI，并先保证掩码可靠，避免漏检导致假完成。

若要对齐论文实验，还需实现 Full-Cover 和 PushAll-Onetime 基线，在相同控制器、清洁预算和海绵处理规则下比较，并单独评估后处理贡献与曲面/新污渍泛化。
