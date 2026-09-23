# 安装、验证与迁移运行步骤

## 1. 工程与数据位置

默认目录结构：

```text
Push_Wiper/
├── AIRBOT-Data-Collection/data/push_wiper_export_assisted_complete_20260922/
└── Push-Wiper-Diffusion-Policy/
```

下列命令从 `Push-Wiper-Diffusion-Policy/` 执行。默认数据根目录按代码位置解析，不依赖用户名。
其他位置的数据用 `task.dataset.root=/绝对路径/数据目录` 覆盖；审核与推理使用 `--data-root`。
保留 manifest 的相对路径、样本内容、记录顺序和原有任务划分。

## 2. 当前 CPU 环境

安装在训练工程的 `.venv`，不向采集环境 `airdc` 安装训练依赖。需要 Python 3.10 和联网下载公开软件包。

```bash
bash scripts/install_cpu.sh
source .venv/bin/activate
export PYTHONNOUSERSITE=1
unset PYTHONPATH
export MPLCONFIGDIR="$PWD/.cache/matplotlib"
python scripts/check_environment.py --device cpu
python -m pip check
```

通用依赖下载较慢时可使用已验证的镜像入口（PyTorch wheel 仍来自官方平台源）：

```bash
PUSH_WIPER_PIP_INDEX=https://pypi.tuna.tsinghua.edu.cn/simple bash scripts/install_cpu.sh
```

如 Python 3.10 不在默认命令位置，可在安装前设置
`PUSH_WIPER_PYTHON=/路径/python3.10`。自定义环境路径使用 `PUSH_WIPER_VENV`，之后激活对应目录。
脚本检查上游源码校验值、关键模块导入和 DDIM 配置。平台依赖与通用依赖分别锁定，
CPU 与 GPU 的 torch/torchvision 版本相同，仅 wheel 后缀不同。

检查实际数据和单元测试：

```bash
python -m push_wiper_dp.audit \
  --data-root ../AIRBOT-Data-Collection/data/push_wiper_export_assisted_complete_20260922 \
  --output outputs/data_audit.json
OMP_NUM_THREADS=2 MKL_NUM_THREADS=2 python -m pytest -q
```

当前应识别 39 个训练段／13 个任务、8 个验证段／3 个任务；无任务交叉。
全部 7 个拍摄参考位姿维度为常量，其归一化结果为零。该统计不等于能够跨拍摄位姿泛化。

## 3. CPU 短训练与完整验收

```bash
python -m push_wiper_dp.train --config-name=push_wiper runtime=cpu_smoke
```

使用与 4090 相同的完整模型，batch=1、训练样本=2、验证样本=1、2 epochs，产生 4 次优化器和 EMA 更新。
图像尺寸仍为 240×320，归一化始终使用全部 39 个训练样本。500 步预热仍生效，第一次更新的学习率为零。

默认输出 `outputs/cpu_smoke/`。目录已有检查点时，训练入口会拒绝覆盖；可用
`run.output_dir=outputs/cpu_smoke_new` 指定新目录，或按下面的恢复方式继续。

完整验收另外运行连续训练和中断恢复两条路径，并对比模型、EMA、优化器、学习率、随机数状态和固定种子推理：

```bash
python -m push_wiper_dp.verify_cpu --output-dir outputs/cpu_acceptance
```

验收目录必须不存在或为空。此命令使用完整正式模型，会生成多份较大的检查点。
成功结果写入 `outputs/cpu_acceptance/verification_report.json`，其中 `passed=true`，
`resume_states_exact=true`、`reload_prediction_exact=true`。这项检查仅证明 CPU 流程正确，不验证 CUDA 性能。

## 4. 迁移至 RTX 4090

仓库已包含默认训练数据，可以直接克隆运行：

```bash
git clone https://github.com/HEADQIANG/Push_Wiper.git
cd Push_Wiper
bash Push-Wiper-Diffusion-Policy/scripts/train_4090.sh
```

该脚本把后续参数传给Hydra，例如`runtime=overfit4090`或`training.stop_after_epochs=1`。
以下为手动安装和分别执行各步骤的方式；命令仍从训练工程目录执行。

目标为 Linux x86_64、RTX 4090 24GB。复制训练工程（含 `third_party` 和 requirements 锁文件）及数据目录；
不复制 `.venv`、`.cache`。正式训练从新初始化开始，不沿用 CPU 短训练权重。
首次迁移可以省略 `outputs/`，其中 CPU 验收的重复完整检查点较大；保留 `docs/validation.md` 即可查看验收摘要。
若迁移已有正式实验，连同完整输出目录和检查点一起复制。

在目标机器安装并检查：

```bash
nvidia-smi
bash scripts/install_cuda.sh
source .venv/bin/activate
export PYTHONNOUSERSITE=1
unset PYTHONPATH
export MPLCONFIGDIR="$PWD/.cache/matplotlib"
python scripts/check_environment.py --device cuda
```

CUDA wheel 为 PyTorch 2.5.1 + CUDA 12.1；目标驱动需要支持 CUDA 12.1。
安装 PyTorch wheel 不会安装或修复 NVIDIA 驱动。目标环境自检会实际执行 CUDA 运算。

先用同一网络过拟合固定的 4 条训练演示：

```bash
python -m push_wiper_dp.train --config-name=push_wiper runtime=overfit4090
```

该配置运行 1000 epochs、batch=4，并输出这 4 条演示的轨迹与误差至 `outputs/overfit4090/overfit/`。
结合 loss 曲线、XY/yaw 曲线确认模型确实拟合数据；验证集误差与训练误差分开报告。

验证正式 batch=16 的显存及吞吐，可先运行一轮并保持完整 3000 epochs 学习率计划：

```bash
python -m push_wiper_dp.train --config-name=push_wiper runtime=rtx4090 \
  training.stop_after_epochs=1
```

确认 `logs.jsonl` 中的 `cuda_peak_allocated_gb`、`cuda_peak_reserved_gb` 和训练耗时后继续：

```bash
python -m push_wiper_dp.train --config-name=push_wiper runtime=rtx4090 \
  training.resume=outputs/rtx4090/checkpoints/latest.ckpt
```

也可直接开始完整实验：

```bash
python -m push_wiper_dp.train --config-name=push_wiper runtime=rtx4090
```

正式配置 batch=16，保留最后不足一个 batch 的数据，3000 epochs 对当前 39 段数据为 9000 次优化器更新。
每 10 epochs 以固定噪声计算 EMA 验证损失；每 50 epochs 保存最新权重及全部验证段的预测图，结束时额外保存。
预期运行时间和峰值显存需要在 4090 上实测。

## 5. 输出与恢复规则

每个实验目录包含：

| 文件或目录 | 内容 |
|---|---|
| `config.yaml`、`.hydra/` | 实际解析后的配置与命令覆盖 |
| `data_audit.json` | 数据数量、动作语义、归一化来源和内容指纹 |
| `logs.jsonl`、`run_summary.json` | loss、学习率、更新次数、耗时及 GPU 显存 |
| `checkpoints/latest.ckpt` | 最近完整 epoch 的模型、EMA、优化器、调度器、RNG 状态 |
| `checkpoints/best.ckpt` | EMA 验证损失最低时的完整检查点 |
| `validation/epoch_XXXX/` | 预测数组、逐样本误差和轨迹图 |

仅在 epoch 边界恢复，`epoch` 表示下一轮要执行的轮次。恢复时保留训练总轮数、batch size、样本子集、
模型结构和学习率计划；要提前暂停，使用 `training.stop_after_epochs`，不要通过改小 `training.num_epochs` 暂停。
数据、划分、顺序或动作语义不匹配会报错；更换根目录不会改变数据指纹。
同设备 CPU 验收要求恢复结果一致；CPU 与 GPU 之间不承诺逐位相同。

## 6. 离线推理

CPU 流程检查权重：

```bash
python -m push_wiper_dp.predict \
  --checkpoint outputs/cpu_smoke/checkpoints/best.ckpt \
  --data-root ../AIRBOT-Data-Collection/data/push_wiper_export_assisted_complete_20260922 \
  --split validation --device cpu
```

4090 正式权重：

```bash
python -m push_wiper_dp.predict \
  --checkpoint outputs/rtx4090/checkpoints/best.ckpt \
  --data-root ../AIRBOT-Data-Collection/data/push_wiper_export_assisted_complete_20260922 \
  --split validation --device cuda:0
```

默认输出到实验目录的 `predictions/validation/`，可通过 `--output-dir` 覆盖。
使用 EMA 模型与固定种子，输出 `predictions.npz`、`metrics.json` 和各样本图像。
NPZ 中 `predictions` 为 `[N,16,3]`，`targets` 为演示标签；米和弧度单位保持不变。
XY ADE/FDE 使用毫米，yaw MAE 使用圆周角误差（度）；连续性指标使用预测的原始展开角度。

Python 接口：

```python
import numpy as np
import torch
from push_wiper_dp.predict import PushWiperPredictor

torch.set_num_threads(4)  # CPU 时避免过多线程开销
predictor = PushWiperPredictor("outputs/rtx4090/checkpoints/best.ckpt", device="cpu")
with np.load("某一段/sample.npz", allow_pickle=False) as sample:
    actions = predictor.predict(sample["mask"], sample["capture_reference_pose"], seed=42)
assert actions.shape == (16, 3)
```

输入 mask 为原始 480×640 的 0/1 数组，参考位姿为 7 维 XYZW 数组。推理不读取 `after_mask` 或目标动作。
16 点覆盖完整段的归一化时间，不是 16 个固定频率控制周期。当前代码不生成机械臂控制命令，
离线轨迹误差不能换算成清洁成功率。
