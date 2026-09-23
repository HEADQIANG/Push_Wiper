# 适配实现与文件职责

## 上游复用

固定 `real-stanford/diffusion_policy` 的 `5ba07ac6661db573af695b419a7947ecb704690f`，
保留完整源码及 MIT 许可证。通过独立环境中的 editable 安装导入；未修改上游文件。
`python scripts/verify_upstream.py` 检查每个上游文件的 SHA-256。

直接复用 `DiffusionUnetImagePolicy`、`ConditionalUnet1D`、`MultiImageObsEncoder`、
ResNet18、`LinearNormalizer`、`EMAModel`、优化器构建与 `BaseWorkspace` 检查点格式。
派生 `TrainDiffusionUnetImageWorkspace` 并覆盖离线训练循环，修正原循环的仿真 runner 要求、
普通模型验证与 EMA 推理不一致，以及调度器、EMA 更新计数、RNG 和轮次的恢复缺口。

## 数据与网络契约

一个 NPZ 对应一个完整推动段，单样本为：

```text
obs.mask                    float32 [1,3,240,320]，Dataset 输出0/1
obs.capture_reference_pose  float32 [1,7]
action                     float32 [16,3]
```

policy 内部将 mask 固定线性变换到 [-1,1]，动作和位姿使用完整训练集统计。
不做滑动窗口、不读取结束掩码、不改变任务划分、不把绝对位置改成位移。
不对已展开的 yaw 重新 wrap。固定拍摄位姿常量维通过官方 normalizer 的零极差处理映射到0。

ResNet18 不下载预训练权重；`share_rgb_model=false` 使官方 GroupNorm 替换生效。
Dataset 统一最近邻缩图，编码器的 resize/crop 均关闭。U-Net 通道 `[256,512,1024]`，
嵌入128、卷积核5、GroupNorm组数8。模型为一个观察生成完整16点，DDIM训练步数100、推理10、sample prediction。
图像尺寸、网络宽度及优化器参数是明确记录的复现选择，不能当作作者未公开的原始配置。

## 文件位置与作用

| 位置 | 作用 |
|---|---|
| `push_wiper_dp/dataset.py`、`audit.py` | NPZ 数据校验、预处理、训练统计、可迁移指纹和审计命令 |
| `push_wiper_dp/config.py`、`configs/` | 共用模型定义、CPU验证、4090正式与4样本过拟合配置 |
| `push_wiper_dp/workspace.py`、`train.py` | 官方 Workspace 适配、本地日志、EMA验证、原子保存和恢复 |
| `push_wiper_dp/evaluation.py`、`predict.py` | EMA加载、16点采样、轨迹指标、NPZ与图像输出 |
| `push_wiper_dp/verify_cpu.py` | 完整模型CPU验收，比较连续训练与中断恢复状态 |
| `scripts/`、`requirements/`、`pyproject.toml` | 隔离环境安装、版本锁定、上游与环境自检 |
| `tests/` | 数据泄漏、归一化、指标、随机性与输出接口测试 |
| `docs/usage.md`、`README.md` | 安装、训练、恢复、推理和4090迁移操作步骤 |
| `docs/validation.md` | 当前真实数据CPU验收结果、证据位置及尚未验证的GPU项 |
| `scripts/train_4090.sh`、`docs/github.md` | GitHub克隆后一键安装和训练、发布数据范围及恢复方法 |
| `docs/validation_reports/` | 随仓库保留的轻量CPU验收证据 |

修改代码后同步检查 `docs/usage.md` 的命令和步骤；新增入口或配置必须记录，并在交付时说明修改文件及作用。

## 检查点及可重复性

检查点同时保留普通模型与 EMA 模型、优化器、学习率状态、EMA计数、最佳验证值、Python/NumPy/Torch
随机数状态及 DataLoader generator 状态。元信息保留模型和预处理配置、动作定义、训练数据指纹及上游版本。
恢复时校验语义和学习率计划，加载到CPU后再搬到目标设备；不恢复旧的绝对输出目录。
同步写临时文件后原子替换，避免后台线程保存CPU共享Tensor时产生竞态。

验证采样与固定噪声损失不推进训练随机数状态。验证损失按样本数加权，最优权重依据 EMA 验证损失选择。
没有仿真环境评测或真机清洁分数；mask、基坐标XY和yaw分别绘图。
