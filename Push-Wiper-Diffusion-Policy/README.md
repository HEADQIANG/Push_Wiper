# Push-Wiper Diffusion Policy

复用官方图像 Diffusion Policy，为现有 Push-Wiper 分段 NPZ 提供训练、验证、断点恢复和离线轨迹生成。
CPU 与 RTX 4090 共用 ResNet18 + 条件一维 U-Net；CPU 验证仅减少样本数与更新次数。

GitHub仓库包含默认数据，克隆后可从仓库根目录运行
`bash Push-Wiper-Diffusion-Policy/scripts/train_4090.sh`，自动安装独立CUDA环境并开始训练。
基础环境要求和恢复方法见[GitHub运行说明](docs/github.md)。

官方源码固定在 `5ba07ac6661db573af695b419a7947ecb704690f`，原样保存在
`third_party/diffusion_policy/`。上游许可证位于该目录的 `LICENSE`，文件校验信息位于
`third_party/UPSTREAM.json`。

从本目录运行：

```bash
bash scripts/install_cpu.sh
source .venv/bin/activate
export PYTHONNOUSERSITE=1
unset PYTHONPATH
export MPLCONFIGDIR="$PWD/.cache/matplotlib"
python -m push_wiper_dp.audit --data-root ../AIRBOT-Data-Collection/data/push_wiper_export_assisted_complete_20260922
python -m push_wiper_dp.train --config-name=push_wiper runtime=cpu_smoke
```

完整安装、CPU 验收、4090 迁移、正式训练、恢复和推理步骤见 [运行文档](docs/usage.md)。
数据字段、模型设置及源码分工见 [适配说明](docs/implementation.md)。
已完成的测试及真实数据 CPU 验收结果见 [验收记录](docs/validation.md)。

本阶段输出每段完整的 16 个 `(x,y,delta_yaw)` 动作，不连接机械臂。CPU 短训练产生的权重用于流程检查，
不能作为已训练完成的清洁策略；当前数据集共有 47 段，尚不足以证明论文清洁效果得到复现。
