# Push-Wiper：Diffusion Policy 复现

基于官方图像 Diffusion Policy，使用二值污渍图与固定拍摄位姿生成完整的16点推动轨迹。
仓库包含训练代码、固定版本的上游源码、依赖锁文件，以及当前47段训练/验证数据。

## 在 RTX 4090 上开始训练

环境要求：Linux x86_64、Python 3.10（含 `venv`）、Git，以及支持 CUDA 12.1 的 NVIDIA 驱动。
Ubuntu 22.04 可通过 `sudo apt install git python3.10 python3.10-venv` 补充基础工具。

```bash
git clone https://github.com/HEADQIANG/Push_Wiper.git
cd Push_Wiper
nvidia-smi
bash Push-Wiper-Diffusion-Policy/scripts/train_4090.sh
```

脚本会在独立 `.venv` 安装 PyTorch 2.5.1 + CUDA 12.1 及锁定依赖，检查 CUDA，随后训练。
默认 batch size 为16、训练3000 epochs；输出位于 `Push-Wiper-Diffusion-Policy/outputs/rtx4090/`。
模型结构和数据预处理已在CPU验证，4090显存和吞吐需要在目标机器实测。

也可以先做4样本过拟合检查：

```bash
bash Push-Wiper-Diffusion-Policy/scripts/train_4090.sh runtime=overfit4090
```

从最近完整epoch恢复：

```bash
bash Push-Wiper-Diffusion-Policy/scripts/train_4090.sh \
  training.resume=outputs/rtx4090/checkpoints/latest.ckpt
```

通用依赖下载较慢时，可在命令前设置
`PUSH_WIPER_PIP_INDEX=https://pypi.tuna.tsinghua.edu.cn/simple`；PyTorch仍使用官方平台源。

## 仓库内容

| 目录 | 内容 |
|---|---|
| `Push-Wiper-Diffusion-Policy/` | 模型适配、CPU/4090配置、训练、恢复、推理、测试与文档 |
| `Push-Wiper-Diffusion-Policy/third_party/` | 官方Diffusion Policy源码、MIT许可证及逐文件校验值 |
| `AIRBOT-Data-Collection/data/push_wiper_export_assisted_complete_20260922/` | 47个NPZ与manifest，训练39段/13任务，验证8段/3任务 |

训练样本和清单约0.5 MB，普通 `git clone` 即可获取，不需要 Git LFS 或子模块。
数据按原任务划分，保留导出动作定义v2；训练不需要连接机械臂。
采集工程目录在本仓库中仅用于保存训练所需的导出数据。

## 文档与验证

- [完整运行文档](Push-Wiper-Diffusion-Policy/docs/usage.md)：安装、CPU验证、4090训练、恢复及离线推理。
- [适配实现及文件职责](Push-Wiper-Diffusion-Policy/docs/implementation.md)。
- [CPU验收记录](Push-Wiper-Diffusion-Policy/docs/validation.md)：28项测试、完整模型更新、保存加载及断点恢复一致性。
- [发布与克隆说明](Push-Wiper-Diffusion-Policy/docs/github.md)。

官方Diffusion Policy固定于 `5ba07ac6661db573af695b419a7947ecb704690f`，保留其原始许可证。
当前47段数据用于打通复现流程；CPU短训练权重和离线轨迹误差不代表论文清洁效果已复现。
