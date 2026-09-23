# GitHub克隆与4090训练

远端仓库：<https://github.com/HEADQIANG/Push_Wiper>。

## 克隆后运行

需要Linux x86_64、Python 3.10及venv、Git，以及能运行CUDA 12.1的NVIDIA驱动。

```bash
git clone https://github.com/HEADQIANG/Push_Wiper.git
cd Push_Wiper
nvidia-smi
bash Push-Wiper-Diffusion-Policy/scripts/train_4090.sh
```

一键脚本完成锁定依赖安装、上游完整性和CUDA自检，然后执行`runtime=rtx4090`。
既有`.venv`会复用，pip仍核对锁定版本。默认输出`Push-Wiper-Diffusion-Policy/outputs/rtx4090/`。
已有检查点时不会自动覆盖，需要显式恢复或指定新输出目录。

```bash
# 保留正式3000轮学习率计划，先检查一轮的显存和耗时
bash Push-Wiper-Diffusion-Policy/scripts/train_4090.sh training.stop_after_epochs=1

# 继续同一实验
bash Push-Wiper-Diffusion-Policy/scripts/train_4090.sh \
  training.resume=outputs/rtx4090/checkpoints/latest.ckpt
```

脚本先切换到训练工程目录，因此相对`training.resume`和`run.output_dir`均按训练工程解释。
`runtime=overfit4090`可用于4样本过拟合；`task.dataset.root=/其他数据目录`可切换数据。
更多命令见[运行文档](usage.md)。

## 随仓库提供的数据

`AIRBOT-Data-Collection/data/push_wiper_export_assisted_complete_20260922/`包含原始导出清单和清单列出的47个NPZ，
约0.5 MB。NPZ逐字节保留，训练集39段、验证集8段，任务无交叉。
完整性和训练接口可通过下列命令检查：

```bash
cd Push-Wiper-Diffusion-Policy
source .venv/bin/activate
unset PYTHONPATH
export PYTHONNOUSERSITE=1
python -m push_wiper_dp.audit \
  --data-root ../AIRBOT-Data-Collection/data/push_wiper_export_assisted_complete_20260922
```

预期数据指纹为`bc84492e63bafd3fb29ef8039d8cde7985e3f3686fc3f182bea8b57a7700918b`。
原始MCAP、标注界面预览、采集SDK、虚拟环境、缓存和训练检查点不属于克隆所需的训练输入。

## 发布内容与验证证据

`third_party/UPSTREAM.json`记录官方版本及369个文件的哈希，源文件完整纳入Git；不依赖子模块初始化。
`.gitattributes`禁止转换上游文件字节，避免换行改变导致完整性检查失败。
发布时保留上游许可证，并忽略环境、缓存、构建输出和检查点。

`docs/validation_reports/`保存此前CPU验收的轻量JSON证据，路径已转为仓库相对路径；
完整权重和图像由训练/推理命令生成。CUDA依赖已固定，GPU实测仍应在4090执行。
