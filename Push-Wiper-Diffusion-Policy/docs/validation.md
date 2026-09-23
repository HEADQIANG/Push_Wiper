# 本机 CPU 验收记录

日期：2026-09-23。目标数据：`push_wiper_export_assisted_complete_20260922`。

## 已完成检查

| 检查 | 结果 |
|---|---|
| 独立环境 | Python 3.10、torch 2.5.1+cpu、torchvision 0.20.1+cpu；`pip check` 通过 |
| 上游完整性 | 固定提交 `5ba07ac6661db573af695b419a7947ecb704690f`，369 个源文件校验一致 |
| 单元测试 | `28 passed`，涵盖数据划分、归一化、字段校验、指纹、指标、RNG隔离和输出 |
| 实际数据审计 | 47 段；训练39段/13任务，验证8段/3任务；任务无交叉 |
| CPU/GPU配置一致性 | policy、shape_meta、Dataset设置、optimizer和EMA设置相同 |
| 正式大小网络 | 模型参数/归一化参数共81,599,749个；ResNet18 + U-Net `[256,512,1024]` |
| CPU CLI短训练 | 2 epochs、4次优化器及EMA更新；正常保存best/latest检查点 |
| 参数更新 | 普通模型与EMA权重均发生变化 |
| 保存/加载 | 固定种子的完整16点预测逐元素一致 |
| 中断恢复 | 连续训练和第1轮后保存恢复，模型、EMA、优化器、学习率及RNG状态完全一致 |
| 批量离线推理 | 全部8个验证样本成功输出，数组形状`[8,16,3]`，生成8张独立对比图 |

完整验收进程记录的最大 RSS 约2337 MiB。此数值仅描述本次 CPU 检查，不是4090显存估计。
matplotlib/pyparsing 发出的弃用提示未影响图像或测试结果。

机器可读证据：

- [环境报告](validation_reports/environment_cpu.json)
- [数据审计](validation_reports/data_audit.json)
- [CPU短训练摘要](validation_reports/cpu_smoke_summary.json)
- [完整验收报告](validation_reports/cpu_acceptance.json)

这些轻量报告随Git发布；完整训练检查点及预测图像保留在本地`outputs/`，克隆后可按运行文档重新生成。

数据指纹：`bc84492e63bafd3fb29ef8039d8cde7985e3f3686fc3f182bea8b57a7700918b`。

## 尚需在4090验证

当前环境没有可用GPU，未执行CUDA模型训练、batch=16显存测试、4样本过拟合或3000 epochs正式训练。
CUDA版本锁已与官方wheel元数据和下载索引核对；GPU实测步骤见[运行文档](usage.md#4-迁移至-rtx-4090)。

本次CPU权重只经历4次更新，轨迹仍明显偏离演示；它证明接口和训练生命周期可运行，不代表策略收敛或清洁效果。
