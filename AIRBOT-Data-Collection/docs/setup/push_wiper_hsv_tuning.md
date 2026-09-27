# Push-Wiper HSV 滑块调参

如果阈值难以兼顾淡色污渍和反光，可继续使用 [人工掩码复查工具](push_wiper_mask_review.md) 修正每张图片。该工具支持补画、擦除和确认，并与正式训练导出连接。

在图形桌面终端运行，使用已有 airdc 环境和 OpenCV。无需连接相机或机械臂。

```bash
cd /home/wp/yuelk_project/Push_Wiper/AIRBOT-Data-Collection
source install/activate_airdc.sh
python -m airbot_ie.push_wiper.tune_hsv --input data/push_wiper
```

窗口显示原图、二值 mask（黑色为污渍）、红色叠加 mask_preview，以及污渍像素数、占比和连通区域数。
六个滑块分别控制 H min/max（0–179）、S min/max（0–255）、V min/max（0–255）。
拖动后实时刷新；下限超过上限时会同步移动另一端，始终保持有效范围。
这里 H 使用 OpenCV 的 8 位范围，不是 0–360 度；不支持用下限大于上限表达跨零色相区间。

把焦点放在图像窗口后使用快捷键：

| 按键 | 操作 |
|---|---|
| N / P | 下一张 / 上一张，末尾循环；同一段先 before 后 after |
| S | 保存当前参数，默认写入 `data/push_wiper_mask_tuned.json` |
| R | 恢复本次启动时载入的参数；需要再按 S 才会更新保存文件 |
| Q / Esc | 退出；关闭窗口也可退出 |

目录模式递归读取所有 `before.png` 和 `after.png`，包含 rejected 段，方便检查；正式导出仍遵循原有质量过滤。
切换图片保留当前滑块参数；保存的是一份全局配置，不是逐图人工标注。退出不会自动保存。
原始图片和元信息不会被修改，按 S 只更新指定配置文件。

## 单图与恢复调参

`--input` 也可以直接指定一张 PNG/JPG 图片的实际路径。
保存后继续调参可运行：

```bash
python -m airbot_ie.push_wiper.tune_hsv \
  --input data/push_wiper \
  --mask-config data/push_wiper_mask_tuned.json \
  --output-config data/push_wiper_mask_tuned.json
```

不指定 `--mask-config` 时从内置 HSV `[0,50,20]`～`[179,255,255]` 开始。
该工具只编辑单条 HSV 规则；多规则或 Lab/灰度配置会明确报错，避免丢弃其他规则。
形态学核大小与最小连通区域面积沿用载入配置，默认分别为 3 和 10，可编辑 JSON 后重新载入。

## 与训练导出一致

工具直接调用导出器的 `stain_mask`，先在原始分辨率上处理，再缩放显示。
采集目录下的 before/after 图片自动使用同目录 meta.json 中的 ROI；独立图片使用全图。
窗口不修改 ROI。保存的配置只含颜色规则和形态学/面积参数。

调好后检查多张图片，尤其是淡色、反光和画面边缘，再导出到一个不存在的新目录：

```bash
python -m airbot_ie.push_wiper.export \
  --input data/push_wiper \
  --output data/push_wiper_export_hsv_tuned \
  --mask-config data/push_wiper_mask_tuned.json
```

配置保存不会自动改变已导出的 NPZ 或图片，需要重新导出。

## 无窗口检查

```bash
python -m airbot_ie.push_wiper.tune_hsv \
  --input data/push_wiper \
  --snapshot /tmp/push_wiper_hsv_snapshot
```

输出目录必须不存在。此模式输出首张图片的 comparison.png、mask.png、mask_preview.png 和 mask_config.json，随后退出。
若 OpenCV 提示无法连接显示器或无法加载 GUI 后端，请回到本机图形桌面、激活 airdc 环境运行；无桌面时使用 snapshot 模式。
