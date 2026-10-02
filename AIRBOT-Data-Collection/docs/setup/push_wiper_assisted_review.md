# 2026-09-22 逐张识别、用户确认流程

用户指定从 `task_20260922_174838_802a3c29d898/segment_20260922_175050_2a8e1efc52fc` 开始（包含该段），按采集顺序对 accepted 段的 before、after 图片逐张生成候选掩码。每次只展示一张，等待用户明确确认；有修改意见时修改同一张并重新展示，不能跳到下一张，也不能把修改意见本身视为确认。

## 保留的人工数据

分界前 66 张图片、33 段的原人工标注全部 confirmed，原图/ROI/掩码校验通过。
复制其当前有效的 review.json 和引用的 PNG 到独立目录 `data/push_wiper_annotations_assisted_20260922`，保留原 `data/push_wiper_annotations` 不变。分界后的旧标注不自动加入本次目录，等待这次逐张确认。

初始数据集已导出到 `data/push_wiper_export_assisted_initial_20260922`：33 段，按任务分为 27 段训练、6 段验证。其他 15 段等待这次复查；原始数据另有 1 段 rejected，仍排除。

## 会话进度与确认

进度文件：`outputs/push_wiper_assisted_review_20260922/review_state.json`。
候选图位于该会话目录的 `candidates/001`、`002` 等目录，含 original.png、mask.png、mask_preview.png、comparison.png 和 proposal.json。
开始时共 30 张待复查；第一张是指定段的 before.png，当前只保存候选文件，尚未写入本次 confirmed 标注目录。

每张图的处理顺序：

1. 读取进度文件，定位 current_number；读取原始图，按原有 ROI 工作；确认源图摘要与队列一致。
2. 视觉检查并生成候选，PNG 必须是原生裁剪尺寸、单通道 uint8、只含 0/255（污渍/干净）。更新 proposal.json 的候选摘要，展示原图、掩码和叠加图。
3. 用户有修改意见时保留旧版本，修正候选与叠加图，仍停在当前项等待确认。
4. 只有用户明确确认当前候选后，重新校验源图与已展示掩码的摘要，使用 annotations.save_annotation 保存到本次标注目录，状态 confirmed。PNG 除以 255 后传入，勿重复形态学处理。
5. 回读 load_annotation(require_confirmed=True) 验证，记录确认的掩码摘要与用户确认文字，将队列该项改为 confirmed，再生成和展示下一张。
6. 同一段的 before、after 均确认后才具备严格导出资格。全部处理完后导出完整的新数据集；切勿覆盖已导出目录或原人工记录。

进度必须以实际保存并校验成功的文件为准。处理期间不修改原图、原始元信息、MCAP 或之前的人工掩码，不自动确认 AI 候选。

## 全部确认后的导出命令

```bash
cd /home/wp/yuelk_project/Push_Wiper/AIRBOT-Data-Collection
source install/activate_airdc.sh
python -m airbot_ie.push_wiper.export \
  --input data/push_wiper \
  --annotations data/push_wiper_annotations_assisted_20260922 \
  --output data/push_wiper_export_assisted_complete_20260922
```

此命令留待全部确认后运行；当前尚未生成 complete 数据集。
