# Push-Wiper 人工污渍掩码复查

用于修正淡色、反光污渍的漏检及工具、背景的误检。自动分割只提供初稿，确认由使用者逐图完成；工具不会自动替使用者确认。人工标签不会改善部署时的自动分割。

## 启动与断点继续

在本机图形桌面终端执行，无需连接机械臂或相机：

```bash
cd /home/wp/yuelk_project/Push_Wiper/AIRBOT-Data-Collection
source install/activate_airdc.sh
python -m airbot_ie.push_wiper.review_masks \
  --input data/push_wiper \
  --annotations data/push_wiper_annotations
```

第一次启动使用默认 HSV 生成初稿。若已保存调参配置，可加
`--mask-config data/push_wiper_mask_tuned.json`。改变该配置只影响尚未保存人工结果的图片。
重启相同命令会恢复草稿、待复查和已确认结果，定位首张未确认图片；全部确认时从第一张开始。

只浏览 accepted 段，忽略 rejected 和 `.partial` 段；排序为任务、段、before、after。
2026-09-22 检查的数据对应 48 段、96 张待人工复查的图片。
顶部显示 task/segment/图片名、总体进度、当前状态、笔刷半径、缩放倍数和污渍占比。
图像从左到右为原图、黑白掩码、红色覆盖预览。为兼容 OpenCV 字体，窗口文字采用英文。

## 操作步骤

1. 对照左侧原图检查掩码。暗淡污渍若仍可看清轮廓，可补画；不要将反光、水迹一律当成污渍。
2. 在中间掩码或右侧预览面板内按住鼠标左键拖动，补画污渍；切到擦除工具后移除误检。
3. 必要时放大、平移或隐藏红色覆盖；左右三栏始终显示相同位置。
4. 可信时按 Enter 确认当前图片。不确定、过曝或边界无法辨认时按 U 标记待复查，不凭空补全。
5. 按 N 切换，分别确认 before 和 after。图片掩码已经正确时可直接 Enter，无需涂画。

| 按键 | 功能 |
|---|---|
| N / P | 下一张 / 上一张，末尾循环；自动保存已有编辑 |
| B / E | 画笔（污渍）/ 擦除（干净） |
| [ / ] | 减小 / 增大笔刷半径，原生图像像素单位，范围 1–100 |
| Z | 撤销一次完整笔划，保留最近 30 步；撤销后也需要重新确认 |
| + 或 = / - | 放大 / 缩小，范围 1–8 倍 |
| I / J / K / L | 向上 / 左 / 下 / 右平移放大区域 |
| O | 隐藏 / 显示红色覆盖 |
| S | 保存草稿，不是确认，也不是保存 HSV 参数 |
| Enter | 保存并确认当前图片，不会替另一张前后图确认 |
| U | 保存并标记为待复查 |
| Q / Esc | 保存编辑并退出；关闭窗口或 Ctrl+C 同样尝试保存 |

状态为 unreviewed（未复查）、draft（草稿）、needs_review（待复查）、confirmed（已确认）。
invalid 表示原图、ROI 或标注损坏/不一致，此时禁止编辑覆盖，可切换其他图片并根据终端错误处理。
修改已确认图片前会先在磁盘撤销确认，再开始编辑，避免程序意外退出后仍将旧确认用于导出。
仅浏览图片不会把它自动确认为有效，也不会保存未修改的自动初稿。
保存失败时保留当前图片和内存编辑并报告错误；关闭窗口保存失败时重新打开窗口，不丢弃编辑。

## 保存约定

原始采集图片、MCAP、元信息不修改。人工目录必须位于原始数据目录之外：

```text
data/push_wiper_annotations/
  task_.../segment_.../
    before/
      review.json
      mask-<SHA256>.png
    after/
      review.json
      mask-<SHA256>.png
```

PNG 为单通道 uint8，污渍=0、干净=255；NPZ 中转换为污渍=0、干净=1。
掩码保存的是原图按原有 ROI 裁剪后的原生分辨率；预览缩放不改变标签坐标。本工具不编辑 ROI。
review.json 包含 schema_version、status、源相对路径、原图 SHA256、ROI、掩码 shape 和 SHA256。
先保存以内容摘要命名的 PNG，再原子更新 review.json。旧 PNG 保留，写入失败不破坏上一份完整记录；
导出只读取 review.json 引用的掩码，不以目录中最新 PNG 为准。不要手工修改记录或 PNG。
同一份标注目录请使用单个复查窗口，避免两个窗口相互覆盖确认状态。

## 正式导出

输出目录必须不存在：

```bash
python -m airbot_ie.push_wiper.export \
  --input data/push_wiper \
  --annotations data/push_wiper_annotations \
  --output data/push_wiper_export_reviewed
```

指定 `--annotations` 后，只导出前后两张图片都 confirmed 且完整性校验通过的段。
未确认、缺失、尺寸不符、ROI/原图变化或掩码损坏都排除，并记录到 quality.json；
`--include-review` 不能绕过人工确认与完整性要求。
原有轨迹采样/时差/模拟数据等检查和按任务划分规则保留；人工确认不等于段一定通过其他检查。
起始掩码为空仍触发原有 empty_initial_mask 检查；after 全白可表示清洁后的无污渍状态。

最终人工掩码直接写入 sample.npz 的 mask/after_mask，并生成预览，不再次做形态学或小区域过滤。
面积、连通区域和 simple/complex 分类按最终掩码重新计算。
manifest.json 和样本 meta.json 用 mask_source 标识 manual_confirmed；样本元信息附带前后确认记录。
不传 `--annotations` 时继续按原自动分割逻辑导出，不读取人工目录。

## 检查与测试

无需窗口生成检查预览（文件必须不存在；不会保存或确认标注）：

```bash
python -m airbot_ie.push_wiper.review_masks \
  --input data/push_wiper \
  --annotations data/push_wiper_annotations \
  --snapshot /tmp/push_wiper_review_preview.png
```

运行人工标注测试与既有采集/导出回归：

```bash
python -m unittest discover -s tests -p 'test_push_wiper*.py' -v
```
