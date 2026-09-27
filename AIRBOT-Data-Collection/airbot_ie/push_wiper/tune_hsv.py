"""Interactively tune the same HSV segmentation used by Push-Wiper export."""

import argparse
import json
import os
from pathlib import Path
import sys

import cv2
import numpy as np

from .config import atomic_json
from .export import MaskConfig, crop, stain_mask


WINDOW = "Push-Wiper HSV | N/P: next/previous | S: save | R: reset | Q: quit"
SLIDERS = [(f"{channel} {bound}", maximum)
           for channel, maximum in zip("HSV", (179, 255, 255))
           for bound in ("min", "max")]


def image_paths(source):
    if source.is_file():
        return [source]
    if source.is_dir():
        paths = sorted(p for p in source.rglob("*.png")
                       if p.name in {"before.png", "after.png"})
        # Keep each before/after pair together, starting with before.
        return sorted(paths, key=lambda p: (str(p.parent), p.name != "before.png"))
    raise ValueError(f"Input does not exist: {source}")


def read_image(path):
    """Honor recorded ROI, as export does; standalone images use the full frame."""
    meta_path = path.with_name("meta.json")
    roi = None
    if meta_path.exists() and path.name in {"before.png", "after.png"}:
        roi = json.loads(meta_path.read_text())["camera"]["roi"]
    return crop(cv2.imread(str(path)), roi)


def render(image, config, name, index, total, status=""):
    # Process at native resolution so morphology and area thresholds match export.
    mask, quality = stain_mask(image, config)
    preview = image.copy()
    preview[mask == 0] = (0.5 * preview[mask == 0] + [0, 0, 127]).astype(np.uint8)
    height, width = image.shape[:2]
    scale = min(480 / width, 360 / height, 1.0)
    size = (max(1, round(width * scale)), max(1, round(height * scale)))
    panels = []
    for title, pixels in (("Original", image),
                          ("Mask: black = stain", cv2.cvtColor(mask * 255, cv2.COLOR_GRAY2BGR)),
                          ("Overlay: red = stain", preview)):
        panel = cv2.resize(pixels, size, interpolation=cv2.INTER_NEAREST)
        panel = cv2.copyMakeBorder(panel, 32, 0, 0, 0, cv2.BORDER_CONSTANT,
                                  value=(245, 245, 245))
        cv2.putText(panel, title, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .55, (30, 30, 30), 1)
        panels.append(panel)
    body = np.hstack(panels)
    canvas = cv2.copyMakeBorder(body, 60, 75, 0, 0, cv2.BORDER_CONSTANT,
                               value=(245, 245, 245))
    rule = config.rules[0]
    lines = [
        (f"[{index + 1}/{total}] {name}", 22),
        (f"HSV min={rule.lower} max={rule.upper} | kernel={config.morphology_kernel} "
         f"min area={config.min_component_area}", 46),
        (f"Stain: {int((mask == 0).sum())} pixels ({quality['dirty_fraction']:.2%}) | "
         f"Components: {quality['components']}", canvas.shape[0] - 48),
        ("N/P: next/previous | S: save config | R: reset | Q/Esc: quit", canvas.shape[0] - 27),
        (status, canvas.shape[0] - 7),
    ]
    for text, y in lines:
        cv2.putText(canvas, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, .48, (30, 30, 30), 1)
    return canvas, mask, preview


class Tuner:
    def __init__(self, paths, config, output):
        self.paths = paths
        self.initial = config.model_copy(deep=True)
        self.config = config.model_copy(deep=True)
        self.output = output
        self.index = 0
        self.image = read_image(paths[0])
        self.ready = False
        self.dirty = True
        self.status = "Parameters apply to all images; press S to save."

    def changed(self, slider, value):
        if not self.ready:
            return
        axis, bound = divmod(slider, 2)
        rule = self.config.rules[0]
        values = rule.lower if bound == 0 else rule.upper
        values[axis] = value
        # Keep lower <= upper, including when dragging one handle past the other.
        if rule.lower[axis] > rule.upper[axis]:
            other = rule.upper if bound == 0 else rule.lower
            other[axis] = value
            self.ready = False
            cv2.setTrackbarPos(SLIDERS[slider ^ 1][0], WINDOW, value)
            self.ready = True
        self.status = "Unsaved parameters. Press S to save."
        self.dirty = True

    def sync_sliders(self):
        self.ready = False
        rule = self.config.rules[0]
        for i, (name, _) in enumerate(SLIDERS):
            cv2.setTrackbarPos(name, WINDOW, (rule.lower if i % 2 == 0 else rule.upper)[i // 2])
        self.ready = True

    def handle_key(self, key):
        if key in (ord("q"), ord("Q"), 27):
            return False
        if key in (ord("n"), ord("N"), ord("p"), ord("P")):
            step = 1 if key in (ord("n"), ord("N")) else -1
            index = (self.index + step) % len(self.paths)
            try:
                image = read_image(self.paths[index])
            except (ValueError, OSError, KeyError, cv2.error) as exc:
                self.status = f"Cannot load image: {exc}"
                print(self.status, file=sys.stderr)
            else:
                self.index, self.image = index, image
                print(f"Image {index + 1}/{len(self.paths)}: {self.paths[index]}", flush=True)
            self.dirty = True
        elif key in (ord("s"), ord("S")):
            try:
                payload = MaskConfig.model_validate(self.config.model_dump()).model_dump()
                self.output.parent.mkdir(parents=True, exist_ok=True)
                atomic_json(self.output, payload)
            except (ValueError, OSError) as exc:
                self.status = f"Save failed: {exc}"
                print(self.status, file=sys.stderr)
            else:
                self.status = "Saved config. See terminal for the full path."
                print(f"Saved HSV config: {self.output.resolve()}", flush=True)
            self.dirty = True
        elif key in (ord("r"), ord("R")):
            self.config = self.initial.model_copy(deep=True)
            self.sync_sliders()
            self.status = "Reset to startup parameters. Press S to save."
            self.dirty = True
        return True

    def run(self):
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        try:
            for i, (name, maximum) in enumerate(SLIDERS):
                cv2.createTrackbar(name, WINDOW, 0, maximum,
                                   lambda value, slider=i: self.changed(slider, value))
            self.sync_sliders()
            cv2.resizeWindow(WINDOW, 1440, 760)
            print(f"Loaded {len(self.paths)} images. N/P: browse; S: save; R: reset; Q: quit.", flush=True)
            while cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) >= 1:
                if self.dirty:
                    path = self.paths[self.index]
                    canvas, _, _ = render(self.image, self.config, f"{path.parent.name}/{path.name}",
                                          self.index, len(self.paths), self.status)
                    cv2.imshow(WINDOW, canvas)
                    self.dirty = False
                if not self.handle_key(cv2.waitKey(30) & 0xFF):
                    break
        finally:
            cv2.destroyAllWindows()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=Path("data/push_wiper"),
                        help="单张图片，或递归查找 before.png/after.png 的目录")
    parser.add_argument("--mask-config", type=Path, help="载入配置；要求只有一条 HSV 规则")
    parser.add_argument("--output-config", type=Path, default=Path("data/push_wiper_mask_tuned.json"),
                        help="按 S 保存的配置路径；重复保存会更新该文件")
    parser.add_argument("--snapshot", type=Path,
                        help="无窗口模式：将首张图的预览和掩码写入一个不存在的新目录")
    args = parser.parse_args(argv)
    try:
        config = (MaskConfig.model_validate(json.loads(args.mask_config.read_text()))
                  if args.mask_config else MaskConfig())
        if len(config.rules) != 1 or config.rules[0].space != "hsv":
            raise ValueError("This tuner requires exactly one HSV rule; Lab/gray/multiple rules are not edited.")
        paths = image_paths(args.input)
        if not paths:
            raise ValueError("No before.png/after.png found; alternatively pass a single image path.")
        if args.output_config.suffix.lower() != ".json":
            raise ValueError("--output-config must be a .json file")
        if args.output_config.name in {"meta.json", "task.json", "samples.jsonl"}:
            raise ValueError("Choose a mask config path, not a raw recording metadata file.")
        tuner = Tuner(paths, config, args.output_config)
        if args.snapshot:
            canvas, mask, preview = render(tuner.image, config, paths[0].name, 0, len(paths))
            args.snapshot.mkdir(parents=True, exist_ok=False)
            for name, pixels in (("comparison.png", canvas), ("mask.png", mask * 255),
                                 ("mask_preview.png", preview)):
                if not cv2.imwrite(str(args.snapshot / name), pixels):
                    raise OSError(f"Could not write {name}")
            atomic_json(args.snapshot / "mask_config.json", config.model_dump())
            print(f"Snapshot saved: {args.snapshot.resolve()}")
        else:
            if sys.platform.startswith("linux") and not (os.getenv("DISPLAY") or os.getenv("WAYLAND_DISPLAY")):
                raise ValueError("没有图形桌面，请在本机桌面终端运行，或使用 --snapshot 输出预览。")
            tuner.run()
        return 0
    except (ValueError, OSError, KeyError, cv2.error) as exc:
        print(f"HSV tuner: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
