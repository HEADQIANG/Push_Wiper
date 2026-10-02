"""Review and paint manual stain masks without changing raw recordings."""

import argparse
from collections import Counter
import json
import os
from pathlib import Path
import sys

import cv2
import numpy as np

from .annotations import (
    annotation_directory, digest, load_annotation, mask_statistics,
    save_annotation, separate_directory,
)
from .export import MaskConfig, crop, stain_mask


WINDOW = "Push-Wiper Manual Review"
ERRORS = (ValueError, OSError, KeyError, TypeError, cv2.error)


def review_paths(source):
    source = Path(source)
    if not source.is_dir():
        raise ValueError(f"Raw dataset does not exist: {source}")
    paths = []
    for meta_path in sorted(source.glob("task_*/segment_*/meta.json")):
        if meta_path.parent.name.endswith(".partial"):
            continue
        meta = json.loads(meta_path.read_text())
        if meta.get("status") == "accepted":
            paths.extend(meta_path.with_name(f"{name}.png") for name in ("before", "after"))
    if not paths:
        raise ValueError("No accepted segments found")
    return paths


class ReviewImage:
    def __init__(self, path, annotations, config):
        self.path, self.annotations = Path(path), Path(annotations)
        metadata = json.loads(self.path.with_name("meta.json").read_text())
        self.roi = metadata["camera"]["roi"]
        payload = self.path.read_bytes()
        self.image = crop(cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR), self.roi)
        self.identity = {
            "source": "/".join(self.path.parts[-3:]),
            "source_sha256": digest(payload),
            "roi": tuple(self.roi) if self.roi is not None else None,
        }
        self.error = ""
        self.status = "unreviewed"
        self.dirty = False
        self.undo_stack = []
        self.stroke_start = None
        record_path = annotation_directory(annotations, path) / "review.json"
        if record_path.exists():
            try:
                self.mask, record = load_annotation(annotations, path, self.roi, self.image.shape[:2])
                self.status = record.status
            except ERRORS as exc:
                # Never silently replace a corrupt or stale manual annotation.
                self.error = str(exc)
                self.status = "invalid"
                self.mask = np.ones(self.image.shape[:2], np.uint8)
        else:
            self.mask, _ = stain_mask(self.image, config)

    def save(self, status=None):
        if self.error:
            raise ValueError(self.error)
        state = status or (self.status if self.status != "unreviewed" else "draft")
        save_annotation(self.annotations, self.path, self.roi, self.mask, state, self.identity)
        self.status, self.dirty = state, False

    def invalidate_confirmation(self):
        if self.error:
            raise ValueError(self.error)
        if self.status == "confirmed":
            # Revoke on disk before the first modification, even if this process crashes.
            self.save("draft")
        self.status = "draft"

    def begin_stroke(self):
        self.invalidate_confirmation()
        self.stroke_start = self.mask.copy()

    def paint(self, start, end, radius, erase=False):
        if self.stroke_start is None:
            self.begin_stroke()
        value = 1 if erase else 0
        cv2.line(self.mask, start, end, value, max(1, 2 * radius), cv2.LINE_8)
        cv2.circle(self.mask, end, radius, value, -1, cv2.LINE_8)
        self.dirty = True

    def end_stroke(self):
        if self.stroke_start is not None:
            if not np.array_equal(self.stroke_start, self.mask):
                self.undo_stack.append(self.stroke_start)
                self.undo_stack = self.undo_stack[-30:]
            self.stroke_start = None

    def undo(self):
        self.end_stroke()
        if self.undo_stack:
            self.invalidate_confirmation()
            self.mask = self.undo_stack.pop()
            self.dirty = True


class Viewport:
    """Explicit viewport makes display scaling and mouse coordinates share one mapping."""
    panel_width, panel_height, top = 420, 315, 104

    def __init__(self, shape):
        self.height, self.width = shape
        self.zoom = 1.0
        self.cx, self.cy = self.width / 2, self.height / 2

    def geometry(self):
        w = max(1, round(self.width / self.zoom))
        h = max(1, round(self.height / self.zoom))
        x = int(np.clip(round(self.cx - w / 2), 0, self.width - w))
        y = int(np.clip(round(self.cy - h / 2), 0, self.height - h))
        scale = min(self.panel_width / w, self.panel_height / h)
        dw, dh = max(1, round(w * scale)), max(1, round(h * scale))
        ox, oy = (self.panel_width - dw) // 2, (self.panel_height - dh) // 2
        return x, y, w, h, ox, oy, dw, dh

    def point(self, mx, my):
        panel = mx // self.panel_width
        if panel not in (1, 2):
            return None
        x, y, w, h, ox, oy, dw, dh = self.geometry()
        px = mx - panel * self.panel_width - ox
        py = my - self.top - oy
        if not (0 <= px < dw and 0 <= py < dh):
            return None
        return x + min(w - 1, int(px * w / dw)), y + min(h - 1, int(py * h / dh))

    def change_zoom(self, factor):
        self.zoom = float(np.clip(self.zoom * factor, 1, 8))

    def pan(self, dx, dy):
        self.cx = float(np.clip(self.cx + dx * self.width / self.zoom / 4, 0, self.width))
        self.cy = float(np.clip(self.cy + dy * self.height / self.zoom / 4, 0, self.height))


class Reviewer:
    def __init__(self, source, annotations, config):
        self.annotations = separate_directory(source, annotations)
        self.paths = review_paths(source)
        self.config = config
        self.states = []
        for path in self.paths:
            try:
                self.states.append(ReviewImage(path, self.annotations, config).status)
            except ERRORS:
                self.states.append("invalid")
        self.index = next((i for i, state in enumerate(self.states) if state != "confirmed"), 0)
        self.current = None
        self.view = None
        self.brush = 5
        self.erase = False
        self.overlay = True
        self.last_point = None
        self.last_panel = None
        self.message = "B/E: paint/erase on Mask or Overlay. Enter confirms this image only."
        self.redraw = True
        self.load(self.index)

    def load(self, index):
        try:
            current = ReviewImage(self.paths[index], self.annotations, self.config)
        except ERRORS as exc:
            current = None
            self.message = f"Cannot load: {exc}"
            self.states[index] = "invalid"
        self.index, self.current = index, current
        if current:
            self.view = Viewport(current.mask.shape)
            self.states[index] = current.status
            self.message = current.error or f"Loaded {current.path.name}; edits are saved as drafts."
        self.redraw = True

    def end_stroke(self):
        if self.current:
            self.current.end_stroke()
        self.last_point = None
        self.last_panel = None

    def autosave(self):
        self.end_stroke()
        if self.current and self.current.dirty:
            self.current.save()
        if self.current:
            self.states[self.index] = self.current.status

    def fail(self, exc):
        self.message = f"ERROR: {exc}"
        self.redraw = True
        print(self.message, file=sys.stderr, flush=True)

    def mouse(self, event, x, y, flags, _param):
        if not self.current or self.current.error:
            return
        try:
            point = self.view.point(x, y)
            panel = x // self.view.panel_width
            if event == cv2.EVENT_LBUTTONDOWN and point is not None:
                self.current.begin_stroke()
                self.current.paint(point, point, self.brush, self.erase)
                self.last_point = point
                self.last_panel = panel
            elif event == cv2.EVENT_MOUSEMOVE and flags & cv2.EVENT_FLAG_LBUTTON:
                if self.current.stroke_start is not None and point is not None:
                    start = self.last_point if self.last_panel == panel else None
                    self.current.paint(start or point, point, self.brush, self.erase)
                    self.last_point = point
                    self.last_panel = panel
                elif point is None:
                    self.last_point = None
                    self.last_panel = None
            elif event == cv2.EVENT_LBUTTONUP:
                self.end_stroke()
            self.states[self.index] = self.current.status
            self.redraw = True
        except ERRORS as exc:
            self.end_stroke()
            self.fail(exc)

    def handle_key(self, key):
        try:
            if key in (ord("q"), ord("Q"), 27):
                self.autosave()
                return False
            if key in (ord("n"), ord("N"), ord("p"), ord("P")):
                self.autosave()
                step = 1 if key in (ord("n"), ord("N")) else -1
                self.load((self.index + step) % len(self.paths))
            elif self.current:
                self.end_stroke()
                if key in (ord("s"), ord("S"), 10, 13, ord("u"), ord("U")):
                    status = ("confirmed" if key in (10, 13) else
                              "needs_review" if key in (ord("u"), ord("U")) else "draft")
                    self.current.save(status)
                    self.message = f"Saved: {status}"
                elif key in (ord("b"), ord("B")):
                    self.erase = False
                elif key in (ord("e"), ord("E")):
                    self.erase = True
                elif key in (ord("z"), ord("Z")):
                    self.current.undo()
                elif key == ord("["):
                    self.brush = max(1, self.brush - 1)
                elif key == ord("]"):
                    self.brush = min(100, self.brush + 1)
                elif key in (ord("o"), ord("O")):
                    self.overlay = not self.overlay
                elif key in (ord("+"), ord("="), ord("-")):
                    self.view.change_zoom(1 / 1.5 if key == ord("-") else 1.5)
                elif chr(key).lower() in "ijkl":
                    dx, dy = {"i": (0, -1), "j": (-1, 0), "k": (0, 1), "l": (1, 0)}[chr(key).lower()]
                    self.view.pan(dx, dy)
                self.states[self.index] = self.current.status
            self.redraw = True
        except ERRORS as exc:
            self.fail(exc)
        return True

    def render(self):
        canvas = np.full((535, 1260, 3), 245, np.uint8)
        counts = Counter(self.states)
        current = self.current
        label = "/".join(self.paths[self.index].parts[-3:])
        lines = [(f"[{self.index + 1}/{len(self.paths)}] {label}", 20),
                 (" | ".join(f"{state}: {counts[state]}" for state in
                             ("unreviewed", "draft", "needs_review", "confirmed", "invalid")), 42)]
        if current:
            stats = mask_statistics(current.mask)
            lines.append((f"State: {current.status}{' (unsaved)' if current.dirty else ''} | "
                          f"Tool: {'ERASER' if self.erase else 'BRUSH'} | radius={self.brush}px | "
                          f"zoom={self.view.zoom:.2f} | stain={stats['dirty_fraction']:.2%}", 64))
            image = current.image
            overlay = image.copy()
            if self.overlay and not current.error:
                overlay[current.mask == 0] = (0.5 * overlay[current.mask == 0] + [0, 0, 127]).astype(np.uint8)
            panels = (image, cv2.cvtColor(current.mask * 255, cv2.COLOR_GRAY2BGR), overlay)
            x, y, w, h, ox, oy, dw, dh = self.view.geometry()
            for i, (pixels, title) in enumerate(zip(panels, ("Original", "Mask: black = stain", "Overlay: red = stain"))):
                left, top = i * self.view.panel_width + ox, self.view.top + oy
                canvas[top:top + dh, left:left + dw] = cv2.resize(
                    pixels[y:y + h, x:x + w], (dw, dh), interpolation=cv2.INTER_NEAREST)
                lines.append((title, 94, i * self.view.panel_width + 8))
        lines.extend([
            ("N/P: next/previous | B/E: brush/eraser | [/]: size | Z: undo | O: toggle overlay", 447),
            ("+/-: zoom | I/J/K/L: pan up/left/down/right | S: save draft | Enter: confirm", 469),
            ("U: needs review | Q/Esc: save edits and exit | A white mask is NOT a confirmed clean image.", 491),
            (self.message[:160], 519),
        ])
        for line in lines:
            text, y = line[:2]
            x = line[2] if len(line) == 3 else 8
            cv2.putText(canvas, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, .48, (30, 30, 30), 1)
        if current and current.error:
            cv2.putText(canvas, "INVALID ANNOTATION - EDITING BLOCKED", (450, 250),
                        cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 0, 200), 2)
        return canvas

    def open_window(self):
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
        cv2.resizeWindow(WINDOW, 1260, 535)
        cv2.setMouseCallback(WINDOW, self.mouse)
        self.redraw = True

    def run(self):
        self.open_window()
        try:
            while True:
                try:
                    try:
                        visible = cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) >= 1
                    except cv2.error:
                        visible = False
                    if not visible:
                        try:
                            self.autosave()
                            break
                        except ERRORS as exc:
                            self.open_window()
                            self.fail(exc)
                    if self.redraw:
                        cv2.imshow(WINDOW, self.render())
                        self.redraw = False
                    key = cv2.waitKey(30) & 0xFF
                    if key != 255 and not self.handle_key(key):
                        break
                except KeyboardInterrupt:
                    if not self.handle_key(ord("q")):
                        break
        finally:
            cv2.destroyAllWindows()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=Path("data/push_wiper"))
    parser.add_argument("--annotations", type=Path, default=Path("data/push_wiper_annotations"))
    parser.add_argument("--mask-config", type=Path, help="Automatic draft settings; existing manual masks are kept")
    parser.add_argument("--snapshot", type=Path, help="Write a preview PNG without opening a window or confirming images")
    args = parser.parse_args(argv)
    try:
        config = (MaskConfig.model_validate(json.loads(args.mask_config.read_text()))
                  if args.mask_config else MaskConfig())
        reviewer = Reviewer(args.input, args.annotations, config)
        print(f"Images: {len(reviewer.paths)}; annotations: {reviewer.annotations}", flush=True)
        if args.snapshot:
            if args.snapshot.exists():
                raise ValueError("Snapshot output already exists")
            args.snapshot.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(args.snapshot), reviewer.render()):
                raise OSError("Could not save preview")
        else:
            if sys.platform.startswith("linux") and not (os.getenv("DISPLAY") or os.getenv("WAYLAND_DISPLAY")):
                raise ValueError("没有图形桌面，请在本机桌面终端运行，或使用 --snapshot 输出预览")
            reviewer.run()
        return 0
    except ERRORS as exc:
        print(f"Mask review: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
