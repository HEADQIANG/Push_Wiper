"""Offline mask + SDK-frame stroke export. Never import or control hardware."""

import argparse
import hashlib
import json
from pathlib import Path
from typing import Literal

import cv2
import numpy as np
from pydantic import Field, model_validator

from .config import Settings, WorkHeightReference, atomic_json
from .annotations import load_annotation, mask_statistics, separate_directory
from .geometry import (
    ACTION_DEFINITION,
    planar_actions,
    resample_actions,
    vector,
)


class MaskRule(Settings):
    space: Literal["hsv", "lab", "gray"] = "hsv"
    lower: list[int] = Field(default_factory=lambda: [0, 50, 20])
    upper: list[int] = Field(default_factory=lambda: [179, 255, 255])

    @model_validator(mode="after")
    def bounds(self):
        maxima = (
            [179, 255, 255]
            if self.space == "hsv"
            else ([255] if self.space == "gray" else [255] * 3)
        )
        if len(self.lower) != len(maxima) or len(self.upper) != len(maxima):
            raise ValueError("Wrong number of mask threshold channels")
        if not all(
            0 <= low <= high <= maximum
            for low, high, maximum in zip(self.lower, self.upper, maxima)
        ):
            raise ValueError("Invalid mask bounds")
        return self


class MaskConfig(Settings):
    rules: list[MaskRule] = Field(default_factory=lambda: [MaskRule()], min_length=1)
    morphology_kernel: int = Field(3, ge=1)
    min_component_area: int = Field(10, ge=1)

    @model_validator(mode="after")
    def odd_kernel(self):
        if self.morphology_kernel % 2 != 1:
            raise ValueError("Morphology kernel must be odd")
        return self


def stain_mask(image, config):
    dirty = np.zeros(image.shape[:2], dtype=np.uint8)
    conversions = {
        "hsv": cv2.COLOR_BGR2HSV,
        "lab": cv2.COLOR_BGR2LAB,
        "gray": cv2.COLOR_BGR2GRAY,
    }
    for rule in config.rules:
        converted = cv2.cvtColor(image, conversions[rule.space])
        lower, upper = (
            np.array(rule.lower, dtype=np.uint8),
            np.array(rule.upper, dtype=np.uint8),
        )
        if rule.space == "gray":
            selected = cv2.inRange(converted, int(lower[0]), int(upper[0]))
        else:
            selected = cv2.inRange(converted, lower, upper)
        dirty |= selected
    kernel = np.ones((config.morphology_kernel, config.morphology_kernel), np.uint8)
    dirty = cv2.morphologyEx(dirty, cv2.MORPH_OPEN, kernel)
    dirty = cv2.morphologyEx(dirty, cv2.MORPH_CLOSE, kernel)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(dirty, connectivity=8)
    filtered = np.zeros_like(dirty)
    components = 0
    for label in range(1, count):
        if stats[label, cv2.CC_STAT_AREA] >= config.min_component_area:
            filtered[labels == label] = 1
            components += 1
    fraction = float(filtered.mean())
    scene = "simple" if components == 1 and fraction < 0.2 else "complex"
    return (1 - filtered).astype(np.uint8), {
        "dirty_fraction": fraction,
        "components": components,
        "segment_scene": scene,
    }


def crop(image, roi):
    if image is None:
        raise ValueError("Missing or unreadable image")
    if roi is None:
        return image.copy()
    x, y, width, height = roi
    if (
        min(x, y) < 0
        or min(width, height) <= 0
        or x + width > image.shape[1]
        or y + height > image.shape[0]
    ):
        raise ValueError("ROI outside actual image")
    return image[y : y + height, x : x + width].copy()


def trajectory_plot(actions):
    image = np.full((520, 640, 3), 255, np.uint8)
    low, high = actions[:, :2].min(axis=0), actions[:, :2].max(axis=0)
    span = np.maximum(high - low, 0.02)
    low = (low + high - span) / 2
    points = (actions[:, :2] - low) / span * [500, 360] + [70, 90]
    points[:, 1] = 520 - points[:, 1]
    points = points.astype(np.int32)
    cv2.rectangle(image, (70, 70), (570, 430), (170, 170, 170), 1)
    cv2.polylines(image, [points], False, (200, 90, 20), 2)
    cv2.circle(image, tuple(points[0]), 6, (0, 160, 0), -1)
    cv2.circle(image, tuple(points[-1]), 6, (0, 0, 220), -1)
    cv2.putText(
        image,
        "SDK reference: base XY (m), NOT sponge center",
        (15, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 0, 0),
        1,
    )
    cv2.putText(
        image,
        f"x: {low[0]:.4f} .. {low[0] + span[0]:.4f}",
        (80, 480),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 0, 0),
        1,
    )
    cv2.putText(
        image,
        f"y: {low[1]:.4f} .. {low[1] + span[1]:.4f}",
        (80, 505),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 0, 0),
        1,
    )
    return image


def load_stroke(directory, metadata, max_gap_s=0.2, max_skew_s=0.1):
    start, end = metadata.get("push_start_ns"), metadata.get("push_end_ns")
    if not isinstance(start, int) or not isinstance(end, int) or end <= start:
        raise ValueError("Missing/invalid effective stroke interval")
    rows = [
        json.loads(line)
        for line in (directory / "samples.jsonl").read_text().splitlines()
        if line.strip()
    ]
    if not rows or any(b["t_ns"] <= a["t_ns"] for a, b in zip(rows, rows[1:])):
        raise ValueError("Missing or non-monotonic raw samples")
    rows = [row for row in rows if start <= row["t_ns"] <= end]
    if len(rows) < 2:
        raise ValueError("Stroke contains fewer than two samples")
    # Retain actual SDK observation times; repeated cache reads add no supervision.
    times = np.asarray([row["follow"]["t_ns"] for row in rows], dtype=np.int64)
    if np.any(np.diff(times) < 0):
        raise ValueError("Non-monotonic robot observation clock")
    unique = np.r_[True, np.diff(times) > 0]
    rows = [row for row, keep in zip(rows, unique) if keep]
    times = times[unique]
    if len(rows) < 2:
        raise ValueError("Stroke contains fewer than two distinct robot observations")
    positions = np.asarray([row["follow"]["position"] for row in rows], dtype=float)
    orientations = np.asarray(
        [row["follow"]["orientation"] for row in rows], dtype=float
    )
    if positions.shape != (len(rows), 3) or not np.isfinite(positions).all():
        raise ValueError("Invalid SDK positions")
    try:
        capture = metadata["references"]["observation"]["follow"]
        capture_orientation = capture["orientation"]
        vector(capture["position"], 3)
    except (KeyError, TypeError) as exc:
        raise ValueError(
            "Missing fixed observation reference O (p_cap); cannot define yaw"
        ) from exc
    actions = planar_actions(positions, orientations, capture_orientation)
    gaps = np.diff(times).astype(float) / 1e9
    skew = max(abs(row["camera_t_ns"] - row["follow"]["t_ns"]) / 1e9 for row in rows)
    quality = {
        "samples": len(rows),
        "duration_s": float((times[-1] - times[0]) / 1e9),
        "effective_hz": float((len(rows) - 1) / ((times[-1] - times[0]) / 1e9)),
        "max_gap_s": float(gaps.max()),
        "max_camera_robot_skew_s": float(skew),
    }
    # Legacy segments remain exportable; never infer their height from a new
    # reference file or from the retired contact reference.
    height = metadata["references"].get("work_height")
    if height is not None:
        quality["work_height_m"] = WorkHeightReference.model_validate(height).z_m
    flags = []
    for failed, flag in (
        (gaps.max() > max_gap_s, "sampling_gap"),
        (skew > max_skew_s, "camera_robot_skew"),
    ):
        if failed:
            flags.append(flag)
    quality["review_flags"] = flags
    return times, actions, positions, orientations, quality


def export_dataset(
    source,
    output,
    mask_config=None,
    validation_fraction=0.2,
    seed=42,
    include_review=False,
    allow_simulated=False,
    max_gap_s=0.2,
    max_skew_s=0.1,
    annotations=None,
):
    source, output = Path(source).resolve(), Path(output).resolve()
    if not source.is_dir():
        raise ValueError("Source dataset does not exist")
    if output == source or source in output.parents:
        raise ValueError("Export directory must be outside the source dataset")
    if annotations is not None:
        annotations = separate_directory(source, annotations)
    if not 0 <= validation_fraction < 1 or min(max_gap_s, max_skew_s) <= 0:
        raise ValueError("Invalid split fraction or timing thresholds")
    output.mkdir(parents=True, exist_ok=False)
    mask_config = mask_config or MaskConfig()
    candidates = []
    reports = []
    for path in sorted(source.glob("task_*/segment_*/meta.json")):
        directory = path.parent
        report = {"source": str(directory), "exported": False}
        try:
            metadata = json.loads(path.read_text())
            report.update(
                {"task_id": metadata["task_id"], "segment_id": metadata["segment_id"]}
            )
            if directory.name.endswith(".partial") or metadata["status"] != "accepted":
                raise ValueError("Rejected or incomplete segment")
            if metadata.get("simulated") and not allow_simulated:
                raise ValueError(
                    "Simulated segment (use --allow-simulated only for testing)"
                )
            if not (directory / "raw.mcap").is_file():
                raise ValueError("Missing raw MCAP")
            task = json.loads((directory.parent / "task.json").read_text())
            if (
                task["task_id"] != metadata["task_id"]
                or directory.name != metadata["segment_id"]
            ):
                raise ValueError("Task/segment identity mismatch")
            if not metadata.get("before") or not metadata.get("after"):
                raise ValueError("Missing before/after observation metadata")
            times, actions, positions, orientations, quality = load_stroke(
                directory, metadata, max_gap_s, max_skew_s
            )
            image = crop(
                cv2.imread(str(directory / "before.png")), metadata["camera"]["roi"]
            )
            after = crop(
                cv2.imread(str(directory / "after.png")), metadata["camera"]["roi"]
            )
            if image.shape != after.shape:
                raise ValueError("Before/after image dimensions differ")
            if annotations is None:
                mask, scene = stain_mask(image, mask_config)
                after_mask, _ = stain_mask(after, mask_config)
            else:
                records = {}
                masks = []
                for name, pixels in (("before", image), ("after", after)):
                    manual, record = load_annotation(
                        annotations, directory / f"{name}.png",
                        metadata["camera"]["roi"], pixels.shape[:2],
                        require_confirmed=True,
                    )
                    masks.append(manual)
                    records[name] = record.model_dump(mode="json")
                mask, after_mask = masks
                scene = mask_statistics(mask)
                report["annotations"] = records
            quality.update(scene)
            if scene["components"] == 0:
                quality["review_flags"].append("empty_initial_mask")
            report["quality"] = quality
            if quality["review_flags"] and not include_review:
                raise ValueError("Needs review: " + ", ".join(quality["review_flags"]))
            candidates.append(
                (
                    directory,
                    metadata,
                    times,
                    actions,
                    positions,
                    orientations,
                    image,
                    mask,
                    after_mask,
                    report,
                )
            )
        except (ValueError, KeyError, OSError, TypeError, cv2.error) as exc:
            report["reason"] = str(exc)
        reports.append(report)

    tasks = sorted(
        {item[1]["task_id"] for item in candidates},
        key=lambda task: hashlib.sha256(f"{seed}:{task}".encode()).hexdigest(),
    )
    val_count = (
        0
        if len(tasks) < 2 or validation_fraction == 0
        else min(len(tasks) - 1, max(1, round(len(tasks) * validation_fraction)))
    )
    val_tasks = set(tasks[:val_count])
    manifest = []
    for (
        directory,
        metadata,
        times,
        actions,
        positions,
        orientations,
        image,
        mask,
        after_mask,
        report,
    ) in candidates:
        split = "validation" if metadata["task_id"] in val_tasks else "train"
        relative = Path(split) / metadata["task_id"] / metadata["segment_id"]
        destination = output / relative
        destination.mkdir(parents=True)
        before = metadata["before"]["robot"]["follow"]
        capture_reference = metadata["references"]["observation"]["follow"]
        height_payload = (
            {"work_height_m": np.float64(report["quality"]["work_height_m"])}
            if "work_height_m" in report["quality"]
            else {}
        )
        np.savez_compressed(
            destination / "sample.npz",
            mask=mask,
            after_mask=after_mask,
            actions=resample_actions(times, actions),
            actions_full=actions,
            timestamps_ns=times,
            positions_full=positions,
            orientations_xyzw=orientations,
            capture_pose=np.r_[before["position"], before["orientation"]],
            capture_reference_pose=np.r_[
                capture_reference["position"], capture_reference["orientation"]
            ],
            action_definition_version=np.int64(ACTION_DEFINITION["version"]),
            **height_payload,
        )
        preview = image.copy()
        preview[mask == 0] = (0.5 * preview[mask == 0] + [0, 0, 127]).astype(np.uint8)
        for name, pixels in (
            ("mask.png", mask * 255),
            ("mask_preview.png", preview),
            ("trajectory.png", trajectory_plot(actions)),
        ):
            if not cv2.imwrite(str(destination / name), pixels):
                raise OSError(f"Could not write {destination / name}")
        report.update(
            {"exported": True, "split": split, "sample": str(relative / "sample.npz")}
        )
        atomic_json(
            destination / "meta.json",
            {
                "task_id": metadata["task_id"],
                "segment_id": metadata["segment_id"],
                "source": str(directory),
                "split": split,
                "simulated": metadata.get("simulated", False),
                "action_reference": metadata["action_reference"],
                "action_definition": ACTION_DEFINITION,
                "quaternion_order": "xyzw",
                "units": {"position": "m", "angle": "rad"},
                "resampling": "16 points, normalized observation time",
                "references": metadata["references"],
                "work_height_available": bool(height_payload),
                "quality": report["quality"],
                "mask_source": "manual_confirmed" if annotations is not None else "automatic",
                **({"annotations": report["annotations"]} if annotations is not None else {}),
            },
        )
        manifest.append(
            {key: report[key] for key in ("task_id", "segment_id", "split", "sample")}
        )
    result = {
        "exported": len(manifest),
        "excluded": len(reports) - len(manifest),
        "segments": reports,
    }
    atomic_json(
        output / "manifest.json",
        {
            "action_definition": ACTION_DEFINITION,
            "samples": manifest,
            "seed": seed,
            "validation_fraction": validation_fraction,
            "validation_tasks": sorted(val_tasks),
            "mask_source": "manual_confirmed" if annotations is not None else "automatic",
        },
    )
    atomic_json(output / "quality.json", result)
    atomic_json(output / "mask_config.json", mask_config.model_dump())
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="Raw task directory")
    parser.add_argument(
        "--output", required=True, help="New export directory (must not exist)"
    )
    parser.add_argument("--mask-config", help="JSON HSV/Lab/grayscale mask rules")
    parser.add_argument("--annotations", help="Require confirmed before/after masks from this directory")
    parser.add_argument("--validation-fraction", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--include-review", action="store_true")
    parser.add_argument("--allow-simulated", action="store_true")
    parser.add_argument("--max-gap-s", type=float, default=0.2)
    parser.add_argument("--max-skew-s", type=float, default=0.1)
    args = parser.parse_args(argv)
    try:
        mask = (
            MaskConfig.model_validate(json.loads(Path(args.mask_config).read_text()))
            if args.mask_config
            else MaskConfig()
        )
        result = export_dataset(
            args.input,
            args.output,
            mask,
            args.validation_fraction,
            args.seed,
            args.include_review,
            args.allow_simulated,
            args.max_gap_s,
            args.max_skew_s,
            annotations=args.annotations,
        )
        print(
            json.dumps(
                {
                    "exported": result["exported"],
                    "excluded": result["excluded"],
                    "report": str(Path(args.output) / "quality.json"),
                },
                ensure_ascii=False,
            )
        )
        return 0 if result["exported"] else 2
    except (ValueError, OSError) as exc:
        print(f"Export failed: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
