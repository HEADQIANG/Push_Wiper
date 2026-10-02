"""Versioned manual masks, independent of automatic segmentation and hardware."""

import hashlib
import json
from pathlib import Path
from typing import Literal

import cv2
import numpy as np
from pydantic import Field

from .config import Settings, atomic_json


def digest(data):
    return hashlib.sha256(data).hexdigest()


class ReviewRecord(Settings):
    schema_version: Literal[1] = 1
    status: Literal["draft", "needs_review", "confirmed"]
    source: str
    source_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    roi: tuple[int, int, int, int] | None
    shape: tuple[int, int]
    mask_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")


def separate_directory(source, destination):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if destination == source or source in destination.parents:
        raise ValueError("Annotation directory must be outside the raw dataset")
    return destination


def annotation_directory(root, image_path):
    image_path = Path(image_path)
    return Path(root) / image_path.parent.parent.name / image_path.parent.name / image_path.stem


def image_identity(image_path, roi):
    path = Path(image_path)
    return {
        "source": "/".join(path.parts[-3:]),
        "source_sha256": digest(path.read_bytes()),
        "roi": tuple(roi) if roi is not None else None,
    }


def mask_statistics(mask):
    dirty = (mask == 0).astype(np.uint8)
    components = cv2.connectedComponents(dirty, connectivity=8)[0] - 1
    fraction = float(dirty.mean())
    return {
        "dirty_fraction": fraction,
        "components": components,
        "segment_scene": "simple" if components == 1 and fraction < 0.2 else "complex",
    }


def load_annotation(root, image_path, roi, shape, require_confirmed=False):
    directory = annotation_directory(root, image_path)
    record_path = directory / "review.json"
    if not record_path.exists():
        raise ValueError(f"Missing annotation: {image_path.name}")
    record = ReviewRecord.model_validate(json.loads(record_path.read_text()))
    identity = image_identity(image_path, roi)
    if any(getattr(record, key) != value for key, value in identity.items()):
        raise ValueError(f"Stale annotation (image/ROI changed): {image_path.name}")
    if record.shape != tuple(shape):
        raise ValueError(f"Annotation dimensions differ: {image_path.name}")
    payload = (directory / f"mask-{record.mask_sha256}.png").read_bytes()
    if digest(payload) != record.mask_sha256:
        raise ValueError(f"Annotation mask checksum mismatch: {image_path.name}")
    pixels = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_UNCHANGED)
    if (pixels is None or pixels.dtype != np.uint8 or pixels.shape != tuple(shape)
            or not np.isin(pixels, (0, 255)).all()):
        raise ValueError(f"Invalid binary annotation mask: {image_path.name}")
    if require_confirmed and record.status != "confirmed":
        raise ValueError(f"Annotation not confirmed ({record.status}): {image_path.name}")
    return (pixels // 255).astype(np.uint8), record


def save_annotation(root, image_path, roi, mask, status, expected_identity):
    # Do not associate an in-memory edit with an image/ROI replaced during review.
    metadata = json.loads(Path(image_path).with_name("meta.json").read_text())
    current = image_identity(image_path, metadata["camera"]["roi"])
    if current != expected_identity or current["roi"] != (tuple(roi) if roi is not None else None):
        raise ValueError("Source image or ROI changed during review; reopen the tool")
    if mask.ndim != 2 or mask.dtype != np.uint8 or not np.isin(mask, (0, 1)).all():
        raise ValueError("Manual mask must contain only uint8 values 0 and 1")
    ok, encoded = cv2.imencode(".png", mask * 255)
    if not ok:
        raise OSError("Could not encode annotation mask")
    payload = encoded.tobytes()
    record = ReviewRecord(status=status, shape=mask.shape, mask_sha256=digest(payload), **current)
    directory = annotation_directory(root, image_path)
    directory.mkdir(parents=True, exist_ok=True)
    # Immutable content-addressed PNGs keep the previous commit intact if saving fails.
    target = directory / f"mask-{record.mask_sha256}.png"
    temporary = target.with_suffix(".png.tmp")
    temporary.write_bytes(payload)
    temporary.replace(target)
    atomic_json(directory / "review.json", record.model_dump(mode="json"))
    return record
