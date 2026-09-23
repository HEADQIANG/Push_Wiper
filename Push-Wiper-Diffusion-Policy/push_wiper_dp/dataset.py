"""Complete pushing segments, without temporal windows or held-out statistics."""

from __future__ import annotations

import copy
import hashlib
import json
from collections import Counter
from pathlib import Path, PurePosixPath
from typing import Any

import numpy as np
import torch
import torch.nn.functional as F

from diffusion_policy.dataset.base_dataset import BaseImageDataset
from diffusion_policy.model.common.normalizer import (
    LinearNormalizer,
    SingleFieldLinearNormalizer,
)


ACTION_DEFINITION = {
    "version": 2,
    "components": ["x", "y", "delta_yaw"],
    "position_reference": "follower SDK end reference in follower base",
    "yaw_reference": "references.observation.follow.orientation",
    "yaw_axis": "follower base Z",
    "yaw_convention": "ZYX yaw(R)=atan2(R[1,0], R[0,0]); unwrap(wrap(yaw(R)-yaw(R_cap)))",
    "units": {"position": "m", "angle": "rad"},
    "quaternion_order": "xyzw",
}
SOURCE_IMAGE_SIZE = (480, 640)
DEFAULT_IMAGE_SIZE = (240, 320)


def _image_size(image_size) -> tuple[int, int]:
    values = tuple(image_size)
    if (
        len(values) != 2
        or any(not isinstance(v, (int, np.integer)) or isinstance(v, bool) or v <= 0 for v in values)
        or values[0] * 4 != values[1] * 3
    ):
        raise ValueError("image_size must contain two positive integers with height:width = 3:4")
    return tuple(int(v) for v in values)


def _finite_array(value, shape: tuple[int, ...], name: str) -> np.ndarray:
    result = np.asarray(value)
    if result.shape != shape:
        raise ValueError(f"{name}: expected shape {shape}, got {result.shape}")
    if result.dtype.kind not in "biuf" or not np.isfinite(result).all():
        raise ValueError(f"{name}: expected finite real numeric values")
    with np.errstate(over="ignore"):
        result = np.asarray(result, dtype=np.float32)
    if not np.isfinite(result).all():
        raise ValueError(f"{name}: values must remain finite in float32")
    return result


def _mask_array(mask) -> np.ndarray:
    result = _finite_array(mask, SOURCE_IMAGE_SIZE, "mask")
    if not np.isin(np.asarray(mask), (0.0, 1.0)).all():
        raise ValueError("mask: expected binary values 0 (dirt) and 1 (clean)")
    if not (result == 0).any():
        raise ValueError("mask: starting observation contains no dirt pixels")
    return result


def preprocess_observation(
    mask,
    capture_reference_pose,
    image_size=DEFAULT_IMAGE_SIZE,
) -> dict[str, torch.Tensor]:
    """Return one observation step; the policy normalizer maps mask [0, 1] to [-1, 1].

    The source mask is 480x640, and the reference pose uses metres and XYZW.
    No geometry, yaw wrapping, quaternion normalization, or label repair is done.
    """
    size = _image_size(image_size)
    pixels = _mask_array(mask)
    pose = _finite_array(capture_reference_pose, (7,), "capture_reference_pose")
    image = torch.from_numpy(pixels.copy()).unsqueeze(0).unsqueeze(0)
    image = F.interpolate(image, size=size, mode="nearest").repeat(1, 3, 1, 1)
    if not torch.any(image == 0):
        raise ValueError("mask: no dirt pixels remain after nearest-neighbor resizing")
    return {
        "mask": image,
        "capture_reference_pose": torch.from_numpy(pose.copy()).unsqueeze(0),
    }


class PushWiperDataset(BaseImageDataset):
    """Load and audit every manifest entry before exposing either dataset split.

    Normalization always uses all training records, even when a caller later
    wraps this dataset in a Subset for a CPU smoke test or an overfit experiment.
    """

    def __init__(self, root: str | Path, split: str = "train", image_size=DEFAULT_IMAGE_SIZE):
        super().__init__()
        if split not in ("train", "validation"):
            raise ValueError("split must be 'train' or 'validation'")
        self.root = Path(root).expanduser().resolve()
        self.split = split
        self.image_size = _image_size(image_size)
        with (self.root / "manifest.json").open(encoding="utf-8") as stream:
            self.manifest = json.load(stream)
        if not isinstance(self.manifest, dict):
            raise ValueError("manifest must be a JSON object")
        action_definition = self.manifest.get("action_definition")
        if not isinstance(action_definition, dict) or any(
            action_definition.get(key) != expected for key, expected in ACTION_DEFINITION.items()
        ):
            raise ValueError("manifest action_definition does not match Push-Wiper action definition v2")
        records = self.manifest.get("samples")
        if not isinstance(records, list) or not records:
            raise ValueError("manifest samples must be a nonempty list")
        self._all_records = copy.deepcopy(records)
        self._raw: dict[tuple[str, str], dict[str, np.ndarray]] = {}
        self._validate_and_load()
        self.records = [r for r in self._all_records if r["split"] == split]
        self.fingerprint = self._fingerprint()

    def _validate_and_load(self) -> None:
        task_splits: dict[str, str] = {}
        paths: set[Path] = set()
        for index, record in enumerate(self._all_records):
            if not isinstance(record, dict) or any(
                not isinstance(record.get(key), str) or not record[key].strip()
                for key in ("task_id", "segment_id", "split", "sample")
            ):
                raise ValueError(f"manifest samples[{index}] requires task_id, segment_id, split, sample strings")
            split = record["split"]
            if split not in ("train", "validation"):
                raise ValueError(f"unsupported split {split!r} in samples[{index}]")
            task_id = record["task_id"]
            previous_split = task_splits.setdefault(task_id, split)
            if previous_split != split:
                raise ValueError(f"task split leakage: {task_id!r} appears in train and validation")
            relative = PurePosixPath(record["sample"])
            if relative.is_absolute() or ".." in relative.parts or "\\" in record["sample"]:
                raise ValueError(f"sample must be a safe relative path: {record['sample']!r}")
            path = (self.root / str(relative)).resolve()
            if not path.is_relative_to(self.root):
                raise ValueError(f"sample path escapes dataset root: {record['sample']!r}")
            if not path.is_file() or path.suffix != ".npz":
                raise ValueError(f"sample must be an existing NPZ file: {record['sample']!r}")
            key = (task_id, record["segment_id"])
            if key in self._raw or path in paths:
                raise ValueError(f"duplicate sample in manifest: {record['sample']!r}")
            paths.add(path)
            try:
                with np.load(path, allow_pickle=False) as sample:
                    required = {"mask", "capture_reference_pose", "actions", "action_definition_version"}
                    missing = required.difference(sample.files)
                    if missing:
                        raise ValueError(f"missing fields: {sorted(missing)}")
                    version = sample["action_definition_version"]
                    if version.shape != () or version.dtype.kind not in "iu" or int(version) != 2:
                        raise ValueError("action_definition_version must be integer scalar 2")
                    values = {
                        "mask": _mask_array(sample["mask"]).astype(np.uint8),
                        "capture_reference_pose": _finite_array(
                            sample["capture_reference_pose"], (7,), "capture_reference_pose"
                        ).copy(),
                        "actions": _finite_array(sample["actions"], (16, 3), "actions").copy(),
                    }
                    # Reject a resolution that loses all dirt at construction time.
                    preprocess_observation(values["mask"], values["capture_reference_pose"], self.image_size)
                    self._raw[key] = values
            except (ValueError, OSError, KeyError, TypeError) as error:
                raise ValueError(f"invalid sample {record['sample']!r}: {error}") from error
        counts = Counter(r["split"] for r in self._all_records)
        if not counts["train"] or not counts["validation"]:
            raise ValueError("manifest must contain nonempty train and validation splits")
        if "validation_tasks" in self.manifest:
            expected = {task for task, split in task_splits.items() if split == "validation"}
            provided = self.manifest["validation_tasks"]
            if not isinstance(provided, list) or any(not isinstance(task, str) for task in provided):
                raise ValueError("manifest validation_tasks must be a list of task IDs")
            if len(provided) != len(set(provided)) or set(provided) != expected:
                raise ValueError("manifest validation_tasks disagrees with sample splits")

    def _fingerprint(self) -> str:
        digest = hashlib.sha256()
        header = {"schema": 1, "action_definition": self.manifest["action_definition"], "image_size": self.image_size}
        digest.update(json.dumps(header, sort_keys=True, separators=(",", ":")).encode())
        # DataLoader samples by index, so manifest order is part of exact resume
        # identity even when every segment and split stays otherwise unchanged.
        for record in self._all_records:
            identity = {key: record[key] for key in ("task_id", "segment_id", "split")}
            digest.update(json.dumps(identity, sort_keys=True, separators=(",", ":")).encode())
            raw = self._raw[(record["task_id"], record["segment_id"])]
            for name in ("mask", "capture_reference_pose", "actions"):
                array = raw[name].astype("u1" if name == "mask" else "<f4", copy=False)
                digest.update(name.encode())
                digest.update(array.tobytes(order="C"))
        return digest.hexdigest()

    def __len__(self) -> int:
        return len(self.records)

    def __getitem__(self, index: int) -> dict[str, Any]:
        record = self.records[index]
        raw = self._raw[(record["task_id"], record["segment_id"])]
        return {
            "obs": preprocess_observation(raw["mask"], raw["capture_reference_pose"], self.image_size),
            "action": torch.from_numpy(raw["actions"].copy()),
        }

    def get_validation_dataset(self) -> "PushWiperDataset":
        result = copy.copy(self)
        result.split = "validation"
        result.records = [r for r in self._all_records if r["split"] == "validation"]
        return result

    def get_normalizer(self, **kwargs) -> LinearNormalizer:
        if kwargs:
            raise TypeError("normalizer settings are fixed to training-only limits in [-1, 1]")
        train = [self._raw[(r["task_id"], r["segment_id"])] for r in self._all_records if r["split"] == "train"]
        normalizer = LinearNormalizer()
        normalizer.fit(
            {
                "action": np.concatenate([sample["actions"] for sample in train], axis=0),
                "capture_reference_pose": np.stack([sample["capture_reference_pose"] for sample in train]),
            },
            last_n_dims=1,
            mode="limits",
            output_min=-1.0,
            output_max=1.0,
        )
        normalizer["mask"] = SingleFieldLinearNormalizer.create_manual(
            scale=np.array([2.0], dtype=np.float32),
            offset=np.array([-1.0], dtype=np.float32),
            input_stats_dict={
                "min": np.array([0.0], dtype=np.float32),
                "max": np.array([1.0], dtype=np.float32),
                "mean": np.array([0.5], dtype=np.float32),
                "std": np.array([0.5], dtype=np.float32),
            },
        )
        normalizer.requires_grad_(False)
        return normalizer

    def get_all_actions(self) -> torch.Tensor:
        return torch.from_numpy(np.concatenate([
            self._raw[(r["task_id"], r["segment_id"])]["actions"] for r in self.records
        ], axis=0))

    def describe(self) -> dict[str, Any]:
        train = [self._raw[(r["task_id"], r["segment_id"])] for r in self._all_records if r["split"] == "train"]
        poses = np.stack([sample["capture_reference_pose"] for sample in train])
        actions = np.concatenate([sample["actions"] for sample in train], axis=0)
        counts = {
            split: {
                "samples": sum(r["split"] == split for r in self._all_records),
                "tasks": len({r["task_id"] for r in self._all_records if r["split"] == split}),
            }
            for split in ("train", "validation")
        }
        return {
            "root": str(self.root),
            "fingerprint": self.fingerprint,
            "samples": len(self._all_records),
            "splits": counts,
            "task_split_overlap": [],
            "action_definition": copy.deepcopy(self.manifest["action_definition"]),
            "preprocessing": {
                "source_mask_shape": list(SOURCE_IMAGE_SIZE),
                "image_size": list(self.image_size),
                "mask_channels": 3,
                "mask_values": {"dirt": 0, "clean": 1},
                "mask_resize": "nearest",
                "mask_normalized_range": [-1, 1],
                "n_obs_steps": 1,
                "action_shape": [16, 3],
                "normalizer_fit_split": "train",
            },
            "train_action_min": actions.min(axis=0).tolist(),
            "train_action_max": actions.max(axis=0).tolist(),
            "train_pose_constant_dimensions": np.flatnonzero(np.ptp(poses, axis=0) == 0).tolist(),
            "minimum_source_dirt_pixels": min(int((sample["mask"] == 0).sum()) for sample in self._raw.values()),
        }
