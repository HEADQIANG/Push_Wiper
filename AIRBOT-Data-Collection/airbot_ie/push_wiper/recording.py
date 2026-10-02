"""Durable segment boundaries and existing AIRDC MCAP serialization."""

import json
import os
import time
import uuid
from collections import defaultdict
from copy import deepcopy
from pathlib import Path

import cv2

from .config import atomic_json
from .clock import TIMESTAMP_BASIS, acquisition_ns


def identifier(prefix):
    return f"{prefix}_{time.strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:12]}"


class SegmentWriter:
    def __init__(self, task_directory, metadata, sampler_factory=None):
        if sampler_factory is None:
            from airdc.common.samplers.mcap_sampler import McapDataSampler

            sampler_factory = McapDataSampler
        self.segment_id = identifier("segment")
        self.directory = Path(task_directory) / (self.segment_id + ".partial")
        self.directory.mkdir(parents=True, exist_ok=False)
        self.metadata = deepcopy(metadata) | {
            "schema_version": 1,
            "segment_id": self.segment_id,
            "status": "incomplete",
            "push_start_ns": None,
            "push_end_ns": None,
            "events": [],
            "sample_count": 0,
            "timestamp_basis": TIMESTAMP_BASIS,
        }
        self.remaining = defaultdict(list)
        self.last_frame_t = None
        self.closed = False
        self.trajectory = (self.directory / "samples.jsonl").open("x", encoding="utf-8")
        self.sampler = sampler_factory()
        self.sampler.set_info({"push_wiper": deepcopy(metadata)})
        if not self.sampler.configure():
            self.trajectory.close()
            raise RuntimeError("MCAP sampler configuration failed")
        self.raw_path = self.sampler.compose_path(self.directory, 0)
        self.checkpoint()

    def checkpoint(self):
        self.trajectory.flush()
        os.fsync(self.trajectory.fileno())
        atomic_json(self.directory / "meta.json", self.metadata)

    def event(self, name, t_ns, **details):
        self.metadata["events"].append({"name": name, "t_ns": int(t_ns), **details})
        self.checkpoint()

    def image(self, name, frame, snapshot):
        if not cv2.imwrite(str(self.directory / f"{name}.png"), frame["data"]):
            raise OSError(f"Could not write {name} image")
        self.metadata[name] = {
            "image_t_ns": int(frame["t"]),
            "robot": deepcopy(snapshot["samples"]),
            "reset_errors": deepcopy(snapshot["errors"]),
        }
        self.checkpoint()

    def append(self, snapshot, frame, phase, stamp_ns=None):
        stamp_ns = acquisition_ns() if stamp_ns is None else int(stamp_ns)
        row = {
            "t_ns": stamp_ns,
            "camera_t_ns": int(frame["t"]),
            "phase": phase,
            "controller_state": snapshot["state"],
            **deepcopy(snapshot["samples"]),
        }
        if (
            self.metadata["sample_count"]
            and stamp_ns <= self.metadata["last_sample_ns"]
        ):
            raise ValueError(
                "Non-monotonic collection clock: "
                f"current={stamp_ns}, previous={self.metadata['last_sample_ns']}"
            )
        payload = {"log_stamps": stamp_ns}
        for name, sample in snapshot["samples"].items():
            for field, suffix in (
                ("joints", "arm/joint_state/position"),
                ("velocity", "arm/joint_state/velocity"),
                ("effort", "arm/joint_state/effort"),
                ("position", "eef/pose/position"),
                ("orientation", "eef/pose/orientation"),
            ):
                if field in sample:
                    payload[f"/{name}/{suffix}"] = {
                        "t": sample["t_ns"],
                        "data": sample[field],
                    }
        if frame["t"] != self.last_frame_t:
            payload["/wrist_camera/color/image_raw"] = frame
            self.last_frame_t = frame["t"]
        for key, value in self.sampler.update(payload).items():
            self.remaining[key].append(value)
        self.trajectory.write(json.dumps(row, allow_nan=False) + "\n")
        self.trajectory.flush()
        self.metadata["sample_count"] += 1
        self.metadata.setdefault("first_sample_ns", stamp_ns)
        self.metadata["last_sample_ns"] = stamp_ns
        return stamp_ns

    def finish(self, status, reason=""):
        if self.closed:
            raise RuntimeError("Segment already closed")
        if status == "accepted" and not all(
            self.metadata.get(key)
            for key in ("before", "after", "push_start_ns", "push_end_ns")
        ):
            raise ValueError("Accepted segment is missing images or stroke boundaries")
        # Final accepted status is only published AFTER MCAP and sidecars finish.
        self.metadata["reason"] = reason
        self.checkpoint()
        try:
            if not self.sampler.save(self.raw_path, dict(self.remaining)):
                raise OSError("MCAP save returned failure")
            Path(self.raw_path).replace(self.directory / "raw.mcap")
            self.metadata["status"] = status
            self.checkpoint()
            self.trajectory.close()
            self.sampler.shutdown()
            final = self.directory.with_name(self.segment_id)
            self.directory.rename(final)
            self.directory = final
            self.closed = True
            return final
        except BaseException:
            self.metadata["status"] = "incomplete"
            self.metadata["reason"] = "Save interrupted or failed"
            try:
                atomic_json(self.directory / "meta.json", self.metadata)
            finally:
                self.trajectory.close()
                self.closed = True
            raise
