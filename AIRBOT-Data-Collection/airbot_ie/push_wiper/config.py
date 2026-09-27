"""Explicit, portable configuration; no TCP or camera calibration is required."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator


class Settings(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)


class WorkHeightReference(Settings):
    z_m: float
    t_ns: int = Field(gt=0)
    frame: Literal["follower_base"] = "follower_base"
    point: Literal["sdk_end_reference"] = "sdk_end_reference"
    unit: Literal["m"] = "m"


class RobotConfig(Settings):
    mode: Literal["teleop", "single_drag"] = "teleop"
    lead_url: str = "localhost"
    lead_port: int = Field(50050, ge=1, le=65535)
    follow_url: str = "localhost"
    follow_port: int = Field(50051, ge=1, le=65535)
    control_hz: float = Field(100, gt=0)
    feedback_timeout_s: float = Field(0.5, gt=0)
    command_timeout_s: float = Field(10, gt=0)

    @model_validator(mode="after")
    def distinct(self):
        if self.mode == "teleop" and (self.lead_url, self.lead_port) == (
            self.follow_url, self.follow_port
        ):
            raise ValueError("Lead and follow endpoints must be different")
        return self

    @property
    def arm_names(self) -> tuple[str, ...]:
        return ("follow",) if self.mode == "single_drag" else ("lead", "follow")


class ResetConfig(Settings):
    done_position_m: float = Field(0.002, gt=0)
    done_angle_deg: float = Field(1, gt=0)
    done_joint_rad: float = Field(0.01, gt=0)
    follow_joint_speed_rad_s: float = Field(0.5, gt=0)
    stable_s: float = Field(0.5, gt=0)
    timeout_s: float = Field(10, gt=0)


class CameraConfig(Settings):
    serial: str = ""
    width: int = Field(640, gt=0)
    height: int = Field(480, gt=0)
    fps: int = Field(30, gt=0)
    stale_s: float = Field(0.5, gt=0)
    # x, y, width, height; null means the full frame, fixed for the session.
    roi: tuple[int, int, int, int] | None = None

    @model_validator(mode="after")
    def roi_inside_image(self):
        if self.roi is not None:
            x, y, w, h = self.roi
            if (
                min(x, y) < 0
                or min(w, h) <= 0
                or x + w > self.width
                or y + h > self.height
            ):
                raise ValueError("ROI must be inside the configured image")
        return self


class CollectionConfig(Settings):
    robot: RobotConfig = Field(default_factory=RobotConfig)
    reset: ResetConfig = Field(default_factory=ResetConfig)
    camera: CameraConfig = Field(default_factory=CameraConfig)
    output: str = "data/push_wiper"
    references: str = "data/push_wiper_references.json"
    sample_hz: float = Field(20, gt=0)
    max_segment_s: float = Field(180, gt=0)
    stain: str = "ketchup"
    operator: str = ""


def load_config(path: str | Path | None) -> CollectionConfig:
    return (
        CollectionConfig.model_validate(json.loads(Path(path).read_text()))
        if path
        else CollectionConfig()
    )


def atomic_json(path: Path, value):
    """The final filename never contains a partially written JSON document."""
    import os

    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)
