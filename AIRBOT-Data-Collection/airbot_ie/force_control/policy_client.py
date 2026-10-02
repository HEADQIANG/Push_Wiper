"""Client for the isolated Push-Wiper Diffusion Policy process.

The AIRBOT environment intentionally does not import torch.  This small
JSON-lines client starts the policy service in its own Python environment and
exchanges one full 16-point planar action segment at a time.
"""

from __future__ import annotations

import base64
import json
import os
import selectors
import subprocess
import threading
import time
import uuid
from pathlib import Path

import numpy as np


ACTION_DEFINITION_VERSION = 2
MASK_SHAPE = (480, 640)


def encode_mask(mask: np.ndarray) -> str:
    """Validate and encode a binary 480x640 uint8 mask."""

    value = np.asarray(mask)
    if value.shape != MASK_SHAPE or value.dtype != np.uint8:
        raise ValueError(f"mask must be uint8 with shape {MASK_SHAPE}")
    if not np.isin(value, (0, 1)).all():
        raise ValueError("mask must contain only 0 (dirt) and 1 (clean)")
    if not np.any(value == 0):
        raise ValueError("mask must contain at least one dirt pixel")
    return base64.b64encode(value.tobytes(order="C")).decode("ascii")


def validate_actions(value, *, expected_version: int = ACTION_DEFINITION_VERSION) -> np.ndarray:
    """Validate a service response and return float32 [16, 3] actions."""

    if not isinstance(value, dict) or not value.get("ok"):
        error = value.get("error", "unknown policy service error") if isinstance(value, dict) else str(value)
        raise RuntimeError(f"Policy service rejected request: {error}")
    version = value.get("action_definition_version", expected_version)
    if int(version) != expected_version:
        raise ValueError(
            f"Unsupported action_definition_version={version}; expected {expected_version}"
        )
    actions = np.asarray(value.get("actions"), dtype=np.float64)
    if actions.shape != (16, 3) or not np.isfinite(actions).all():
        raise ValueError("Policy service returned non-finite actions with shape other than (16, 3)")
    return actions.astype(np.float32)


def validate_planar_actions(actions, safety: dict) -> np.ndarray:
    """Reject a complete segment that exceeds configured workspace limits."""

    report = inspect_planar_actions(actions, safety)
    if not report["ok"]:
        raise ValueError(report["violations"][0])
    return np.asarray(actions, dtype=np.float32)


def inspect_planar_actions(actions, safety: dict) -> dict:
    """Return action safety metrics without clipping or raising.

    Offline inspection needs to preserve an unsafe prediction so that the
    operator can see why the deployment gate rejected it.  The live path still
    calls :func:`validate_planar_actions`, which raises on the first violation.
    """

    values = np.asarray(actions, dtype=float)
    report = {
        "ok": False,
        "violations": [],
        "metrics": {},
        "limits": {},
    }
    if values.shape != (16, 3) or not np.isfinite(values).all():
        report["violations"].append("Policy actions must be finite with shape (16, 3)")
        return report

    workspace = safety.get("workspace")
    if not isinstance(workspace, dict):
        report["violations"].append("safety.workspace must be configured")
        return report
    limits = report["limits"]
    for axis, index in (("x", 0), ("y", 1)):
        low, high = float(workspace[f"{axis}_min_m"]), float(workspace[f"{axis}_max_m"])
        limits[f"{axis}_min_m"], limits[f"{axis}_max_m"] = low, high
        report["metrics"][f"{axis}_min_m"] = float(values[:, index].min())
        report["metrics"][f"{axis}_max_m"] = float(values[:, index].max())
        if low >= high or np.any(values[:, index] < low) or np.any(values[:, index] > high):
            report["violations"].append(f"Policy {axis} actions exceed the configured workspace")
    optional_limits = {
        "max_action_step_m": safety.get("max_action_step_m"),
        "max_action_yaw_rad": safety.get("max_action_yaw_rad"),
        "max_abs_yaw_rad": safety.get("max_abs_yaw_rad"),
    }
    parsed_limits = {}
    for key, raw_value in optional_limits.items():
        if raw_value is None:
            parsed_limits[key] = None
            continue
        try:
            value = float(raw_value)
        except (TypeError, ValueError):
            report["violations"].append(f"{key} must be positive or null")
            parsed_limits[key] = None
            continue
        if not np.isfinite(value) or value <= 0:
            report["violations"].append(f"{key} must be positive or null")
            parsed_limits[key] = None
            continue
        parsed_limits[key] = value
    max_step = parsed_limits["max_action_step_m"]
    max_yaw = parsed_limits["max_action_yaw_rad"]
    max_abs_yaw = parsed_limits["max_abs_yaw_rad"]
    limits.update(parsed_limits)
    report["metrics"].update(
        xy_step_max_m=float(np.linalg.norm(np.diff(values[:, :2], axis=0), axis=1).max()),
        delta_yaw_min_rad=float(values[:, 2].min()),
        delta_yaw_max_rad=float(values[:, 2].max()),
        delta_yaw_abs_max_rad=float(np.abs(values[:, 2]).max()),
        yaw_step_max_rad=float(np.abs(np.diff(values[:, 2])).max()),
    )
    if max_step is not None and np.any(np.linalg.norm(np.diff(values[:, :2], axis=0), axis=1) > max_step):
        report["violations"].append(f"Policy XY step exceeds {max_step:g} m")
    if max_abs_yaw is not None and np.any(np.abs(values[:, 2]) > max_abs_yaw):
        report["violations"].append("Policy delta_yaw exceeds the configured absolute limit")
    if max_yaw is not None and np.any(np.abs(np.diff(values[:, 2])) > max_yaw):
        report["violations"].append(f"Policy yaw step exceeds {max_yaw:g} rad")
    report["ok"] = not report["violations"]
    return report


class PolicyClient:
    """Start and communicate with ``push_wiper_dp.policy_service``."""

    def __init__(
        self,
        checkpoint: str | Path,
        policy_python: str | Path,
        policy_root: str | Path,
        device: str = "cuda:0",
        startup_timeout_s: float = 120.0,
        request_timeout_s: float = 120.0,
    ) -> None:
        self.checkpoint = Path(checkpoint).expanduser().resolve()
        self.policy_python = str(Path(policy_python).expanduser())
        self.policy_root = Path(policy_root).expanduser().resolve()
        self.device = str(device)
        self.startup_timeout_s = float(startup_timeout_s)
        self.request_timeout_s = float(request_timeout_s)
        if not self.checkpoint.is_file():
            raise FileNotFoundError(f"Policy checkpoint does not exist: {self.checkpoint}")
        if self.startup_timeout_s <= 0 or self.request_timeout_s <= 0:
            raise ValueError("Policy service timeouts must be positive")
        self.process: subprocess.Popen[str] | None = None
        self._ready = threading.Event()
        self._stderr_lines: list[str] = []
        self._stderr_thread: threading.Thread | None = None
        self._lock = threading.Lock()

    def start(self) -> None:
        if self.process is not None:
            raise RuntimeError("Policy service is already started")
        env = os.environ.copy()
        existing = env.get("PYTHONPATH", "")
        env["PYTHONPATH"] = str(self.policy_root) + (os.pathsep + existing if existing else "")
        self.process = subprocess.Popen(
            [
                self.policy_python,
                "-m",
                "push_wiper_dp.policy_service",
                "--checkpoint",
                str(self.checkpoint),
                "--device",
                self.device,
            ],
            cwd=str(self.policy_root),
            env=env,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        self._stderr_thread = threading.Thread(target=self._read_stderr, daemon=True)
        self._stderr_thread.start()
        deadline = time.monotonic() + self.startup_timeout_s
        while time.monotonic() < deadline:
            if self._ready.wait(0.05):
                return
            if self.process.poll() is not None:
                message = self._service_error("policy service exited during startup")
                self.close()
                raise RuntimeError(message)
        message = self._service_error("timed out waiting for policy service readiness")
        self.close()
        raise TimeoutError(message)

    def _read_stderr(self) -> None:
        process = self.process
        if process is None or process.stderr is None:
            return
        for line in process.stderr:
            line = line.rstrip()
            if line:
                self._stderr_lines.append(line)
                try:
                    if json.loads(line).get("ready") is True:
                        self._ready.set()
                except (ValueError, AttributeError):
                    pass

    def _service_error(self, prefix: str) -> str:
        details = "; ".join(self._stderr_lines[-3:])
        return f"{prefix}{(': ' + details) if details else ''}"

    def predict(self, mask: np.ndarray, capture_reference_pose, seed: int = 42) -> np.ndarray:
        if self.process is None or self.process.poll() is not None:
            raise RuntimeError(self._service_error("policy service is not running"))
        pose = np.asarray(capture_reference_pose, dtype=np.float32)
        if pose.shape != (7,) or not np.isfinite(pose).all():
            raise ValueError("capture_reference_pose must contain seven finite values")
        payload = {
            "request_id": uuid.uuid4().hex,
            "mask_b64": encode_mask(mask),
            "capture_reference_pose": pose.tolist(),
            "seed": int(seed),
        }
        line = json.dumps(payload, separators=(",", ":")) + "\n"
        with self._lock:
            assert self.process.stdin is not None and self.process.stdout is not None
            try:
                self.process.stdin.write(line)
                self.process.stdin.flush()
            except (BrokenPipeError, OSError) as exc:
                raise RuntimeError(self._service_error("could not send policy request")) from exc
            selector = selectors.DefaultSelector()
            try:
                selector.register(self.process.stdout, selectors.EVENT_READ)
                events = selector.select(self.request_timeout_s)
            finally:
                selector.close()
            if not events:
                raise TimeoutError(self._service_error("timed out waiting for policy response"))
            response_line = self.process.stdout.readline()
        if not response_line:
            raise RuntimeError(self._service_error("policy service closed stdout"))
        try:
            response = json.loads(response_line)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"Invalid policy service response: {response_line!r}") from exc
        if response.get("request_id") not in (payload["request_id"], None):
            raise RuntimeError("Policy response request_id does not match the request")
        return validate_actions(response)

    def close(self) -> None:
        process, self.process = self.process, None
        if process is None:
            return
        try:
            if process.stdin is not None:
                process.stdin.close()
        except OSError:
            pass
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3.0)

    def __enter__(self) -> "PolicyClient":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
