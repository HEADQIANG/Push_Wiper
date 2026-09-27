"""Hardware adapters for the standalone force-position controller."""

from __future__ import annotations

import csv
import logging
import struct
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import numpy as np

from .core import Pose, normalize_quaternion, normalize_vector

LOGGER = logging.getLogger(__name__)


class ForceReader(Protocol):
    def read(self) -> tuple[float, ...]: ...

    def zero(self) -> None: ...

    def close(self) -> None: ...


class RobotAdapter(Protocol):
    def connect(self) -> None: ...

    def read_pose(self) -> Pose: ...

    def move_to_pose(self, pose: Pose) -> bool: ...

    def start_pose_servo(self) -> bool: ...

    def drag_to_pose(self, prompt: str) -> Pose: ...

    def send_pose(self, pose: Pose) -> bool: ...

    def stop(self) -> None: ...

    def close(self) -> None: ...


class LFS6D65Reader:
    """Minimal non-GUI reader for the supplied LFS-6D65 protocol."""

    slave_id = 1
    baudrate = 115200
    register_start = 1
    register_count = 38
    zero_register = 2560
    zero_values = [1, 0]

    def __init__(self, port: str, timeout_s: float = 1.0):
        try:
            import minimalmodbus
            import serial
        except ImportError as exc:
            raise RuntimeError(
                "LFS-6D65 control requires minimalmodbus and pyserial; "
                "install the force_control extra"
            ) from exc

        self.port = str(port)
        self.instrument = minimalmodbus.Instrument(self.port, self.slave_id)
        serial_port = self.instrument.serial
        serial_port.baudrate = self.baudrate
        serial_port.bytesize = 8
        serial_port.parity = serial.PARITY_NONE
        serial_port.stopbits = 1
        serial_port.timeout = timeout_s
        serial_port.write_timeout = timeout_s
        self.instrument.mode = minimalmodbus.MODE_RTU
        self.instrument.clear_buffers_before_each_transaction = True
        self.lock = threading.Lock()

    def read(self) -> tuple[float, ...]:
        try:
            with self.lock:
                registers = self.instrument.read_registers(
                    self.register_start,
                    self.register_count,
                    functioncode=3,
                )
        except Exception as exc:
            raise RuntimeError(f"LFS-6D65 read failed on {self.port}: {exc}") from exc
        payload = b"".join(struct.pack(">H", value) for value in registers[:12])
        values = struct.unpack(">6f", payload)
        if not np.isfinite(values).all():
            raise RuntimeError("LFS-6D65 returned a non-finite wrench")
        return tuple(float(value) for value in values)

    def zero(self) -> None:
        try:
            with self.lock:
                self.instrument.write_registers(self.zero_register, self.zero_values)
        except Exception as exc:
            raise RuntimeError(f"LFS-6D65 zero failed on {self.port}: {exc}") from exc

    def close(self) -> None:
        serial_port = self.instrument.serial
        if serial_port.is_open:
            serial_port.close()


@dataclass(frozen=True)
class WrenchSample:
    t_mono: float
    values: np.ndarray

    def __post_init__(self):
        values = normalize_vector(self.values, 6, "wrench")
        object.__setattr__(self, "values", values)


class ForceSampler:
    """Read the blocking Modbus device in a worker and expose the latest sample."""

    def __init__(self, reader: ForceReader, poll_interval_s: float = 0.02):
        self.reader = reader
        self.poll_interval_s = max(0.0, float(poll_interval_s))
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest_sample: WrenchSample | None = None
        self.error: BaseException | None = None
        self.thread: threading.Thread | None = None

    def start(self) -> None:
        if self.thread is not None:
            raise RuntimeError("Force sampler already started")
        self.stop_event.clear()
        self.thread = threading.Thread(
            target=self._run,
            name="lfs6d65-force-reader",
            daemon=True,
        )
        self.thread.start()

    def _run(self) -> None:
        while not self.stop_event.is_set():
            started = time.monotonic()
            try:
                values = np.asarray(self.reader.read())
                # Timestamp after the blocking Modbus transaction so stale-data
                # detection measures the age of the received sample, rather than
                # the time at which the request started.
                sample = WrenchSample(time.monotonic(), values)
                with self.lock:
                    self.latest_sample = sample
                    self.error = None
            except BaseException as exc:  # preserve the hardware error for the controller
                with self.lock:
                    self.error = exc
                LOGGER.warning("LFS-6D65 read failed: %s", exc)
            delay = self.poll_interval_s - (time.monotonic() - started)
            if delay > 0:
                self.stop_event.wait(delay)

    def latest(self, max_age_s: float) -> WrenchSample:
        with self.lock:
            sample = self.latest_sample
            error = self.error
        if sample is None:
            raise RuntimeError(f"No LFS-6D65 sample is available: {error or 'waiting'}")
        age = time.monotonic() - sample.t_mono
        if age > max_age_s:
            raise RuntimeError(f"LFS-6D65 sample is stale ({age:.3f}s)")
        return sample

    def wait_for_first(self, timeout_s: float) -> WrenchSample:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self.lock:
                sample = self.latest_sample
                error = self.error
            if sample is not None:
                return sample
            if error is not None:
                raise RuntimeError(f"LFS-6D65 read failed: {error}")
            time.sleep(0.001)
        raise RuntimeError("Timed out waiting for the first LFS-6D65 sample")

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
            self.thread = None


def collect_bias(
    reader: ForceReader,
    duration_s: float = 2.0,
    poll_interval_s: float = 0.02,
    minimum_samples: int = 10,
) -> np.ndarray:
    """Collect an unloaded software bias after the sensor hardware zero."""

    if duration_s <= 0:
        raise ValueError("Bias duration must be positive")
    samples: list[np.ndarray] = []
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline or len(samples) < minimum_samples:
        values = normalize_vector(reader.read(), 6, "wrench")
        samples.append(values)
        if time.monotonic() >= deadline and len(samples) >= minimum_samples:
            break
        time.sleep(poll_interval_s)
    return np.mean(np.stack(samples), axis=0)


class AirbotPlayAdapter:
    """AIRBOT SDK adapter using Cartesian pose servo commands."""

    def __init__(self, url: str = "localhost", port: int = 50051):
        self.url = url
        self.port = int(port)
        self.sdk = None
        self.robot_mode = None

    def connect(self) -> None:
        if self.sdk is not None:
            return
        try:
            from airbot_py.arm import AIRBOTPlay, RobotMode
        except ImportError as exc:
            raise RuntimeError(
                "AIRBOT control requires the airbot_py SDK in the active environment"
            ) from exc
        self.sdk = AIRBOTPlay(url=self.url, port=self.port)
        self.robot_mode = RobotMode
        if not self.sdk.connect():
            raise RuntimeError(f"Could not connect to AIRBOT server {self.url}:{self.port}")

    def _require_sdk(self):
        if self.sdk is None or self.robot_mode is None:
            raise RuntimeError("AIRBOT adapter is not connected")

    def read_pose(self) -> Pose:
        self._require_sdk()
        raw = self.sdk.get_end_pose()
        if raw is None or len(raw) != 2:
            raise RuntimeError("AIRBOT returned no end pose")
        position = normalize_vector(raw[0], 3, "AIRBOT position")
        orientation = normalize_quaternion(raw[1])
        return position, orientation

    def move_to_pose(self, pose: Pose) -> bool:
        self._require_sdk()
        position, orientation = pose
        if not self.sdk.switch_mode(self.robot_mode.PLANNING_POS):
            return False
        return bool(
            self.sdk.move_to_cart_pose(
                [position.tolist(), orientation.tolist()], blocking=True
            )
        )

    def start_pose_servo(self) -> bool:
        self._require_sdk()
        return bool(self.sdk.switch_mode(self.robot_mode.SERVO_CART_POSE))

    def start_gravity_comp(self) -> bool:
        self._require_sdk()
        return bool(self.sdk.switch_mode(self.robot_mode.GRAVITY_COMP))

    def stop_gravity_comp(self) -> bool:
        self._require_sdk()
        return bool(self.sdk.switch_mode(self.robot_mode.PLANNING_POS))

    def drag_to_pose(self, prompt: str) -> Pose:
        """Let the operator drag the arm, then return the confirmed pose."""

        self._require_sdk()
        if not self.start_gravity_comp():
            raise RuntimeError("Could not enter AIRBOT gravity compensation mode")
        try:
            input(prompt)
        finally:
            if not self.stop_gravity_comp():
                raise RuntimeError("Could not leave AIRBOT gravity compensation mode")
        return self.read_pose()

    def send_pose(self, pose: Pose) -> bool:
        self._require_sdk()
        position, orientation = pose
        self.sdk.servo_cart_pose([position.tolist(), orientation.tolist()])
        return True

    def stop(self) -> None:
        if self.sdk is None or self.robot_mode is None:
            return
        try:
            self.sdk.switch_mode(self.robot_mode.PLANNING_POS)
        except Exception:
            LOGGER.exception("Failed to switch AIRBOT to planning mode during stop")

    def close(self) -> None:
        if self.sdk is not None:
            self.sdk.disconnect()
            self.sdk = None


class CsvLogger:
    """Append controller telemetry without coupling it to the UI."""

    fields = (
        "t_s",
        "state",
        "fx_n",
        "fy_n",
        "fz_raw_n",
        "normal_force_n",
        "mx_nm",
        "my_nm",
        "mz_nm",
        "bias_fz_n",
        "delta_n_m",
        "position_x_m",
        "position_y_m",
        "position_z_m",
        "command_z_m",
    )

    def __init__(self, path: str | Path | None):
        self.path = Path(path) if path else None
        self.stream = None
        self.writer = None
        if self.path:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.stream = self.path.open("w", newline="", encoding="utf-8")
            self.writer = csv.DictWriter(self.stream, fieldnames=self.fields)
            self.writer.writeheader()

    def write(self, row: dict) -> None:
        if self.writer is None:
            return
        self.writer.writerow({field: row.get(field, "") for field in self.fields})
        self.stream.flush()

    def close(self) -> None:
        if self.stream is not None:
            self.stream.close()
            self.stream = None
