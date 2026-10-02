"""KWR75 (坤维) serial force/torque sensor adapter."""

from __future__ import annotations

import struct
import threading
import time

import numpy as np


class KunweiKwr75Reader:
    """Read KWR75 binary frames and expose SI wrench values.

    Frames are ``48 AA`` + six little-endian float32 values in kgf/kgf.m
    + ``0D 0A``.  The device is streamed at 460800 baud after the start
    command; tare is performed locally because this protocol has no zero
    command.
    """

    START = b"\x48\xaa\x0d\x0a"
    STOP = b"\x43\xaa\x0d\x0a"
    FRAME_SIZE = 28
    HEADER = b"\x48\xaa"
    TAIL = b"\x0d\x0a"

    def __init__(self, port: str, timeout_s: float = 1.0, baudrate: int = 460800,
                 tare_duration_s: float = 1.0, stale_s: float = 0.2):
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("Kunwei KWR75 requires pyserial") from exc
        if baudrate <= 0 or timeout_s <= 0 or tare_duration_s <= 0 or stale_s <= 0:
            raise ValueError("Kunwei serial and timing values must be positive")
        self.port, self.timeout_s = str(port), float(timeout_s)
        self.baudrate, self.tare_duration_s, self.stale_s = int(baudrate), float(tare_duration_s), float(stale_s)
        self._serial_cls = serial.Serial
        self.serial = None
        self._thread = None
        self._stop = threading.Event()
        self._condition = threading.Condition()
        self._latest = None
        self._latest_time = 0.0
        self._sequence = 0
        self._bias = np.zeros(6, dtype=float)
        self._error = None

    @staticmethod
    def decode_frame(frame: bytes) -> tuple[float, ...]:
        if len(frame) != 28 or frame[:2] != KunweiKwr75Reader.HEADER or frame[-2:] != KunweiKwr75Reader.TAIL:
            raise ValueError("invalid KWR75 frame")
        raw = struct.unpack("<6f", frame[2:26])
        values = np.asarray(raw, dtype=float) * 9.81
        if not np.isfinite(values).all():
            raise ValueError("KWR75 returned non-finite values")
        return tuple(float(x) for x in values)

    def start(self) -> None:
        if self._thread is not None:
            return
        self.serial = self._serial_cls(self.port, baudrate=self.baudrate, bytesize=8,
                                       parity="N", stopbits=1, timeout=0.1, write_timeout=self.timeout_s)
        self.serial.write(self.START)
        self._stop.clear()
        self._thread = threading.Thread(target=self._reader_loop, name="kunwei-kwr75-reader", daemon=True)
        self._thread.start()

    def _reader_loop(self) -> None:
        buffer = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self.serial.read(256)
                if not chunk:
                    continue
                buffer.extend(chunk)
                while True:
                    index = buffer.find(self.HEADER)
                    if index < 0:
                        buffer.clear()
                        break
                    if index:
                        del buffer[:index]
                    if len(buffer) < self.FRAME_SIZE:
                        break
                    frame = bytes(buffer[:self.FRAME_SIZE])
                    if frame[-2:] != self.TAIL:
                        del buffer[:2]
                        continue
                    del buffer[:self.FRAME_SIZE]
                    values = np.asarray(self.decode_frame(frame))
                    with self._condition:
                        self._latest = values
                        self._latest_time = time.monotonic()
                        self._sequence += 1
                        self._error = None
                        self._condition.notify_all()
            except BaseException as exc:
                with self._condition:
                    self._error = exc
                    self._condition.notify_all()
                return

    def read(self) -> tuple[float, ...]:
        self.start()
        with self._condition:
            previous = self._sequence
            deadline = time.monotonic() + self.timeout_s
            while self._sequence == previous:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._condition.wait(remaining)
            if self._sequence == previous:
                raise RuntimeError(f"Kunwei KWR75 read timed out on {self.port}: {self._error or 'no frame'}")
            values = self._latest.copy()
            age = time.monotonic() - self._latest_time
        if age > self.stale_s:
            raise RuntimeError(f"Kunwei KWR75 sample is stale ({age:.3f}s)")
        return tuple((values - self._bias).tolist())

    def latest_raw(self):
        with self._condition:
            if self._latest is None:
                return None
            return self._latest.copy()

    def zero(self) -> None:
        self.start()
        samples = []
        deadline = time.monotonic() + self.tare_duration_s
        hard_deadline = deadline + self.timeout_s
        while time.monotonic() < hard_deadline and (time.monotonic() < deadline or len(samples) < 10):
            try:
                self.read()
                raw = self.latest_raw()
                if raw is not None:
                    samples.append(raw)
            except RuntimeError:
                if time.monotonic() >= deadline and samples:
                    break
        if len(samples) < 1:
            raise RuntimeError(f"Kunwei KWR75 tare failed on {self.port}: no samples")
        self._bias = np.mean(np.stack(samples), axis=0)

    def close(self) -> None:
        self._stop.set()
        if self.serial is not None:
            try:
                self.serial.write(self.STOP)
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self.serial is not None and self.serial.is_open:
            self.serial.close()
        self._thread = None
        self.serial = None
