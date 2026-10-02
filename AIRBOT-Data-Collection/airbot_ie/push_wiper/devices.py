"""Hardware adapters. Reads never switch robot mode or send commands."""

import fcntl
import hashlib
import logging
import os
import time
from contextlib import ExitStack
from pathlib import Path
from threading import Event, Lock, Thread

import numpy as np

from .geometry import vector
from .clock import acquisition_ns


def lock_robot_endpoints(endpoints):
    """One controller per endpoint, including the legacy follow script."""
    stack = ExitStack()
    try:
        for endpoint in sorted(set(endpoints)):
            digest = hashlib.sha256(endpoint.encode()).hexdigest()[:24]
            path = Path(f"/tmp/airbot-control-{os.getuid()}-{digest}.lock")
            stream = stack.enter_context(path.open("a+"))
            try:
                fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError(
                    f"Another controller owns {endpoint}; stop it first"
                ) from exc
        return stack
    except BaseException:
        stack.close()
        raise


class SDKArm:
    def __init__(self, url, port, feedback_timeout_s=0.5):
        from airbot_py.arm import AIRBOTPlay, RobotMode, SpeedProfile

        self.sdk = AIRBOTPlay(url=url, port=port)
        self.modes = {
            "planning": RobotMode.PLANNING_POS,
            "servo": RobotMode.SERVO_JOINT_POS,
            "gravity": RobotMode.GRAVITY_COMP,
        }
        self.slow_profile = SpeedProfile.SLOW
        self.feedback_timeout_s = feedback_timeout_s
        self.last_feedback = None
        self.feedback_changed = time.monotonic()
        self.info = {}

    def connect(self):
        if not self.sdk.connect():
            raise RuntimeError("Cannot connect to AIRBOT server")
        # SDK metadata contains protobuf repeated containers, not JSON lists.
        self.info = {
            key: value
            if isinstance(value, (str, bool, int, float, type(None)))
            else list(value)
            for key, value in self.sdk.get_product_info().items()
        }
        self.feedback_changed = time.monotonic()

    def read(self):
        # AIRBOT SDK 5.1.6 caches asynchronous feedback. Polling that cache must
        # not make disconnected/stalled feedback appear fresh. Track message
        # replacement, not joint values (stationary arms still receive messages).
        feedback = self.sdk._feedback_jointstates
        if feedback is None or not self.sdk._feedbacking:
            raise RuntimeError("AIRBOT feedback disconnected")
        now = time.monotonic()
        if feedback is not self.last_feedback:
            self.last_feedback = feedback
            self.feedback_changed = now
        if now - self.feedback_changed > self.feedback_timeout_s:
            raise RuntimeError("AIRBOT feedback stream is stale")
        joints = self.sdk.get_joint_pos()
        velocity = self.sdk.get_joint_vel()
        pose = self.sdk.get_end_pose()
        effort = self.sdk.get_joint_eff()
        if pose is None:
            raise RuntimeError("AIRBOT pose unavailable")
        result = {
            "joints": joints,
            "velocity": velocity,
            "position": pose[0],
            "orientation": pose[1],
            "t_ns": acquisition_ns(),
            "feedback_age_s": now - self.feedback_changed,
        }
        if (
            effort is not None
            and np.asarray(effort).shape == (6,)
            and np.isfinite(effort).all()
        ):
            result["effort"] = list(effort)
        return result

    def set_mode(self, mode):
        return self.sdk.switch_mode(self.modes[mode])

    def slow(self):
        self.sdk.set_speed_profile(self.slow_profile)

    def move(self, target):
        return self.sdk.move_to_joint_pos(list(target), blocking=False)

    def servo(self, target):
        result = self.sdk.servo_joint_pos(list(target))
        if result is False:
            raise RuntimeError("Joint servo rejected")

    def close(self):
        self.sdk.disconnect()


class CameraWorker:
    """Own the existing blocking driver in one thread, publishing latest BGR."""

    def __init__(self, config):
        self.config = config
        self.stop = Event()
        self.ready = Event()
        self.lock = Lock()
        self.frame = None
        self.error = None
        self.info = {}
        self.thread = Thread(target=self._run, name="push-wiper-camera", daemon=True)

    def start(self):
        self.thread.start()
        if not self.ready.wait(10):
            raise RuntimeError(
                f"Camera {self.config.serial} startup timed out; check USB connection "
                "and close other camera applications"
            )
        self.latest()

    def _run(self):
        camera = None
        try:
            from airdc.common.devices.cameras.intelrealsense import IntelRealSenseCamera

            camera = IntelRealSenseCamera(
                camera_index=self.config.serial,
                width=self.config.width,
                height=self.config.height,
                fps=self.config.fps,
                enable_color=True,
                enable_depth=False,
                align_depth=False,
                blocking=True,
                rgb_camera={"pixel_format": "bgr8"},
            )
            if not camera.configure():
                raise RuntimeError("RealSense configuration failed")
            self.info = camera.get_info()
            while not self.stop.is_set():
                frame = camera.capture_observation(timeout=1.0)["color/image_raw"]
                self._publish_frame(frame)
        except Exception as exc:
            logging.getLogger(__name__).debug(
                "Camera acquisition failed", exc_info=True
            )
            self.error = str(exc)
            self.ready.set()
        finally:
            if camera is not None:
                try:
                    camera.shutdown()
                except Exception:
                    logging.getLogger(__name__).warning(
                        "Camera cleanup failed", exc_info=True
                    )

    def _publish_frame(self, frame):
        # Called only after blocking capture returns a newly acquired frame.
        # The camera driver's wall-clock stamp is not on our monotonic timeline.
        stamp = acquisition_ns()
        with self.lock:
            self.frame = {"data": frame["data"].copy(), "t": stamp}
        self.ready.set()

    def latest(self):
        if self.error:
            raise RuntimeError(f"Camera failed: {self.error}")
        with self.lock:
            if self.frame is None:
                raise RuntimeError("No camera frame yet")
            age_s = (acquisition_ns() - self.frame["t"]) / 1e9
            if age_s > self.config.stale_s:
                raise RuntimeError(
                    f"Camera {self.config.serial} frame is stale "
                    f"(age={age_s:.3f}s, limit={self.config.stale_s:.3f}s); "
                    "check USB connection and run the standalone camera preview"
                )
            return {"data": self.frame["data"].copy(), "t": self.frame["t"]}

    def wait_for_frame_after(self, timestamp_ns: int, timeout_s: float = 5.0):
        """Return a fresh frame acquired after ``timestamp_ns``.

        ``latest()`` is intentionally allowed to return the most recent frame,
        which is useful for continuous processing but can still be a frame
        captured while the robot was moving to the observation pose.  The
        online policy loop uses this method after the configured settle delay
        so each prediction is based on a post-motion image.
        """

        if not isinstance(timestamp_ns, (int, np.integer)):
            raise ValueError("timestamp_ns must be an integer nanosecond timestamp")
        timeout_s = float(timeout_s)
        if timeout_s <= 0 or not np.isfinite(timeout_s):
            raise ValueError("timeout_s must be positive and finite")
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.error:
                raise RuntimeError(f"Camera failed: {self.error}")
            with self.lock:
                frame = None if self.frame is None else {
                    "data": self.frame["data"].copy(),
                    "t": self.frame["t"],
                }
            if frame is not None and int(frame["t"]) > int(timestamp_ns):
                age_s = (acquisition_ns() - frame["t"]) / 1e9
                if age_s <= self.config.stale_s:
                    return frame
            time.sleep(0.001)
        if self.error:
            raise RuntimeError(f"Camera failed: {self.error}")
        raise TimeoutError(
            f"Timed out waiting for a camera frame newer than {timestamp_ns}"
        )

    def close(self):
        self.stop.set()
        self.thread.join(timeout=3)


class MockArm:
    """No SDK import or hardware connection; motion is intentionally synthetic."""

    def __init__(self):
        self.joints = np.zeros(6)
        self.target = None
        self.mode = "planning"
        self.commands = []
        self.fail_read = False
        self.reject_move = False
        self.stuck = False
        self.info = {"product_type": "MOCK", "eef_types": []}

    def connect(self):
        pass

    def read(self):
        if self.fail_read:
            raise RuntimeError("Simulated disconnect")
        if self.target is not None and not self.stuck:
            self.joints = np.asarray(self.target, dtype=float).copy()
            self.target = None
        angle = self.joints[5]
        return {
            "joints": self.joints.tolist(),
            "velocity": [0.0] * 6,
            "position": [0.3 + self.joints[0], self.joints[1], 0.3 + self.joints[2]],
            "orientation": [
                0.0,
                0.0,
                float(np.sin(angle / 2)),
                float(np.cos(angle / 2)),
            ],
            "t_ns": acquisition_ns(),
            "feedback_age_s": 0.0,
        }

    def set_mode(self, mode):
        self.commands.append(("mode", mode))
        self.mode = mode
        self.target = None
        return True

    def slow(self):
        self.commands.append(("slow",))

    def move(self, target):
        self.commands.append(("move", list(target)))
        if self.reject_move:
            return False
        self.target = list(target)
        return True

    def servo(self, target):
        self.commands.append(("servo", list(target)))
        self.target = list(target)

    def close(self):
        pass
