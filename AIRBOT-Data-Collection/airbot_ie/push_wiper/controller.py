"""Single motion owner. The core is deterministic and testable without an SDK."""

import logging
import time
from concurrent.futures import Future
from copy import deepcopy
from queue import Empty, Queue
from threading import Event, Lock, Thread

import numpy as np

from .config import WorkHeightReference
from .clock import acquisition_ns
from .geometry import pose_error, quaternion, vector


class CommandError(ValueError):
    """Rejected operator action, not a hardware failure."""


class ArmController:
    """Shared registration, reset, feedback and hold for the connected arms."""

    def __init__(self, arms, config, observation=None):
        self.arms = arms
        self.config = config
        self.observation = deepcopy(observation)
        self.state = "idle"
        self.error = ""
        self.samples = {}
        self.events = []
        self.done_since = None
        self.deadline = None
        self.hold_targets = {}
        self.hold_errors = {}
        self.follow_target = None
        self.follow_time = None
        self.cancel_event = Event()

    def event(self, name, **details):
        self.events.append({"name": name, "t_ns": acquisition_ns(), **details})
        self.events = self.events[-200:]

    def refresh(self, now):
        samples = {}
        for name, arm in self.arms.items():
            sample = arm.read()
            for field in ("joints", "velocity"):
                sample[field] = vector(sample[field], 6).tolist()
            sample["position"] = vector(sample["position"], 3).tolist()
            sample["orientation"] = quaternion(sample["orientation"]).tolist()
            samples[name] = sample
        self.samples = samples

    def errors(self):
        if not self.observation or not self.samples:
            return {}
        result = {}
        for name, sample in self.samples.items():
            reference = self.observation[name]
            position, angle = pose_error(sample, reference)
            result[name] = {
                "position_m": position,
                "angle_deg": angle,
                "joint_rad": float(
                    np.max(np.abs(np.asarray(sample["joints"]) - reference["joints"]))
                ),
            }
        return result

    def within(self):
        errors = self.errors()
        return bool(errors) and all(
            value["position_m"] <= self.config.done_position_m
            and value["angle_deg"] <= self.config.done_angle_deg
            and value["joint_rad"] <= self.config.done_joint_rad
            for value in errors.values()
        )

    def observation_error_text(self):
        return "; ".join(
            f"{name}: position={value['position_m'] * 1000:.3f}mm "
            f"(limit {self.config.done_position_m * 1000:.3f}mm), "
            f"angle={value['angle_deg']:.3f}deg "
            f"(limit {self.config.done_angle_deg:.3f}deg), "
            f"joint={value['joint_rad']:.5f}rad "
            f"(limit {self.config.done_joint_rad:.5f}rad)"
            for name, value in self.errors().items()
        )

    def _check_cancelled(self):
        if self.cancel_event.is_set() and self.state != "fault":
            raise RuntimeError("Controller stopping; motion command cancelled")

    def _mode(self, name, mode):
        self._check_cancelled()
        if not self.arms[name].set_mode(mode):
            raise RuntimeError(f"{name}: could not enter {mode}")
        self._check_cancelled()

    def register_observation_pose(self, now):
        if self.state not in {"idle", "following", "observing"}:
            raise CommandError("Observation registration is unavailable in this state")
        if not self.samples:
            raise CommandError("Wait for robot feedback before registering")
        # Preserve the actual camera pose even when the two arms differ.
        # Each arm returns to its own measured joints, not the other arm's.
        reference = deepcopy(self.samples)
        self.observation = reference
        self.reset_to_observation(now)
        return deepcopy(reference)

    def simulate_lead(self, now, delta):
        arm = self.arms.get("lead", self.arms["follow"])
        if arm.info.get("product_type") != "MOCK" or self.state != "following":
            raise CommandError(
                "Synthetic movement requires mock mode and G"
            )
        arm.joints += vector(delta, 6)

    def register_work_height(self, now):
        """Record the follower's SDK Z only; never command or detect contact."""
        if self.state not in {"idle", "following", "observing"}:
            raise CommandError("Work height registration is unavailable in this state")
        if not self.samples:
            raise CommandError("Wait for robot feedback before registering height")
        sample = self.samples["follow"]
        return WorkHeightReference(
            z_m=vector(sample["position"], 3)[2], t_ns=sample["t_ns"]
        ).model_dump(mode="json")

    def reset_to_observation(self, now):
        if self.state not in {"idle", "following", "observing", "settling"}:
            raise CommandError("Reset is unavailable in the current controller state")
        if not self.observation:
            raise CommandError("Register observation pose with O before reset")
        if not self.samples:
            raise CommandError("Wait for robot feedback before reset")
        # Set state BEFORE switching modes: no further follow command can escape.
        self.state = "resetting"
        self.done_since = None
        self.deadline = now + self.config.timeout_s
        self.event("reset_started", errors=self.errors())
        try:
            for name, arm in self.arms.items():
                self._mode(name, "planning")
                arm.slow()
            for name, arm in self.arms.items():
                self._check_cancelled()
                if not arm.move(self.observation[name]["joints"]):
                    raise RuntimeError(f"{name}: reset motion rejected")
        except Exception as exc:
            self.fault(str(exc))
            raise

    def resume_follow(self, now):
        if self.state not in {"idle", "observing", "settling"}:
            raise CommandError("Motion can only resume from idle or observation hold")
        if not self.samples:
            raise CommandError("Wait for robot feedback before enabling motion")
        try:
            self._resume_motion(now)
            self._check_cancelled()
            self.state = "following"
            self.event(self.resume_event)
        except Exception as exc:
            self.fault(str(exc))
            raise

    def fault(self, reason):
        self.state = "fault"
        self.error = reason
        self.hold_targets = {}
        self.hold_errors = {}
        # Attempt every connected arm independently, even after a disconnect.
        for name, arm in self.arms.items():
            try:
                target = vector(arm.read()["joints"], 6).tolist()
                self._mode(name, "servo")
                arm.servo(target)
                self.hold_targets[name] = target
            except Exception as exc:
                logging.getLogger(__name__).debug("Cannot hold %s", name, exc_info=True)
                self.hold_errors[name] = str(exc)
        self.event("fault", reason=reason, hold_errors=self.hold_errors.copy())

    def tick(self, now=None):
        now = time.monotonic() if now is None else now
        if self.state == "fault":
            for name, target in list(self.hold_targets.items()):
                try:
                    self.arms[name].servo(target)
                except Exception as exc:
                    logging.getLogger(__name__).debug(
                        "Hold connection lost: %s", name, exc_info=True
                    )
                    self.hold_errors[name] = str(exc)
                    del self.hold_targets[name]
            return
        try:
            self.refresh(now)
            if self.state == "following":
                self._motion_tick(now)
            elif self.state in {"resetting", "settling"}:
                if now >= self.deadline:
                    reason = (
                        "Observation reset timed out"
                        if self.state == "resetting"
                        else "Observation hold drifted out of tolerance; recovery timed out"
                    )
                    raise RuntimeError(f"{reason}: {self.observation_error_text()}")
                if self.within():
                    self.done_since = (
                        now if self.done_since is None else self.done_since
                    )
                    if now - self.done_since >= self.config.stable_s:
                        event = (
                            "reset_completed"
                            if self.state == "resetting"
                            else "observation_recovered"
                        )
                        self.state = "observing"
                        self.event(event, errors=self.errors())
                else:
                    self.done_since = None
            elif self.state == "observing" and not self.within():
                # Withdraw capture readiness immediately, but let a transient
                # feedback/hold fluctuation settle without ending the episode.
                # Do not send a new motion command or change the registered O.
                self.state = "settling"
                self.done_since = None
                self.deadline = now + self.config.timeout_s
                self.event("observation_unstable", errors=self.errors())
                logging.getLogger(__name__).warning(
                    "Observation hold outside tolerance; capture disabled while "
                    "waiting for stability: %s", self.observation_error_text()
                )
        except Exception as exc:
            logging.getLogger(__name__).debug("Control tick failed", exc_info=True)
            self.fault(str(exc))

    def get_status(self):
        return deepcopy(
            {
                "state": self.state,
                "error": self.error,
                "samples": self.samples,
                "errors": self.errors(),
                "events": self.events,
                "hold_errors": self.hold_errors,
                "published_ns": acquisition_ns(),
            }
        )


class DualArmController(ArmController):
    resume_event = "follow_resumed"

    def __init__(self, lead, follow, config, observation=None):
        super().__init__({"lead": lead, "follow": follow}, config, observation)

    def _resume_motion(self, now):
        # Prime follower at its current position before releasing the leader.
        self._mode("follow", "servo")
        self.arms["follow"].servo(self.samples["follow"]["joints"])
        self.follow_target = np.asarray(self.samples["follow"]["joints"]).copy()
        self.follow_time = now
        self._mode("lead", "gravity")

    def _motion_tick(self, now):
        # Bound command slew, not the permissible lead/follow error.
        # Cap dt so a delayed iteration cannot issue a large jump.
        dt = min(max(now - self.follow_time, 0.0), 0.05)
        step = self.config.follow_joint_speed_rad_s * dt
        desired = np.asarray(self.samples["lead"]["joints"])
        self.follow_target += np.clip(desired - self.follow_target, -step, step)
        self.follow_time = now
        self.arms["follow"].servo(self.follow_target.tolist())


class SingleArmDragController(ArmController):
    """Record the tool arm in gravity compensation without position commands.

    Keep the existing `following` state for the collection FSM and raw schema.
    """

    resume_event = "drag_resumed"

    def __init__(self, follow, config, observation=None):
        super().__init__({"follow": follow}, config, observation)

    def _resume_motion(self, now):
        self._mode("follow", "gravity")

    def _motion_tick(self, now):
        # Feedback was refreshed by tick; the operator supplies all movement.
        pass


class ControllerRunner:
    """Only this thread calls motion APIs; UI/recording consume copied snapshots."""

    def __init__(self, controller, hz=100):
        self.controller = controller
        self.period = 1 / hz
        self.queue = Queue()
        self.lock = Lock()
        self.status = controller.get_status()
        self.status["active_command"] = None
        self.stop_event = Event()
        self.stop_reason = "Collector stopped"
        self.thread = Thread(target=self._run, name="push-wiper-control", daemon=True)

    def start(self):
        self.thread.start()

    def submit(self, command, **kwargs):
        future = Future()
        self.queue.put((command, kwargs, future))
        return future

    def snapshot(self):
        with self.lock:
            return deepcopy(self.status)

    def _publish(self):
        status = self.controller.get_status()
        status["active_command"] = None
        with self.lock:
            self.status = status

    def _begin_command(self, command):
        with self.lock:
            # Do not renew published_ns or feedback timestamps: these samples
            # predate the command and must not be recorded as current feedback.
            self.status["active_command"] = {
                "name": command, "started_ns": acquisition_ns()
            }

    def _run(self):
        try:
            while not self.stop_event.is_set():
                start = time.monotonic()
                self.controller.tick(start)
                if self.stop_event.is_set():
                    break
                self._publish()
                try:
                    command, kwargs, future = self.queue.get_nowait()
                except Empty:
                    pass
                else:
                    self._begin_command(command)
                    try:
                        method = getattr(self.controller, command)
                        if command == "fault":
                            result = method(**kwargs)
                        else:
                            result = method(now=time.monotonic(), **kwargs)
                        # Publish post-command feedback before resolving the
                        # future, so the UI never reuses the pre-command state.
                        if not self.stop_event.is_set():
                            self.controller.tick(time.monotonic())
                        self._publish()
                        future.set_result(result)
                    except Exception as exc:
                        logging.getLogger(__name__).debug(
                            "Controller command failed", exc_info=True
                        )
                        if (
                            not isinstance(exc, CommandError)
                            and self.controller.state != "fault"
                        ):
                            self.controller.fault(str(exc))
                        self._publish()
                        future.set_exception(exc)
                self.stop_event.wait(max(0, self.period - (time.monotonic() - start)))
        finally:
            # Graceful exit also stops follow/reset before disconnecting.
            self.controller.fault(self.stop_reason)
            while True:
                try:
                    _, _, future = self.queue.get_nowait()
                    future.set_exception(RuntimeError("Controller stopped"))
                except Empty:
                    break

    def request_stop(self, reason="Collector stopped"):
        self.stop_reason = reason
        self.controller.cancel_event.set()
        self.stop_event.set()

    def close(self):
        self.request_stop(self.stop_reason)
        self.thread.join(timeout=3)
        if self.thread.is_alive():
            raise RuntimeError("Control thread did not stop; check hardware state")
