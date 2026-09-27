"""State machine for standalone and trajectory-backed force-position control."""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from .core import (
    AdmittanceConfig,
    AdmittanceController,
    FixedXYTrajectory,
    PoseTrajectory,
    TrajectoryPoint,
    normal_force_from_fz,
    normalize_quaternion,
    normalize_vector,
)
from .hardware import CsvLogger, ForceReader, ForceSampler, RobotAdapter, collect_bias

LOGGER = logging.getLogger(__name__)


@dataclass
class RunnerConfig:
    control_hz: float = 100.0
    target_force_n: float = 5.0
    force_sign: float = -1.0
    filter_cutoff_hz: float = 2.0
    max_force_n: float = 8.0
    sensor_stale_s: float = 0.2
    bias_duration_s: float = 2.0
    zero_settle_s: float = 0.1
    sensor_poll_interval_s: float = 0.02
    contact_threshold_n: float = 0.5
    approach_timeout_s: float = 10.0
    approach_speed_m_s: float = 0.002
    approach_depth_m: float = 0.05
    settle_s: float = 0.5
    safe_clearance_m: float = 0.02
    retract_s: float = 1.0
    hold_duration_s: float = 0.0
    max_tangential_step_m: float = 0.01
    max_tangential_speed_m_s: float = 0.05
    log_path: str | None = "data/force_hybrid_push.csv"
    admittance: AdmittanceConfig = field(default_factory=AdmittanceConfig)

    def __post_init__(self):
        positive = (
            ("control_hz", self.control_hz),
            ("filter_cutoff_hz", self.filter_cutoff_hz),
            ("target_force_n", self.target_force_n),
            ("max_force_n", self.max_force_n),
            ("sensor_stale_s", self.sensor_stale_s),
            ("bias_duration_s", self.bias_duration_s),
            ("zero_settle_s", self.zero_settle_s),
            ("contact_threshold_n", self.contact_threshold_n),
            ("approach_timeout_s", self.approach_timeout_s),
            ("approach_speed_m_s", self.approach_speed_m_s),
            ("approach_depth_m", self.approach_depth_m),
            ("settle_s", self.settle_s),
            ("safe_clearance_m", self.safe_clearance_m),
            ("retract_s", self.retract_s),
        )
        for name, value in positive:
            if value <= 0:
                raise ValueError(f"{name} must be positive")
        if self.force_sign not in (-1, 1):
            raise ValueError("force_sign must be either -1 or 1")
        if self.target_force_n >= self.max_force_n:
            raise ValueError("target_force_n must be below max_force_n")
        if self.contact_threshold_n >= self.max_force_n:
            raise ValueError("contact_threshold_n must be below max_force_n")
        if self.hold_duration_s < 0:
            raise ValueError("hold_duration_s must be non-negative")
        if self.max_tangential_step_m <= 0:
            raise ValueError("max_tangential_step_m must be positive")
        if self.max_tangential_speed_m_s <= 0:
            raise ValueError("max_tangential_speed_m_s must be positive")


def _first_order_alpha(cutoff_hz: float, dt_s: float) -> float:
    return float(1.0 - np.exp(-2.0 * np.pi * cutoff_hz * max(dt_s, 1e-6)))


class ForceFilter:
    def __init__(self, cutoff_hz: float):
        self.cutoff_hz = cutoff_hz
        self.value: np.ndarray | None = None
        self.timestamp: float | None = None

    def update(self, values: np.ndarray, timestamp: float) -> np.ndarray:
        values = normalize_vector(values, 6, "wrench")
        if self.value is None or self.timestamp is None:
            self.value = values.copy()
        elif timestamp > self.timestamp:
            alpha = _first_order_alpha(self.cutoff_hz, timestamp - self.timestamp)
            self.value = self.value + alpha * (values - self.value)
        self.timestamp = max(self.timestamp or timestamp, timestamp)
        return self.value.copy()


class HybridRunner:
    """Run approach, contact settling, force hold, and safe retract."""

    STATES = (
        "PRECHECK",
        "MOVE_SAFE",
        "APPROACH",
        "CONTACT_SETTLE",
        "FORCE_HOLD",
        "RETRACT",
        "FAULT",
        "DONE",
    )

    def __init__(
        self,
        robot: RobotAdapter,
        reader: ForceReader,
        trajectory: PoseTrajectory,
        config: RunnerConfig,
        logger: CsvLogger | None = None,
    ):
        self.robot = robot
        self.reader = reader
        self.trajectory = trajectory
        self.config = config
        self.logger = logger or CsvLogger(config.log_path)
        self.state = "PRECHECK"
        self.error = ""
        self.bias = np.zeros(6, dtype=float)
        self.sampler: ForceSampler | None = None
        self.filter = ForceFilter(config.filter_cutoff_hz)
        self.admittance = AdmittanceController(config.admittance)
        self.command_position: np.ndarray | None = None
        self.last_loop_time: float | None = None
        self.started_time: float | None = None

    def _set_state(self, state: str) -> None:
        if state not in self.STATES:
            raise ValueError(f"Unknown controller state: {state}")
        self.state = state
        LOGGER.info("Force controller state: %s", state)

    def _read_force(self) -> tuple[np.ndarray, float, float]:
        if self.sampler is None:
            raise RuntimeError("Force sampler is not running")
        sample = self.sampler.latest(self.config.sensor_stale_s)
        filtered = self.filter.update(sample.values, sample.t_mono)
        normal_force = normal_force_from_fz(
            filtered[2], self.bias[2], self.config.force_sign
        )
        return filtered, normal_force, sample.t_mono

    def _send(self, point: TrajectoryPoint, current_position: np.ndarray, delta_n: float):
        """Track the reference pose with a total inward normal displacement."""

        normal = point.normal_base
        tangential_error = point.position_base - current_position
        normal_error = float(np.dot(tangential_error, normal))
        tangential_error = tangential_error - normal * normal_error
        tangent_norm = float(np.linalg.norm(tangential_error))
        max_tangent_step = min(
            self.config.max_tangential_step_m,
            self.config.max_tangential_speed_m_s / self.config.control_hz,
        )
        if tangent_norm > max_tangent_step:
            tangential_error *= max_tangent_step / tangent_norm
        # delta_n is already integrated by the admittance controller.  Anchor
        # it to the trajectory reference, not to the measured pose each tick.
        command_position = (
            current_position + tangential_error + normal * (normal_error - delta_n)
        )
        self.command_position = command_position
        if not self.robot.send_pose((command_position, point.orientation_xyzw)):
            raise RuntimeError("AIRBOT rejected a Cartesian pose command")
        return command_position

    def _send_approach(
        self, point: TrajectoryPoint, current_position: np.ndarray
    ) -> np.ndarray:
        """Send the pre-contact pose, including its commanded normal motion.

        During approach the desired normal position follows the slow descent
        trajectory directly.  No contact-relative admittance offset is applied.
        """

        normal = point.normal_base
        tangential_error = point.position_base - current_position
        normal_error = float(np.dot(tangential_error, normal))
        tangential_error = tangential_error - normal * normal_error
        tangent_norm = float(np.linalg.norm(tangential_error))
        max_tangent_step = min(
            self.config.max_tangential_step_m,
            self.config.max_tangential_speed_m_s / self.config.control_hz,
        )
        if tangent_norm > max_tangent_step:
            tangential_error *= max_tangent_step / tangent_norm
        command_position = current_position + tangential_error + normal * normal_error
        self.command_position = command_position
        if not self.robot.send_pose((command_position, point.orientation_xyzw)):
            raise RuntimeError("AIRBOT rejected a Cartesian approach command")
        return command_position

    def _log(
        self,
        t_s: float,
        wrench: np.ndarray,
        normal_force: float,
        position: np.ndarray,
    ) -> None:
        self.logger.write(
            {
                "t_s": t_s,
                "state": self.state,
                "fx_n": wrench[0] - self.bias[0],
                "fy_n": wrench[1] - self.bias[1],
                "fz_raw_n": wrench[2],
                "normal_force_n": normal_force,
                "mx_nm": wrench[3] - self.bias[3],
                "my_nm": wrench[4] - self.bias[4],
                "mz_nm": wrench[5] - self.bias[5],
                "bias_fz_n": self.bias[2],
                "delta_n_m": self.admittance.delta_m,
                "position_x_m": position[0],
                "position_y_m": position[1],
                "position_z_m": position[2],
                "command_z_m": self.command_position[2]
                if self.command_position is not None
                else "",
            }
        )

    def _retract(self, normal: np.ndarray, orientation: np.ndarray) -> None:
        self._set_state("RETRACT")
        if not self.robot.start_pose_servo():
            raise RuntimeError("Could not enter AIRBOT Cartesian pose servo mode")
        current, _ = self.robot.read_pose()
        target = current + normal * self.config.safe_clearance_m
        deadline = time.monotonic() + self.config.retract_s
        while time.monotonic() < deadline:
            fraction = 1.0 - max(0.0, (deadline - time.monotonic()) / self.config.retract_s)
            position = current + (target - current) * fraction
            if not self.robot.send_pose((position, orientation)):
                raise RuntimeError("AIRBOT rejected the retract command")
            time.sleep(1.0 / self.config.control_hz)

    def run(self) -> str:
        self.started_time = time.monotonic()
        robot_ready = False
        motion_started = False
        try:
            self.robot.connect()
            robot_ready = True
            current_position, current_orientation = self.robot.read_pose()
            self._set_state("PRECHECK")

            self.reader.zero()
            # The LFS-6D65 needs a short interval to apply the hardware zero
            # command before its holding registers answer reliably.
            time.sleep(self.config.zero_settle_s)
            self.bias = collect_bias(
                self.reader,
                duration_s=self.config.bias_duration_s,
                poll_interval_s=self.config.sensor_poll_interval_s,
            )
            if not np.isfinite(self.bias).all():
                raise RuntimeError("Force bias is not finite")

            needs_approach_reference = (
                hasattr(self.trajectory, "z_reference_m")
                and getattr(self.trajectory, "z_reference_m") is None
            ) or (
                hasattr(self.trajectory, "surface_z_m")
                and getattr(self.trajectory, "surface_z_m") is None
            )
            if needs_approach_reference:
                reference_z = current_position[2]
                if not getattr(self.trajectory, "approach_from_current", False):
                    reference_z -= self.config.approach_depth_m
                self.trajectory.set_z_reference(
                    reference_z
                )
            first = self.trajectory.sample(0.0)
            nominal_z = first.position_base[2]
            # Move to the known/reference surface height plus clearance before
            # the slow approach.  Keeping the current height when it is above
            # this point would make a 5 cm approach limit fail on a low table.
            safe_z = nominal_z + self.config.safe_clearance_m
            first_safe = TrajectoryPoint(
                position_base=np.array([first.position_base[0], first.position_base[1], safe_z]),
                orientation_xyzw=first.orientation_xyzw,
                normal_base=first.normal_base,
            )
            self._set_state("MOVE_SAFE")
            motion_started = True
            if not self.robot.move_to_pose((first_safe.position_base, first_safe.orientation_xyzw)):
                raise RuntimeError("AIRBOT rejected the safe move")
            if not self.robot.start_pose_servo():
                raise RuntimeError("Could not enter AIRBOT Cartesian pose servo mode")

            self.sampler = ForceSampler(
                self.reader, poll_interval_s=self.config.sensor_poll_interval_s
            )
            self.sampler.start()
            self.sampler.wait_for_first(self.config.sensor_stale_s)
            loop_period = 1.0 / self.config.control_hz
            next_tick = time.monotonic()
            approach_started = next_tick
            approach_start_z = safe_z
            self._set_state("APPROACH")
            while True:
                now = time.monotonic()
                wrench, normal_force, _ = self._read_force()
                current_position, current_orientation = self.robot.read_pose()
                if normal_force > self.config.max_force_n:
                    raise RuntimeError(f"Force limit exceeded during approach: {normal_force:.3f} N")
                if normal_force >= self.config.contact_threshold_n:
                    break
                if now - approach_started > self.config.approach_timeout_s:
                    raise RuntimeError("Contact approach timed out")
                if approach_start_z - current_position[2] >= self.config.approach_depth_m:
                    raise RuntimeError("Maximum approach depth reached without contact")
                point = self.trajectory.sample(0.0)
                point = TrajectoryPoint(
                    position_base=np.array(
                        [point.position_base[0], point.position_base[1],
                         approach_start_z - self.config.approach_speed_m_s * (now - approach_started)]
                    ),
                    orientation_xyzw=point.orientation_xyzw,
                    normal_base=point.normal_base,
                )
                self._send_approach(point, current_position)
                self._log(now - self.started_time, wrench, normal_force, current_position)
                next_tick = max(next_tick + loop_period, time.monotonic())
                time.sleep(max(0.0, next_tick - time.monotonic()))

            contact_position, contact_orientation = self.robot.read_pose()
            if hasattr(self.trajectory, "set_z_reference"):
                self.trajectory.set_z_reference(float(contact_position[2]))
            self.admittance.reset()
            self._set_state("CONTACT_SETTLE")
            settle_deadline = time.monotonic() + self.config.settle_s
            while time.monotonic() < settle_deadline:
                now = time.monotonic()
                wrench, normal_force, _ = self._read_force()
                current_position, _ = self.robot.read_pose()
                if normal_force > self.config.max_force_n:
                    raise RuntimeError(f"Force limit exceeded during settling: {normal_force:.3f} N")
                point = self.trajectory.sample(0.0)
                self._send(point, current_position, 0.0)
                self._log(now - self.started_time, wrench, normal_force, current_position)
                time.sleep(loop_period)

            self._set_state("FORCE_HOLD")
            hold_started = time.monotonic()
            # Settling has its own timing; do not catch up its elapsed ticks.
            next_tick = hold_started
            last_admittance = None
            hold_duration = self.config.hold_duration_s
            if hold_duration <= 0 and hasattr(self.trajectory, "duration_s"):
                hold_duration = float(getattr(self.trajectory, "duration_s"))
            while (
                hold_duration <= 0
                or time.monotonic() - hold_started < hold_duration
            ):
                now = time.monotonic()
                wrench, normal_force, sample_time = self._read_force()
                current_position, _ = self.robot.read_pose()
                if normal_force > self.config.max_force_n:
                    raise RuntimeError(f"Force limit exceeded: {normal_force:.3f} N")
                dt = loop_period if last_admittance is None else now - last_admittance
                last_admittance = now
                delta = self.admittance.update(
                    self.config.target_force_n - normal_force, dt
                )
                if self.admittance.displacement_limited:
                    raise RuntimeError(
                        "Admittance displacement limit reached: "
                        f"{delta:.4f} m"
                    )
                point = self.trajectory.sample(time.monotonic() - hold_started)
                self._send(point, current_position, delta)
                self._log(now - self.started_time, wrench, normal_force, current_position)
                next_tick = max(next_tick + loop_period, time.monotonic())
                time.sleep(max(0.0, next_tick - time.monotonic()))

            self._retract(point.normal_base, point.orientation_xyzw)
            self._set_state("DONE")
            return self.state
        except KeyboardInterrupt:
            self.error = "Interrupted by operator"
            try:
                _, orientation = self.robot.read_pose()
                self._retract(np.array([0.0, 0.0, 1.0]), orientation)
            except Exception:
                LOGGER.exception("Emergency retract failed after operator interrupt")
            self._set_state("FAULT")
            return self.state
        except Exception as exc:
            self.error = str(exc)
            LOGGER.exception("Force controller fault: %s", exc)
            try:
                if robot_ready and motion_started:
                    current, orientation = self.robot.read_pose()
                    self._retract(np.array([0.0, 0.0, 1.0]), orientation)
            except Exception:
                LOGGER.exception("Emergency retract failed")
            self._set_state("FAULT")
            return self.state
        finally:
            if self.sampler is not None:
                self.sampler.stop()
            try:
                self.robot.stop()
            finally:
                self.robot.close()
                self.reader.close()
                self.logger.close()


def parse_admittance(mapping: dict[str, Any]) -> AdmittanceConfig:
    return AdmittanceConfig(
        mass_kg=float(mapping.get("mass_kg", 0.05)),
        damping_ns_m=float(mapping.get("damping_ns_m", 10.0)),
        stiffness_n_m=float(mapping.get("stiffness_n_m", 500.0)),
        max_press_speed_m_s=float(mapping.get("max_press_speed_m_s", 0.003)),
        max_retract_speed_m_s=float(mapping.get("max_retract_speed_m_s", 0.008)),
        max_displacement_m=float(mapping.get("max_displacement_m", 0.020)),
    )


def parse_runner_config(mapping: dict[str, Any]) -> RunnerConfig:
    force = mapping.get("force", {})
    control = mapping.get("control", {})
    safety = mapping.get("safety", {})
    standalone = mapping.get("standalone", {})
    sensor = mapping.get("sensor", {})
    return RunnerConfig(
        control_hz=float(control.get("control_hz", 100.0)),
        target_force_n=float(force.get("target_force_n", 5.0)),
        force_sign=float(force.get("force_sign", -1)),
        filter_cutoff_hz=float(force.get("filter_cutoff_hz", 2.0)),
        max_force_n=float(safety.get("max_force_n", 8.0)),
        sensor_stale_s=float(safety.get("sensor_stale_s", 0.2)),
        bias_duration_s=float(force.get("bias_duration_s", 2.0)),
        zero_settle_s=float(sensor.get("zero_settle_s", 0.1)),
        sensor_poll_interval_s=float(force.get("sensor_poll_interval_s", 0.02)),
        contact_threshold_n=float(standalone.get("contact_threshold_n", 0.5)),
        approach_timeout_s=float(standalone.get("approach_timeout_s", 10.0)),
        approach_speed_m_s=float(standalone.get("approach_speed_m_s", 0.002)),
        approach_depth_m=float(standalone.get("approach_depth_m", 0.05)),
        settle_s=float(standalone.get("settle_s", 0.5)),
        safe_clearance_m=float(standalone.get("safe_clearance_m", 0.02)),
        retract_s=float(standalone.get("retract_s", 1.0)),
        hold_duration_s=float(standalone.get("hold_duration_s", 0.0)),
        max_tangential_step_m=float(control.get("max_tangential_step_m", 0.01)),
        max_tangential_speed_m_s=float(
            control.get("max_tangential_speed_m_s", 0.05)
        ),
        log_path=mapping.get("logging", {}).get("path", "data/force_hybrid_push.csv"),
        admittance=parse_admittance(mapping.get("admittance", {})),
    )


def build_fixed_xy_trajectory(mapping: dict[str, Any], orientation_xyzw) -> FixedXYTrajectory:
    standalone = mapping.get("standalone", {})
    if "fixed_xy" not in standalone:
        raise ValueError("standalone.fixed_xy is required")
    trajectory = FixedXYTrajectory(
        standalone["fixed_xy"],
        standalone.get("orientation") or orientation_xyzw,
    )
    nominal_z = standalone.get("surface_z_m")
    if nominal_z is not None:
        trajectory.set_z_reference(float(nominal_z))
    return trajectory
