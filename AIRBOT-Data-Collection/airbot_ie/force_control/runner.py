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
    PlanarWaypointTrajectory,
    PoseTrajectory,
    TrajectoryPoint,
    fz_magnitude_from_wrench,
    normalize_quaternion,
    normalize_vector,
    normal_force_from_wrench,
    quaternion_to_matrix,
    quaternion_slerp,
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
    hardware_zero: bool = True
    rezero_after_preposition: bool = False
    sensor_poll_interval_s: float = 0.02
    contact_threshold_n: float = 5.0
    contact_confirm_s: float = 0.1
    approach_timeout_s: float = 10.0
    approach_speed_m_s: float = 0.002
    approach_depth_m: float = 0.05
    settle_s: float = 0.5
    safe_clearance_m: float = 0.02
    move_to_capture_pose: bool = False
    staged_move_to_trajectory: bool = False
    orientation_transition_s: float = 1.0
    safe_descent_s: float = 1.0
    high_z_m: float | None = None
    pose_feedback_timeout_s: float = 1.0
    orientation_feedback_timeout_s: float = 30.0
    vertical_axis_tolerance_deg: float = 12.0
    pose_position_tolerance_m: float = 0.015
    pose_orientation_tolerance_deg: float = 8.0
    force_hold_xy_tolerance_m: float = 0.02
    force_hold_xy_feedback_timeout_s: float = 60.0
    waypoint_xy_tolerance_m: float = 0.002
    waypoint_timeout_s: float = 15.0
    retract_s: float = 1.0
    hold_duration_s: float = 0.0
    max_tangential_step_m: float = 0.01
    max_tangential_speed_m_s: float = 0.05
    log_path: str | None = "data/force_hybrid_push.csv"
    force_display_interval_s: float = 0.2
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
            ("contact_confirm_s", self.contact_confirm_s),
            ("approach_timeout_s", self.approach_timeout_s),
            ("approach_speed_m_s", self.approach_speed_m_s),
            ("approach_depth_m", self.approach_depth_m),
            ("settle_s", self.settle_s),
            ("safe_clearance_m", self.safe_clearance_m),
            ("orientation_transition_s", self.orientation_transition_s),
            ("safe_descent_s", self.safe_descent_s),
            ("pose_feedback_timeout_s", self.pose_feedback_timeout_s),
            ("orientation_feedback_timeout_s", self.orientation_feedback_timeout_s),
            ("vertical_axis_tolerance_deg", self.vertical_axis_tolerance_deg),
            ("pose_position_tolerance_m", self.pose_position_tolerance_m),
            ("pose_orientation_tolerance_deg", self.pose_orientation_tolerance_deg),
            ("force_hold_xy_tolerance_m", self.force_hold_xy_tolerance_m),
            ("force_hold_xy_feedback_timeout_s", self.force_hold_xy_feedback_timeout_s),
            ("retract_s", self.retract_s),
            ("force_display_interval_s", self.force_display_interval_s),
        )
        for name, value in positive:
            if value <= 0:
                raise ValueError(f"{name} must be positive")
        for name, value in (
            ("waypoint_xy_tolerance_m", self.waypoint_xy_tolerance_m),
            ("waypoint_timeout_s", self.waypoint_timeout_s),
        ):
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if self.high_z_m is not None and (
            not np.isfinite(self.high_z_m) or self.high_z_m <= 0
        ):
            raise ValueError("high_z_m must be positive and finite when provided")
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
        "MOVE_CAPTURE",
        "MOVE_HIGH_XY",
        "MOVE_ORIENTATION",
        "MOVE_ORIENTATION_VERTICAL",
        "MOVE_ORIENTATION_RAW",
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

    def _read_force(
        self,
        orientation_xyzw: np.ndarray,
        normal_base: np.ndarray | None = None,
    ) -> tuple[np.ndarray, float, float]:
        if self.sampler is None:
            raise RuntimeError("Force sampler is not running")
        sample = self.sampler.latest(self.config.sensor_stale_s)
        filtered = self.filter.update(sample.values, sample.t_mono)
        if normal_base is None:
            normal_base = np.array([0.0, 0.0, 1.0])
        normal_force = normal_force_from_wrench(
            filtered,
            self.bias,
            orientation_xyzw,
            normal_base,
            self.config.force_sign,
        )
        return filtered, normal_force, sample.t_mono

    def _zero_and_collect_bias(self, phase: str) -> np.ndarray:
        """Zero the sensor and collect an unloaded bias for one robot pose.

        The wrench seen by a force/torque sensor depends on the tool attitude.
        A bias collected before trajectory preposition can therefore be invalid
        after the arm changes orientation, even while the tool is still
        suspended.  Keeping this operation in one helper makes the initial and
        post-preposition calibration paths identical.
        """

        if self.config.hardware_zero:
            LOGGER.info("Force sensor zero/tare: %s", phase)
            self.reader.zero()
            # Give a backend a short interval after zero/tare before sampling.
            time.sleep(self.config.zero_settle_s)
        else:
            LOGGER.warning(
                "Hardware zero/tare is disabled during %s; using software bias only",
                phase,
            )
        bias = collect_bias(
            self.reader,
            duration_s=self.config.bias_duration_s,
            poll_interval_s=self.config.sensor_poll_interval_s,
        )
        if not np.isfinite(bias).all():
            raise RuntimeError(f"Force bias is not finite after {phase}")
        LOGGER.info(
            "Force sensor bias after %s: %s",
            phase,
            np.array2string(bias, precision=6),
        )
        return bias

    def _send(
        self,
        point: TrajectoryPoint,
        current_position: np.ndarray,
        delta_n: float,
        *,
        limit_tangential_step: bool = True,
    ):
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
        if limit_tangential_step and tangent_norm > max_tangent_step:
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
        point: TrajectoryPoint | None = None,
        waypoint_index: int | None = None,
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
                "fz_magnitude_n": fz_magnitude_from_wrench(wrench, self.bias),
                "waypoint_index": waypoint_index,
                "target_x_m": point.position_base[0] if point is not None else "",
                "target_y_m": point.position_base[1] if point is not None else "",
                "xy_error_m": float(np.linalg.norm(position[:2] - point.position_base[:2]))
                if point is not None else "",
                "command_x_m": self.command_position[0]
                if self.command_position is not None else "",
                "command_y_m": self.command_position[1]
                if self.command_position is not None else "",
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

    def _servo_interpolate_pose(
        self,
        target_position: np.ndarray,
        target_orientation: np.ndarray,
        duration_s: float,
        phase: str,
        feedback_timeout_s: float | None = None,
        orientation_axis: int | None = None,
        orientation_axis_tolerance_deg: float | None = None,
    ) -> None:
        """Move continuously with linear position and quaternion SLERP."""

        if duration_s <= 0:
            raise ValueError("Pose interpolation duration must be positive")
        target_position = normalize_vector(target_position, 3, "target_position")
        target_orientation = normalize_quaternion(target_orientation)
        if not self.robot.start_pose_servo():
            raise RuntimeError(f"Could not enter AIRBOT pose servo mode for {phase}")
        start_position, start_orientation = self.robot.read_pose()
        start_position = normalize_vector(start_position, 3, "current_position")
        start_orientation = normalize_quaternion(start_orientation)
        steps = max(1, int(np.ceil(duration_s * self.config.control_hz)))
        LOGGER.info(
            "%s: start_position=%s target_position=%s target_orientation=%s duration=%.3fs",
            phase,
            np.array2string(start_position, precision=6),
            np.array2string(target_position, precision=6),
            np.array2string(target_orientation, precision=6),
            duration_s,
        )
        for index in range(1, steps + 1):
            fraction = index / steps
            position = start_position + fraction * (target_position - start_position)
            orientation = quaternion_slerp(start_orientation, target_orientation, fraction)
            if not self.robot.send_pose((position, orientation)):
                raise RuntimeError(f"AIRBOT rejected the {phase} command")
            time.sleep(duration_s / steps)
        self._wait_for_pose_feedback(
            target_position,
            target_orientation,
            phase,
            timeout_s=feedback_timeout_s,
            orientation_axis=orientation_axis,
            orientation_axis_tolerance_deg=orientation_axis_tolerance_deg,
        )

    def _wait_for_pose_feedback(
        self,
        target_position: np.ndarray,
        target_orientation: np.ndarray,
        phase: str,
        timeout_s: float | None = None,
        orientation_axis: int | None = None,
        orientation_axis_tolerance_deg: float | None = None,
    ) -> None:
        """Verify the asynchronous Cartesian servo reached its final pose.

        ``AIRBOTPlay.servo_cart_pose`` intentionally returns ``None`` after
        queueing a command, so a successful Python call does not mean the arm
        accepted or reached the pose.  AIRBOT also requires continuous pose
        commands while ``SERVO_CART_POSE`` is active.  Keep retransmitting the
        final target while checking feedback so a delayed arm does not stop
        after the interpolation stream ends.
        """

        feedback_timeout_s = (
            self.config.pose_feedback_timeout_s if timeout_s is None else float(timeout_s)
        )
        if feedback_timeout_s <= 0 or not np.isfinite(feedback_timeout_s):
            raise ValueError("Pose feedback timeout must be positive and finite")
        if orientation_axis is not None and orientation_axis not in (0, 1, 2):
            raise ValueError("orientation_axis must be 0, 1, or 2")
        axis_tolerance = (
            self.config.pose_orientation_tolerance_deg
            if orientation_axis_tolerance_deg is None
            else float(orientation_axis_tolerance_deg)
        )
        if axis_tolerance <= 0 or not np.isfinite(axis_tolerance):
            raise ValueError("Orientation axis tolerance must be positive and finite")
        deadline = time.monotonic() + feedback_timeout_s
        last_position_error = float("inf")
        last_orientation_error = float("inf")
        orientation_check_error = float("inf")
        last_orientation = None
        while time.monotonic() < deadline:
            if not self.robot.send_pose((target_position, target_orientation)):
                raise RuntimeError(f"AIRBOT rejected the {phase} target pose command")
            actual_position, actual_orientation = self.robot.read_pose()
            actual_position = normalize_vector(actual_position, 3, "feedback_position")
            actual_orientation = normalize_quaternion(actual_orientation)
            last_position_error = float(np.linalg.norm(actual_position - target_position))
            dot = float(abs(np.dot(actual_orientation, target_orientation)))
            last_orientation_error = float(
                np.degrees(2.0 * np.arccos(np.clip(dot, -1.0, 1.0)))
            )
            if orientation_axis is None:
                orientation_check_error = last_orientation_error
            else:
                target_axis = quaternion_to_matrix(target_orientation)[:, orientation_axis]
                actual_axis = quaternion_to_matrix(actual_orientation)[:, orientation_axis]
                axis_dot = float(np.clip(np.dot(target_axis, actual_axis), -1.0, 1.0))
                orientation_check_error = float(np.degrees(np.arccos(axis_dot)))
            last_orientation = actual_orientation
            if (
                last_position_error <= self.config.pose_position_tolerance_m
                and orientation_check_error <= axis_tolerance
            ):
                break
            time.sleep(min(0.02, 1.0 / self.config.control_hz))
        if last_orientation is None:
            raise RuntimeError(f"No AIRBOT pose feedback received after {phase}")
        target_z = quaternion_to_matrix(target_orientation)[:, 2]
        actual_z = quaternion_to_matrix(last_orientation)[:, 2]
        target_x = quaternion_to_matrix(target_orientation)[:, 0]
        actual_x = quaternion_to_matrix(last_orientation)[:, 0]
        LOGGER.info(
            "%s feedback: position_error=%.4fm orientation_error=%.2fdeg "
            "target_tcp_x=%s actual_tcp_x=%s target_tcp_z=%s actual_tcp_z=%s",
            phase,
            last_position_error,
            last_orientation_error,
            np.array2string(target_x, precision=5),
            np.array2string(actual_x, precision=5),
            np.array2string(target_z, precision=5),
            np.array2string(actual_z, precision=5),
        )
        if (
            last_position_error > self.config.pose_position_tolerance_m
            or orientation_check_error > axis_tolerance
        ):
            raise RuntimeError(
                f"AIRBOT did not reach the {phase} target pose: "
                f"position_error={last_position_error:.4f} m, "
                f"orientation_error={orientation_check_error:.2f} deg"
            )

    def _wait_for_force_hold_xy_feedback(
        self,
        target_position: np.ndarray,
        target_orientation: np.ndarray,
    ) -> None:
        """Keep sending the final force-hold pose until its XY position is accepted.

        Force hold uses an asynchronous Cartesian servo.  At the end of the
        trajectory the final command can still be queued while feedback is
        lagging behind, so continue sending that target until the horizontal
        position is within the configured tolerance.  A timeout is logged and
        the caller proceeds to retract, preventing an indefinite wait.
        """

        target_position = normalize_vector(target_position, 3, "force_hold_target_position")
        target_orientation = normalize_quaternion(target_orientation)
        deadline = time.monotonic() + self.config.force_hold_xy_feedback_timeout_s
        last_xy_error = float("inf")
        while time.monotonic() < deadline:
            if not self.robot.send_pose((target_position, target_orientation)):
                raise RuntimeError(
                    "AIRBOT rejected the FORCE_HOLD final target pose command"
                )
            actual_position, _ = self.robot.read_pose()
            actual_position = normalize_vector(actual_position, 3, "feedback_position")
            last_xy_error = float(np.linalg.norm(actual_position[:2] - target_position[:2]))
            if last_xy_error <= self.config.force_hold_xy_tolerance_m:
                LOGGER.info(
                    "FORCE_HOLD final XY accepted: xy_position_error=%.4f m "
                    "tolerance=%.4f m",
                    last_xy_error,
                    self.config.force_hold_xy_tolerance_m,
                )
                return
            time.sleep(min(0.02, 1.0 / self.config.control_hz))

        LOGGER.warning(
            "FORCE_HOLD final XY acceptance timed out after %.1fs: "
            "xy_position_error=%.4f m tolerance=%.4f m; proceeding to RETRACT",
            self.config.force_hold_xy_feedback_timeout_s,
            last_xy_error,
            self.config.force_hold_xy_tolerance_m,
        )

    def _move_capture_pose(self, capture_pose: np.ndarray) -> None:
        """Move to the observation pose, with a high-clearance fallback.

        AIRBOT's joint planner can occasionally reject a single large move
        that combines XY displacement and an orientation change, even though
        the same target is reachable through a high intermediate pose.
        """

        target_position = capture_pose[:3]
        target_orientation = capture_pose[3:]
        if self.robot.move_to_pose((target_position, target_orientation)):
            return

        current_position, current_orientation = self.robot.read_pose()
        high_z = max(float(current_position[2]), float(target_position[2]))
        LOGGER.warning(
            "AIRBOT rejected direct capture pose; retrying via high-clearance "
            "position with current orientation"
        )
        if not self.robot.move_to_pose(
            (
                np.array([current_position[0], current_position[1], high_z]),
                current_orientation,
            )
        ):
            raise RuntimeError("AIRBOT rejected the capture high-clearance lift")
        if not self.robot.move_to_pose(
            (
                np.array([target_position[0], target_position[1], high_z]),
                current_orientation,
            )
        ):
            raise RuntimeError("AIRBOT rejected the capture high-clearance XY move")
        self._servo_interpolate_pose(
            target_position,
            target_orientation,
            self.config.orientation_transition_s,
            "capture pose fallback orientation transition",
        )

    def run(self, close_resources: bool = True) -> str:
        """Run one segment.

        ``close_resources=True`` preserves the standalone command behaviour.
        A multi-segment policy loop passes ``False`` so it can return the arm
        to the observation pose and reuse the connected camera/force device.
        The force sampler and telemetry logger are always stopped here.
        """
        self.started_time = time.monotonic()
        robot_ready = False
        motion_started = False
        try:
            self.robot.connect()
            robot_ready = True
            current_position, current_orientation = self.robot.read_pose()
            self._set_state("PRECHECK")

            self.bias = self._zero_and_collect_bias("initial PRECHECK")

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
            capture_pose = getattr(self.trajectory, "capture_reference_pose", None)
            if self.config.move_to_capture_pose:
                if capture_pose is None:
                    raise RuntimeError(
                        "move_to_capture_pose requires trajectory capture_reference_pose"
                    )
                capture_pose = normalize_vector(capture_pose, 7, "capture_reference_pose")
                self._set_state("MOVE_CAPTURE")
                motion_started = True
                self._move_capture_pose(capture_pose)
                current_position, current_orientation = self.robot.read_pose()
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
            staged_preposition = (
                self.config.staged_move_to_trajectory and capture_pose is not None
            )
            if staged_preposition:
                motion_started = True
                # Separate the large XY move and the orientation/height change.
                # A single Cartesian plan to a low, surface-aligned pose can be
                # rejected even when each intermediate pose is reachable.
                # The capture pose is used for the initial observation only.
                # Once XY is known, perform the attitude transition at the
                # high observation height, not at the contact approach height.
                requested_high_z = (
                    float(capture_pose[2])
                    if self.config.high_z_m is None
                    else float(self.config.high_z_m)
                )
                high_z = max(safe_z, requested_high_z)
                if high_z != requested_high_z:
                    LOGGER.warning(
                        "Configured high_z_m=%.4f m is below the safe descent height "
                        "%.4f m; using %.4f m",
                        requested_high_z,
                        safe_z,
                        high_z,
                    )
                high_position = np.array(
                    [first.position_base[0], first.position_base[1], high_z],
                    dtype=float,
                )
                self._set_state("MOVE_HIGH_XY")
                LOGGER.info(
                    "Trajectory high XY target: position=%s orientation=%s "
                    "(requested_high_z=%.6f)",
                    np.array2string(high_position, precision=6),
                    np.array2string(np.asarray(capture_pose[3:]), precision=6),
                    requested_high_z,
                )
                if not self.robot.move_to_pose((high_position, capture_pose[3:])):
                    raise RuntimeError(
                        "AIRBOT rejected the trajectory high XY move "
                        f"at position={high_position.tolist()}"
                    )
                if getattr(self.trajectory, "replay_full_orientation", False):
                    vertical_orientation = self.trajectory.vertical_alignment_orientation(0.0)
                    self._set_state("MOVE_ORIENTATION_VERTICAL")
                    self._servo_interpolate_pose(
                        high_position,
                        vertical_orientation,
                        self.config.orientation_transition_s,
                        "Trajectory TCP X vertical alignment",
                        feedback_timeout_s=self.config.orientation_feedback_timeout_s,
                        orientation_axis=0,
                        orientation_axis_tolerance_deg=self.config.vertical_axis_tolerance_deg,
                    )
                    self._set_state("MOVE_ORIENTATION_RAW")
                    self._servo_interpolate_pose(
                        high_position,
                        first.orientation_xyzw,
                        self.config.orientation_transition_s,
                        "Trajectory raw orientation transition",
                        feedback_timeout_s=self.config.orientation_feedback_timeout_s,
                    )
                else:
                    self._set_state("MOVE_ORIENTATION")
                    self._servo_interpolate_pose(
                        high_position,
                        first.orientation_xyzw,
                        self.config.orientation_transition_s,
                        "Trajectory high orientation transition",
                        feedback_timeout_s=self.config.orientation_feedback_timeout_s,
                    )
                self._set_state("MOVE_SAFE")
                self._servo_interpolate_pose(
                    first_safe.position_base,
                    first.orientation_xyzw,
                    self.config.safe_descent_s,
                    "Trajectory vertical safe descent",
                )
            else:
                self._set_state("MOVE_SAFE")
            motion_started = True
            LOGGER.info(
                "MOVE_SAFE target: position=%s orientation=%s",
                np.array2string(first_safe.position_base, precision=6),
                np.array2string(first_safe.orientation_xyzw, precision=6),
            )
            if not staged_preposition:
                if not self.robot.move_to_pose(
                    (first_safe.position_base, first_safe.orientation_xyzw)
                ):
                    raise RuntimeError(
                        "AIRBOT rejected the safe move "
                        f"at position={first_safe.position_base.tolist()}"
                    )
            if self.config.rezero_after_preposition:
                LOGGER.info(
                    "Preposition complete; re-zeroing force sensor before APPROACH "
                    "while the tool remains suspended"
                )
                self.bias = self._zero_and_collect_bias("post-preposition")
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
            contact_started: float | None = None
            next_force_display = approach_started
            self._set_state("APPROACH")
            while True:
                now = time.monotonic()
                current_position, current_orientation = self.robot.read_pose()
                point = self.trajectory.sample(0.0)
                wrench, normal_force, _ = self._read_force(
                    current_orientation, point.normal_base
                )
                fz_magnitude = fz_magnitude_from_wrench(wrench, self.bias)
                if fz_magnitude > self.config.max_force_n:
                    raise RuntimeError(
                        f"Fz magnitude limit exceeded during approach: {fz_magnitude:.3f} N"
                    )
                if normal_force > self.config.max_force_n:
                    raise RuntimeError(f"Force limit exceeded during approach: {normal_force:.3f} N")
                if fz_magnitude >= self.config.contact_threshold_n:
                    if contact_started is None:
                        contact_started = now
                    elif now - contact_started >= self.config.contact_confirm_s:
                        LOGGER.info(
                            "Contact confirmed: |Fz_bias_corrected|=%.3f N for %.3fs",
                            fz_magnitude,
                            now - contact_started,
                        )
                        break
                else:
                    contact_started = None
                if now >= next_force_display:
                    LOGGER.info(
                        "APPROACH force: Fz_raw=%.3f N Fz_bias_corrected=%.3f N "
                        "|Fz_bias_corrected|=%.3f N normal_force=%.3f N z=%.4f m",
                        wrench[2],
                        wrench[2] - self.bias[2],
                        fz_magnitude,
                        normal_force,
                        current_position[2],
                    )
                    next_force_display = now + self.config.force_display_interval_s
                if now - approach_started > self.config.approach_timeout_s:
                    raise RuntimeError("Contact approach timed out")
                if approach_start_z - current_position[2] >= self.config.approach_depth_m:
                    raise RuntimeError("Maximum approach depth reached without contact")
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
                current_position, current_orientation = self.robot.read_pose()
                point = self.trajectory.sample(0.0)
                wrench, normal_force, _ = self._read_force(
                    current_orientation, point.normal_base
                )
                fz_magnitude = fz_magnitude_from_wrench(wrench, self.bias)
                if fz_magnitude > self.config.max_force_n:
                    raise RuntimeError(
                        f"Fz magnitude limit exceeded during settling: {fz_magnitude:.3f} N"
                    )
                if normal_force > self.config.max_force_n:
                    raise RuntimeError(f"Force limit exceeded during settling: {normal_force:.3f} N")
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
            waypoint_tracking = isinstance(self.trajectory, PlanarWaypointTrajectory)
            waypoint_index = 0
            waypoint_started = hold_started
            while (
                waypoint_tracking
                or hold_duration <= 0
                or time.monotonic() - hold_started < hold_duration
            ):
                now = time.monotonic()
                current_position, current_orientation = self.robot.read_pose()
                point = (
                    self.trajectory.sample_waypoint(waypoint_index)
                    if waypoint_tracking
                    else self.trajectory.sample(time.monotonic() - hold_started)
                )
                wrench, normal_force, sample_time = self._read_force(
                    current_orientation, point.normal_base
                )
                fz_magnitude = fz_magnitude_from_wrench(wrench, self.bias)
                if fz_magnitude > self.config.max_force_n:
                    raise RuntimeError(
                        f"Fz magnitude limit exceeded during force hold: {fz_magnitude:.3f} N"
                    )
                if normal_force > self.config.max_force_n:
                    raise RuntimeError(f"Force limit exceeded: {normal_force:.3f} N")
                dt = loop_period if last_admittance is None else now - last_admittance
                last_admittance = now
                delta = self.admittance.update(
                    self.config.target_force_n - fz_magnitude, dt
                )
                if self.admittance.displacement_limited:
                    raise RuntimeError(
                        "Admittance displacement limit reached: "
                        f"{delta:.4f} m"
                    )
                self._send(
                    point,
                    current_position,
                    delta,
                    limit_tangential_step=not waypoint_tracking,
                )
                self._log(
                    now - self.started_time,
                    wrench,
                    normal_force,
                    current_position,
                    point,
                    waypoint_index if waypoint_tracking else None,
                )
                if waypoint_tracking:
                    xy_error = float(np.linalg.norm(
                        current_position[:2] - point.position_base[:2]
                    ))
                    if xy_error <= self.config.waypoint_xy_tolerance_m:
                        LOGGER.info(
                            "FORCE_HOLD waypoint %d/%d accepted: xy_error=%.4f m",
                            waypoint_index + 1,
                            self.trajectory.waypoint_count,
                            xy_error,
                        )
                        waypoint_index += 1
                        if waypoint_index == self.trajectory.waypoint_count:
                            break
                        waypoint_started = time.monotonic()
                    elif now - waypoint_started >= self.config.waypoint_timeout_s:
                        raise RuntimeError(
                            f"FORCE_HOLD waypoint {waypoint_index + 1}/"
                            f"{self.trajectory.waypoint_count} timed out: "
                            f"xy_error={xy_error:.4f} m"
                        )
                next_tick = max(next_tick + loop_period, time.monotonic())
                time.sleep(max(0.0, next_tick - time.monotonic()))

            if waypoint_tracking:
                final_point = self.trajectory.sample_waypoint(
                    self.trajectory.waypoint_count - 1
                )
            else:
                final_point = self.trajectory.sample(hold_duration)
                self._wait_for_force_hold_xy_feedback(
                    final_point.position_base,
                    final_point.orientation_xyzw,
                )
            self._retract(final_point.normal_base, final_point.orientation_xyzw)
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
            self.logger.close()
            if close_resources:
                try:
                    self.robot.stop()
                finally:
                    self.robot.close()
                    self.reader.close()


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
    trajectory = mapping.get("trajectory", {})
    return RunnerConfig(
        control_hz=float(control.get("control_hz", 100.0)),
        target_force_n=float(force.get("target_force_n", 5.0)),
        force_sign=float(force.get("force_sign", -1)),
        filter_cutoff_hz=float(force.get("filter_cutoff_hz", 2.0)),
        max_force_n=float(safety.get("max_force_n", 8.0)),
        sensor_stale_s=float(safety.get("sensor_stale_s", 0.2)),
        bias_duration_s=float(force.get("bias_duration_s", 2.0)),
        zero_settle_s=float(sensor.get("zero_settle_s", 0.1)),
        hardware_zero=bool(sensor.get("hardware_zero", True)),
        rezero_after_preposition=bool(sensor.get("rezero_after_preposition", False)),
        sensor_poll_interval_s=float(force.get("sensor_poll_interval_s", 0.02)),
        contact_threshold_n=float(standalone.get("contact_threshold_n", 5.0)),
        contact_confirm_s=float(standalone.get("contact_confirm_s", 0.1)),
        approach_timeout_s=float(standalone.get("approach_timeout_s", 10.0)),
        approach_speed_m_s=float(standalone.get("approach_speed_m_s", 0.002)),
        approach_depth_m=float(standalone.get("approach_depth_m", 0.05)),
        settle_s=float(standalone.get("settle_s", 0.5)),
        safe_clearance_m=float(standalone.get("safe_clearance_m", 0.02)),
        move_to_capture_pose=bool(trajectory.get("move_to_capture_pose", False)),
        staged_move_to_trajectory=bool(
            trajectory.get("staged_move_to_trajectory", False)
        ),
        orientation_transition_s=float(
            trajectory.get("orientation_transition_s", 1.0)
        ),
        safe_descent_s=float(trajectory.get("safe_descent_s", 1.0)),
        high_z_m=(
            None
            if trajectory.get("high_z_m") is None
            else float(trajectory.get("high_z_m"))
        ),
        pose_feedback_timeout_s=float(safety.get("pose_feedback_timeout_s", 1.0)),
        orientation_feedback_timeout_s=float(
            safety.get("orientation_feedback_timeout_s", 30.0)
        ),
        vertical_axis_tolerance_deg=float(
            safety.get("vertical_axis_tolerance_deg", 12.0)
        ),
        pose_position_tolerance_m=float(
            safety.get("pose_position_tolerance_m", 0.015)
        ),
        pose_orientation_tolerance_deg=float(
            safety.get("pose_orientation_tolerance_deg", 8.0)
        ),
        force_hold_xy_tolerance_m=float(
            safety.get("force_hold_xy_tolerance_m", 0.02)
        ),
        force_hold_xy_feedback_timeout_s=float(
            safety.get("force_hold_xy_feedback_timeout_s", 60.0)
        ),
        waypoint_xy_tolerance_m=float(safety.get("waypoint_xy_tolerance_m", 0.002)),
        waypoint_timeout_s=float(safety.get("waypoint_timeout_s", 15.0)),
        retract_s=float(standalone.get("retract_s", 1.0)),
        hold_duration_s=float(standalone.get("hold_duration_s", 0.0)),
        max_tangential_step_m=float(control.get("max_tangential_step_m", 0.01)),
        max_tangential_speed_m_s=float(
            control.get("max_tangential_speed_m_s", 0.05)
        ),
        log_path=mapping.get("logging", {}).get("path", "data/force_hybrid_push.csv"),
        force_display_interval_s=float(
            mapping.get("logging", {}).get("force_display_interval_s", 0.2)
        ),
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
