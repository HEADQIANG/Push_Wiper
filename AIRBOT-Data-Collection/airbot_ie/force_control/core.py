"""Pure, hardware-independent force-position control primitives.

The module deliberately knows nothing about AIRBOT or serial ports.  A
trajectory provider supplies a desired base-frame pose and outward surface
normal.  The executor can then reuse the same admittance controller for the
standalone fixed-XY test and for a later Push-Wiper planar trajectory.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import numpy as np


Pose = tuple[np.ndarray, np.ndarray]


def normalize_quaternion(value) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (4,) or not np.isfinite(result).all():
        raise ValueError("Quaternion must contain four finite values")
    length = float(np.linalg.norm(result))
    if not 0.9 <= length <= 1.1:
        raise ValueError(f"Quaternion norm is outside the valid range: {length}")
    return result / length


def normalize_vector(value, size: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {size} finite values")
    return result


def normalize_unit_vector(value, name: str) -> np.ndarray:
    result = normalize_vector(value, 3, name)
    length = float(np.linalg.norm(result))
    if length < 1e-9:
        raise ValueError(f"{name} must not be zero")
    return result / length


def quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """Hamilton product for XYZW quaternions."""

    x1, y1, z1, w1 = normalize_quaternion(left)
    x2, y2, z2, w2 = normalize_quaternion(right)
    return normalize_quaternion(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ]
    )


def yaw_quaternion(angle: float) -> np.ndarray:
    half = float(angle) / 2.0
    return np.array([0.0, 0.0, np.sin(half), np.cos(half)], dtype=float)


def quaternion_slerp(start, end, fraction: float) -> np.ndarray:
    """Interpolate two XYZW quaternions along the shortest rotation."""

    if not np.isfinite(fraction):
        raise ValueError("SLERP fraction must be finite")
    start_q = normalize_quaternion(start)
    end_q = normalize_quaternion(end)
    t = float(np.clip(fraction, 0.0, 1.0))
    dot = float(np.dot(start_q, end_q))
    if dot < 0.0:
        end_q = -end_q
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 1.0 - 1e-8:
        return normalize_quaternion(start_q + t * (end_q - start_q))
    angle = float(np.arccos(dot))
    sine = float(np.sin(angle))
    first_weight = np.sin((1.0 - t) * angle) / sine
    second_weight = np.sin(t * angle) / sine
    return normalize_quaternion(first_weight * start_q + second_weight * end_q)


def quaternion_to_matrix(value: np.ndarray) -> np.ndarray:
    """Return a 3x3 rotation matrix for an XYZW quaternion."""

    x, y, z, w = normalize_quaternion(value)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    """Convert a proper 3x3 rotation matrix to an XYZW quaternion."""

    rotation = np.asarray(matrix, dtype=float)
    if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
        raise ValueError("Rotation matrix must be 3x3 and finite")
    trace = float(np.trace(rotation))
    if trace > 0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = 2.0 * np.sqrt(max(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2], 1e-12))
        w = (rotation[2, 1] - rotation[1, 2]) / scale
        x = 0.25 * scale
        y = (rotation[0, 1] + rotation[1, 0]) / scale
        z = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = 2.0 * np.sqrt(max(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2], 1e-12))
        w = (rotation[0, 2] - rotation[2, 0]) / scale
        x = (rotation[0, 1] + rotation[1, 0]) / scale
        y = 0.25 * scale
        z = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = 2.0 * np.sqrt(max(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1], 1e-12))
        w = (rotation[1, 0] - rotation[0, 1]) / scale
        x = (rotation[0, 2] + rotation[2, 0]) / scale
        y = (rotation[1, 2] + rotation[2, 1]) / scale
        z = 0.25 * scale
    return normalize_quaternion([x, y, z, w])


def surface_aligned_orientation(
    normal_base, yaw_rad: float, tool_normal_axis: str = "z"
) -> np.ndarray:
    """Build an orientation whose selected TCP axis points into the surface.

    The yaw is measured in the base frame, matching ``ACTION_DEFINITION``.
    The tangent heading is projected onto the surface plane so this helper is
    also ready for a future curved-surface trajectory provider.

    ``tool_normal_axis`` is useful when the physical tool is mounted so that
    its working direction is the TCP X or Y axis rather than TCP Z.  The
    historical default remains ``z``.
    """

    normal = normalize_unit_vector(normal_base, "normal_base")
    axis = str(tool_normal_axis).strip().lower()
    if axis not in {"x", "y", "z"}:
        raise ValueError("tool_normal_axis must be one of x, y, z")
    inward = -normal
    heading = np.array([np.cos(yaw_rad), np.sin(yaw_rad), 0.0], dtype=float)
    tangent = heading - inward * float(np.dot(heading, inward))
    if np.linalg.norm(tangent) < 1e-9:
        fallback = np.array([1.0, 0.0, 0.0], dtype=float)
        tangent = fallback - inward * float(np.dot(fallback, inward))
    tangent /= np.linalg.norm(tangent)
    if axis == "z":
        z_axis = inward
        x_axis = tangent
        y_axis = np.cross(z_axis, x_axis)
    elif axis == "x":
        x_axis = inward
        y_axis = tangent
        z_axis = np.cross(x_axis, y_axis)
    else:
        y_axis = inward
        z_axis = tangent
        x_axis = np.cross(y_axis, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis /= np.linalg.norm(y_axis)
    z_axis /= np.linalg.norm(z_axis)
    return matrix_to_quaternion(np.column_stack((x_axis, y_axis, z_axis)))


def surface_aligned_orientation_from_reference(
    normal_base, reference_orientation, tool_normal_axis: str = "z"
) -> np.ndarray:
    """Align one tool axis with the inward normal while preserving a reference heading."""

    normal = normalize_unit_vector(normal_base, "normal_base")
    reference = quaternion_to_matrix(normalize_quaternion(reference_orientation))
    axis = str(tool_normal_axis).strip().lower()
    if axis not in {"x", "y", "z"}:
        raise ValueError("tool_normal_axis must be one of x, y, z")
    inward = -normal
    tangent_axis = {"x": 1, "y": 2, "z": 0}[axis]
    tangent = reference[:, tangent_axis]
    tangent = tangent - inward * float(np.dot(tangent, inward))
    if np.linalg.norm(tangent) < 1e-9:
        fallback = np.array([1.0, 0.0, 0.0])
        tangent = fallback - inward * float(np.dot(fallback, inward))
    tangent /= np.linalg.norm(tangent)
    if axis == "z":
        z_axis = inward
        x_axis = tangent
        y_axis = np.cross(z_axis, x_axis)
    elif axis == "x":
        x_axis = inward
        y_axis = tangent
        z_axis = np.cross(x_axis, y_axis)
    else:
        y_axis = inward
        z_axis = tangent
        x_axis = np.cross(y_axis, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis /= np.linalg.norm(y_axis)
    z_axis /= np.linalg.norm(z_axis)
    return matrix_to_quaternion(np.column_stack((x_axis, y_axis, z_axis)))


def normal_force_from_fz(fz_raw: float, bias_fz: float, force_sign: float = -1.0) -> float:
    """Return a positive inward contact force from the sensor FZ channel."""

    if not np.isfinite([fz_raw, bias_fz, force_sign]).all():
        raise ValueError("Force conversion received a non-finite value")
    if force_sign not in (-1, 1):
        raise ValueError("force_sign must be +1 or -1")
    return float(force_sign * (fz_raw - bias_fz))


def fz_magnitude_from_wrench(wrench, bias) -> float:
    """Return the absolute bias-corrected force from the sensor FZ channel."""

    values = normalize_vector(wrench, 6, "wrench")
    offset = normalize_vector(bias, 6, "bias")
    return float(abs(values[2] - offset[2]))


def normal_force_from_wrench(
    wrench,
    bias,
    orientation_xyzw,
    normal_base=(0.0, 0.0, 1.0),
    force_sign: float = -1.0,
) -> float:
    """Project a tool-frame wrench onto the inward base-frame surface normal.

    A captured NPZ pose can tilt the tool substantially.  In that case a
    vertical contact force is distributed across the sensor's FX/FY/FZ axes,
    so using only FZ misses contact.  ``force_sign=-1`` preserves the sensor
    convention used by the existing controller: compression points inward and
    is reported with a negative projection on the inward tool-frame axis.
    """

    values = normalize_vector(wrench, 6, "wrench")
    offset = normalize_vector(bias, 6, "bias")
    orientation = normalize_quaternion(orientation_xyzw)
    normal = normalize_unit_vector(normal_base, "normal_base")
    if not np.isfinite(force_sign) or force_sign not in (-1, 1):
        raise ValueError("force_sign must be either +1 or -1")
    inward_tool = quaternion_to_matrix(orientation).T @ (-normal)
    projected_inward = float(np.dot(values[:3] - offset[:3], inward_tool))
    return float(force_sign * projected_inward)


@dataclass(frozen=True)
class TrajectoryPoint:
    """Desired pose and outward surface normal in the robot base frame."""

    position_base: np.ndarray
    orientation_xyzw: np.ndarray
    normal_base: np.ndarray

    def __post_init__(self):
        position = normalize_vector(self.position_base, 3, "position_base")
        orientation = normalize_quaternion(self.orientation_xyzw)
        normal = normalize_unit_vector(self.normal_base, "normal_base")
        object.__setattr__(self, "position_base", position)
        object.__setattr__(self, "orientation_xyzw", orientation)
        object.__setattr__(self, "normal_base", normal)


class PoseTrajectory(Protocol):
    def sample(self, t_s: float) -> TrajectoryPoint:
        """Return a desired point, clamped at the trajectory boundaries."""


@dataclass
class AdmittanceConfig:
    """Cartesian normal-direction parameters in SI units."""

    mass_kg: float = 0.05
    damping_ns_m: float = 10.0
    stiffness_n_m: float = 500.0
    max_press_speed_m_s: float = 0.003
    max_retract_speed_m_s: float = 0.008
    max_displacement_m: float = 0.020

    def __post_init__(self):
        if self.mass_kg <= 0 or self.damping_ns_m < 0 or self.stiffness_n_m < 0:
            raise ValueError("Admittance mass must be positive and damping/stiffness non-negative")
        if self.max_press_speed_m_s <= 0 or self.max_retract_speed_m_s <= 0:
            raise ValueError("Admittance speed limits must be positive")
        if self.max_displacement_m <= 0:
            raise ValueError("Admittance displacement limit must be positive")


class AdmittanceController:
    """Discrete second-order admittance controller from the paper's Eq. (3).

    ``delta_m`` is the total displacement from the trajectory reference,
    positive toward the surface; it is not a per-cycle position increment.
    The executor maps it to the base frame as
    ``p_command = p_tracking - normal_base * delta_m``.
    """

    def __init__(self, config: AdmittanceConfig):
        self.config = config
        self.delta_m = 0.0
        self.velocity_m_s = 0.0
        self.displacement_limited = False

    def reset(self) -> None:
        self.delta_m = 0.0
        self.velocity_m_s = 0.0
        self.displacement_limited = False

    def update(self, force_error_n: float, dt_s: float) -> float:
        if not np.isfinite([force_error_n, dt_s]).all():
            raise ValueError("Admittance update received a non-finite value")
        if dt_s <= 0 or dt_s > 0.5:
            raise ValueError(f"Unexpected admittance timestep: {dt_s}")

        cfg = self.config
        # Backward Euler evaluates damping and stiffness at the new state.
        # Solving together with delta_next = delta + dt * velocity_next
        # avoids the numerical oscillation of the explicit update at 100 Hz
        # with the default mass, damping and stiffness.
        proposed_velocity = (
            cfg.mass_kg * self.velocity_m_s
            + dt_s * (float(force_error_n) - cfg.stiffness_n_m * self.delta_m)
        ) / (
            cfg.mass_kg
            + cfg.damping_ns_m * dt_s
            + cfg.stiffness_n_m * dt_s * dt_s
        )
        proposed_velocity = float(
            np.clip(
                proposed_velocity,
                -cfg.max_retract_speed_m_s,
                cfg.max_press_speed_m_s,
            )
        )
        proposed_delta = self.delta_m + proposed_velocity * dt_s
        clipped_delta = float(
            np.clip(proposed_delta, -cfg.max_displacement_m, cfg.max_displacement_m)
        )
        self.displacement_limited = clipped_delta != proposed_delta
        if clipped_delta != proposed_delta:
            proposed_velocity = 0.0
        self.velocity_m_s = proposed_velocity
        self.delta_m = clipped_delta
        return self.delta_m


class FixedXYTrajectory:
    """Fixed XY pose provider for the standalone force-hold experiment."""

    def __init__(self, fixed_xy, orientation_xyzw, normal_base=(0.0, 0.0, 1.0)):
        xy = normalize_vector(fixed_xy, 2, "fixed_xy")
        self.fixed_xy = xy
        self.orientation_xyzw = normalize_quaternion(orientation_xyzw)
        self.normal_base = normalize_unit_vector(normal_base, "normal_base")
        self.z_reference_m: float | None = None

    def set_z_reference(self, z_reference_m: float) -> None:
        if not np.isfinite(z_reference_m):
            raise ValueError("z_reference_m must be finite")
        self.z_reference_m = float(z_reference_m)

    def sample(self, t_s: float) -> TrajectoryPoint:
        if self.z_reference_m is None:
            raise RuntimeError("Set the fixed trajectory Z reference before sampling")
        return TrajectoryPoint(
            position_base=np.array(
                [self.fixed_xy[0], self.fixed_xy[1], self.z_reference_m], dtype=float
            ),
            orientation_xyzw=self.orientation_xyzw.copy(),
            normal_base=self.normal_base.copy(),
        )


class DragYSweepTrajectory:
    """Relative Y sweep captured from a manually dragged starting pose.

    The trajectory starts at the dragged base-frame XY position, moves in the
    positive base-Y direction by ``distance_m``, then returns to the starting
    Y coordinate.  Z is filled in by the runner after contact detection so the
    same provider can be used with the normal-direction admittance controller.
    """

    def __init__(
        self,
        start_position,
        start_orientation,
        distance_m: float = 0.10,
        duration_s: float | None = 6.0,
        align_tool_z: bool = True,
        tool_normal_axis: str = "z",
        surface_z_m: float | None = None,
        speed_m_s: float | None = None,
    ):
        position = normalize_vector(start_position, 3, "start_position")
        if distance_m <= 0 or not np.isfinite(distance_m):
            raise ValueError("distance_m must be positive and finite")
        if speed_m_s is not None:
            if speed_m_s <= 0 or not np.isfinite(speed_m_s):
                raise ValueError("speed_m_s must be positive and finite")
            # Smoothstep reaches 3*d/T at its peak for the out-and-back
            # profile, so choose T such that the peak speed is speed_m_s.
            duration_s = 3.0 * float(distance_m) / float(speed_m_s)
        if duration_s is None or duration_s <= 0 or not np.isfinite(duration_s):
            raise ValueError("duration_s must be positive and finite")
        self.start_position = position
        self.start_orientation = normalize_quaternion(start_orientation)
        self.distance_m = float(distance_m)
        self._duration_s = float(duration_s)
        if surface_z_m is not None and not np.isfinite(surface_z_m):
            raise ValueError("surface_z_m must be finite when provided")
        self.surface_z_m: float | None = (
            None if surface_z_m is None else float(surface_z_m)
        )
        # In drag mode the operator supplies a near-surface starting height.
        # The runner must not infer a point below it before the slow approach.
        self.approach_from_current = True
        self.align_tool_z = bool(align_tool_z)
        self.tool_normal_axis = str(tool_normal_axis).strip().lower()
        if self.tool_normal_axis not in {"x", "y", "z"}:
            raise ValueError("tool_normal_axis must be one of x, y, z")
        rotation = quaternion_to_matrix(self.start_orientation)
        yaw_axis = {"x": 1, "y": 2, "z": 0}[self.tool_normal_axis]
        yaw_vector = rotation[:, yaw_axis]
        if np.hypot(yaw_vector[0], yaw_vector[1]) < 1e-8:
            raise ValueError("Dragged starting pose has undefined base-plane yaw")
        self.start_yaw_rad = float(np.arctan2(yaw_vector[1], yaw_vector[0]))

    @property
    def duration_s(self) -> float:
        return self._duration_s

    def set_z_reference(self, surface_z_m: float) -> None:
        if not np.isfinite(surface_z_m):
            raise ValueError("surface_z_m must be finite")
        self.surface_z_m = float(surface_z_m)

    @staticmethod
    def _smoothstep(value: float) -> float:
        value = float(np.clip(value, 0.0, 1.0))
        return value * value * (3.0 - 2.0 * value)

    def sample(self, t_s: float) -> TrajectoryPoint:
        if self.surface_z_m is None:
            raise RuntimeError("Set the sweep trajectory Z reference before sampling")
        phase = float(np.clip(t_s, 0.0, self._duration_s) / self._duration_s)
        if phase <= 0.5:
            offset = self.distance_m * self._smoothstep(phase * 2.0)
        else:
            offset = self.distance_m * (1.0 - self._smoothstep((phase - 0.5) * 2.0))
        if self.align_tool_z:
            orientation = surface_aligned_orientation(
                (0.0, 0.0, 1.0), self.start_yaw_rad, self.tool_normal_axis
            )
        else:
            orientation = self.start_orientation.copy()
        return TrajectoryPoint(
            position_base=np.array(
                [self.start_position[0], self.start_position[1] + offset, self.surface_z_m],
                dtype=float,
            ),
            orientation_xyzw=orientation,
            normal_base=np.array([0.0, 0.0, 1.0], dtype=float),
        )


def _cubic_hermite(values: np.ndarray, times: np.ndarray, query: float) -> np.ndarray:
    """C1 interpolation with finite-difference tangents.

    This is an in-repo dependency-free equivalent of the smooth waypoint
    interpolation needed before a B-spline implementation is introduced.
    """

    if len(values) == 1:
        return values[0].copy()
    q = float(np.clip(query, times[0], times[-1]))
    index = int(np.searchsorted(times, q, side="right") - 1)
    index = min(max(index, 0), len(times) - 2)
    t0, t1 = times[index], times[index + 1]
    h = t1 - t0
    u = 0.0 if h <= 0 else (q - t0) / h
    tangents = np.gradient(values, times, axis=0, edge_order=1)
    h00 = 2 * u**3 - 3 * u**2 + 1
    h10 = u**3 - 2 * u**2 + u
    h01 = -2 * u**3 + 3 * u**2
    h11 = u**3 - u**2
    return (
        h00 * values[index]
        + h10 * h * tangents[index]
        + h01 * values[index + 1]
        + h11 * h * tangents[index + 1]
    )


class NpzPlanarTrajectory:
    """Planar Push-Wiper trajectory provider with optional yaw replay."""

    def __init__(
        self,
        positions_xy: np.ndarray,
        times_s: np.ndarray,
        surface_z_m: float | None,
        capture_orientation_xyzw,
        replay_yaw: bool = True,
        align_tool_z: bool = True,
        orientations_xyzw: np.ndarray | None = None,
        replay_full_orientation: bool = False,
        tool_normal_axis: str = "z",
    ):
        positions = np.asarray(positions_xy, dtype=float)
        times = np.asarray(times_s, dtype=float)
        if positions.ndim != 2 or positions.shape[1] != 2 or len(positions) < 2:
            raise ValueError("Planar trajectory must contain at least two XY points")
        if not np.isfinite(positions).all():
            raise ValueError("Planar trajectory contains non-finite positions")
        if times.shape != (len(positions),) or not np.isfinite(times).all():
            raise ValueError("Planar trajectory times do not match the XY points")
        if np.any(np.diff(times) <= 0):
            raise ValueError("Planar trajectory times must be strictly increasing")
        self.positions_xy = positions
        self.times_s = times
        if surface_z_m is not None and not np.isfinite(surface_z_m):
            raise ValueError("surface_z_m must be finite when provided")
        self.surface_z_m = None if surface_z_m is None else float(surface_z_m)
        self.capture_orientation_xyzw = normalize_quaternion(capture_orientation_xyzw)
        # Filled by from_npz/from_actions.  The position is the observation
        # pose and must not be confused with the surface height.
        self.capture_reference_pose: np.ndarray | None = None
        self.replay_yaw = replay_yaw
        self.align_tool_z = bool(align_tool_z)
        self.tool_normal_axis = str(tool_normal_axis).strip().lower()
        if self.tool_normal_axis not in {"x", "y", "z"}:
            raise ValueError("tool_normal_axis must be one of x, y, z")
        self.replay_full_orientation = bool(replay_full_orientation)
        if orientations_xyzw is None:
            self.orientations_xyzw = None
        else:
            orientations = np.asarray(orientations_xyzw, dtype=float)
            if orientations.shape != (len(positions), 4) or not np.isfinite(orientations).all():
                raise ValueError(
                    "orientations_xyzw must have shape (N, 4) and finite values"
                )
            self.orientations_xyzw = np.stack(
                [normalize_quaternion(value) for value in orientations]
            )
        if self.replay_full_orientation and self.orientations_xyzw is None:
            raise ValueError(
                "replay_full_orientation requires NPZ orientations_xyzw data"
            )
        capture_rotation = quaternion_to_matrix(self.capture_orientation_xyzw)
        yaw_axis = {"x": 1, "y": 2, "z": 0}[self.tool_normal_axis]
        yaw_vector = capture_rotation[:, yaw_axis]
        if np.hypot(yaw_vector[0], yaw_vector[1]) < 1e-8:
            raise ValueError("Capture reference pose has undefined base-plane yaw")
        self.capture_yaw_rad = float(
            np.arctan2(yaw_vector[1], yaw_vector[0])
        )
        self.yaw = np.zeros(len(positions), dtype=float)

    @classmethod
    def from_actions(
        cls,
        actions,
        capture_reference_pose,
        duration_s: float,
        surface_z_m: float | None,
        replay_yaw: bool = True,
        align_tool_z: bool = True,
        replay_full_orientation: bool = False,
        tool_normal_axis: str = "z",
    ) -> "NpzPlanarTrajectory":
        """Build the planar ASPI equivalent directly from policy actions.

        Policy actions use the repository contract ``[x_base, y_base,
        delta_yaw]``.  When ``replay_yaw`` is disabled, an ``[x_base,
        y_base]`` array is also accepted and the trajectory keeps a fixed
        orientation.  The reference pose is the fixed observation pose used
        during training; its position is part of the policy observation and
        its orientation defines the yaw frame.
        """

        if duration_s <= 0 or not np.isfinite(duration_s):
            raise ValueError("duration_s must be positive and finite")
        values = np.asarray(actions, dtype=float)
        if values.shape not in {(16, 2), (16, 3)} or not np.isfinite(values).all():
            raise ValueError("Policy actions must have shape (16, 2) or (16, 3) and finite values")
        if replay_yaw and values.shape[1] != 3:
            raise ValueError("replay_yaw=True requires policy actions with delta_yaw")
        capture = np.asarray(capture_reference_pose, dtype=float)
        if capture.shape != (7,) or not np.isfinite(capture).all():
            raise ValueError("capture_reference_pose must have shape (7,)")
        instance = cls(
            values[:, :2],
            np.linspace(0.0, float(duration_s), len(values)),
            surface_z_m,
            capture[3:],
            replay_yaw=replay_yaw,
            align_tool_z=align_tool_z,
            replay_full_orientation=replay_full_orientation,
            tool_normal_axis=tool_normal_axis,
        )
        # ``delta_yaw`` is an unwrapped increment relative to the fixed O
        # orientation. Preserve it when present; XY-only trajectories keep
        # the constructor's zero yaw sequence and therefore a fixed pose.
        instance.yaw = np.unwrap(values[:, 2]) if values.shape[1] == 3 else np.zeros(len(values))
        instance.capture_reference_pose = capture.copy()
        return instance

    @classmethod
    def from_npz(
        cls,
        path: str | Path,
        duration_s: float,
        surface_z_m: float | None = None,
        replay_yaw: bool = True,
        align_tool_z: bool = True,
        replay_full_orientation: bool = False,
        tool_normal_axis: str = "z",
    ) -> "NpzPlanarTrajectory":
        if duration_s <= 0:
            raise ValueError("duration_s must be positive")
        with np.load(path, allow_pickle=False) as data:
            # Keep deployment tied to the repository's exported action
            # contract: absolute base-frame x/y and delta_yaw relative to the
            # fixed observation reference pose.
            try:
                from airbot_ie.push_wiper.geometry import ACTION_DEFINITION

                expected_version = int(ACTION_DEFINITION["version"])
            except ImportError:
                expected_version = 2
            if "action_definition_version" in data:
                version = int(np.asarray(data["action_definition_version"]).reshape(()))
                if version != expected_version:
                    raise ValueError(
                        "Unsupported NPZ action_definition_version: "
                        f"{version}; expected {expected_version}"
                    )
            actions = np.asarray(
                data["actions_full"] if "actions_full" in data else data["actions"],
                dtype=float,
            )
            if actions.ndim != 2 or actions.shape[1] < 3:
                raise ValueError("NPZ actions must have shape (N, 3)")
            if "capture_reference_pose" not in data:
                raise ValueError("NPZ is missing capture_reference_pose")
            capture = np.asarray(data["capture_reference_pose"], dtype=float)
            if capture.shape != (7,):
                raise ValueError("capture_reference_pose must have shape (7,)")
            if not np.isfinite(capture).all():
                raise ValueError("capture_reference_pose contains non-finite values")
            if surface_z_m is None and "work_height_m" in data:
                surface_z_m = float(np.asarray(data["work_height_m"]).reshape(()))
            orientations = None
            if replay_full_orientation:
                if "orientations_xyzw" not in data:
                    raise ValueError(
                        "replay_full_orientation requires NPZ orientations_xyzw data"
                    )
                orientations = np.asarray(data["orientations_xyzw"], dtype=float)
                if orientations.shape != (len(actions), 4):
                    raise ValueError(
                        "NPZ orientations_xyzw must have shape (N, 4)"
                    )
            # Older exports have no work-height annotation.  Keep the value
            # unset so the runner can choose a bounded approach from the current
            # robot pose; capture_reference_pose.z is a camera reference height,
            # not a measured surface height.
            if "timestamps_ns" in data and len(data["timestamps_ns"]) == len(actions):
                raw_times = np.asarray(data["timestamps_ns"], dtype=np.int64)
                elapsed = (raw_times - raw_times[0]).astype(float) / 1e9
                if len(elapsed) < 2 or np.any(np.diff(elapsed) <= 0):
                    elapsed = np.linspace(0.0, duration_s, len(actions))
                else:
                    elapsed = elapsed / elapsed[-1] * duration_s
            else:
                elapsed = np.linspace(0.0, duration_s, len(actions))
            instance = cls(
                actions[:, :2],
                elapsed,
                surface_z_m,
                capture[3:],
                replay_yaw=replay_yaw,
                align_tool_z=align_tool_z,
                orientations_xyzw=orientations,
                replay_full_orientation=replay_full_orientation,
                tool_normal_axis=tool_normal_axis,
            )
            instance.yaw = np.unwrap(actions[:, 2])
            instance.capture_reference_pose = capture.copy()
            return instance

    @classmethod
    def from_xy_json(
        cls,
        path: str | Path,
        capture_reference_pose=None,
        duration_s: float | None = None,
        surface_z_m: float | None = None,
        align_tool_z: bool = True,
        tool_normal_axis: str = "z",
    ) -> "NpzPlanarTrajectory":
        """Load a time-parameterized XY JSON trajectory with fixed orientation.

        The JSON format is the report emitted by the Push-Wiper offline exporter:
        ``control_points`` (preferred) or ``samples`` contain objects with
        ``t_s``, ``x_m`` and ``y_m``.  No yaw is read or replayed.  If
        ``duration_s`` is supplied, the source time axis is rescaled to it.
        The exported ``capture_reference_pose`` and ``surface_z_m`` metadata
        are used when the corresponding arguments are omitted.
        """

        source = Path(path).expanduser()
        try:
            payload = json.loads(source.read_text(encoding="utf-8"))
        except OSError as exc:
            raise FileNotFoundError(f"Could not read XY trajectory JSON: {source}") from exc
        except json.JSONDecodeError as exc:
            raise ValueError(f"Invalid XY trajectory JSON: {source}") from exc
        if not isinstance(payload, dict):
            raise ValueError("XY trajectory JSON root must be an object")
        records = payload.get("control_points") or payload.get("samples")
        if not isinstance(records, list) or len(records) < 2:
            raise ValueError("XY trajectory JSON must contain at least two control_points or samples")
        try:
            times = np.asarray([float(item["t_s"]) for item in records], dtype=float)
            positions = np.asarray(
                [[float(item["x_m"]), float(item["y_m"])] for item in records],
                dtype=float,
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("XY trajectory records must contain finite t_s, x_m and y_m") from exc
        if not np.isfinite(times).all() or not np.isfinite(positions).all():
            raise ValueError("XY trajectory JSON contains non-finite values")
        times = times - times[0]
        if np.any(np.diff(times) <= 0) or times[-1] <= 0:
            raise ValueError("XY trajectory times must be strictly increasing")
        source_duration = float(times[-1])
        payload_duration = payload.get("duration_s")
        target_duration = (
            source_duration
            if duration_s is None and payload_duration is None
            else float(payload_duration if duration_s is None else duration_s)
        )
        if target_duration <= 0 or not np.isfinite(target_duration):
            raise ValueError("XY trajectory duration_s must be positive and finite")
        times = times / source_duration * target_duration
        if surface_z_m is None and payload.get("surface_z_m") is not None:
            surface_z_m = float(payload["surface_z_m"])
        if capture_reference_pose is None:
            capture_reference_pose = payload.get("capture_reference_pose")
        if capture_reference_pose is None:
            raise ValueError(
                "XY trajectory JSON must contain capture_reference_pose or provide it in config"
            )
        capture = np.asarray(capture_reference_pose, dtype=float)
        if capture.shape != (7,) or not np.isfinite(capture).all():
            raise ValueError("capture_reference_pose must have shape (7,)")
        instance = cls(
            positions,
            times,
            surface_z_m,
            capture[3:],
            replay_yaw=False,
            align_tool_z=align_tool_z,
            tool_normal_axis=tool_normal_axis,
        )
        instance.capture_reference_pose = capture.copy()
        return instance

    @property
    def duration_s(self) -> float:
        return float(self.times_s[-1])

    def vertical_alignment_orientation(self, t_s: float = 0.0) -> np.ndarray:
        """Return an orientation with the configured tool axis vertical inward.

        Raw orientation replay is staged by the force runner: this orientation
        is reached first, then the recorded raw quaternion is applied.  The
        planar yaw reference is preserved while the tool normal axis is aligned
        with the surface normal.
        """

        if not self.replay_full_orientation:
            raise RuntimeError(
                "Vertical alignment staging requires full raw orientation replay"
            )
        clipped_t = float(np.clip(t_s, self.times_s[0], self.times_s[-1]))
        upper = int(np.searchsorted(self.times_s, clipped_t, side="right"))
        if upper <= 0:
            reference = self.orientations_xyzw[0]
        elif upper >= len(self.times_s):
            reference = self.orientations_xyzw[-1]
        else:
            lower = upper - 1
            span = self.times_s[upper] - self.times_s[lower]
            fraction = (clipped_t - self.times_s[lower]) / span
            reference = quaternion_slerp(
                self.orientations_xyzw[lower],
                self.orientations_xyzw[upper],
                fraction,
            )
        return surface_aligned_orientation_from_reference(
            (0.0, 0.0, 1.0), reference, self.tool_normal_axis
        )

    def set_z_reference(self, surface_z_m: float) -> None:
        if not np.isfinite(surface_z_m):
            raise ValueError("surface_z_m must be finite")
        self.surface_z_m = float(surface_z_m)

    def sample(self, t_s: float) -> TrajectoryPoint:
        position_xy = _cubic_hermite(self.positions_xy, self.times_s, t_s)
        return self._point_at(position_xy, t_s)

    def _point_at(self, position_xy: np.ndarray, t_s: float) -> TrajectoryPoint:
        if self.surface_z_m is None:
            raise RuntimeError("Set the planar trajectory Z reference before sampling")
        if self.replay_full_orientation:
            clipped_t = float(np.clip(t_s, self.times_s[0], self.times_s[-1]))
            upper = int(np.searchsorted(self.times_s, clipped_t, side="right"))
            if upper <= 0:
                orientation = self.orientations_xyzw[0].copy()
            elif upper >= len(self.times_s):
                orientation = self.orientations_xyzw[-1].copy()
            else:
                lower = upper - 1
                span = self.times_s[upper] - self.times_s[lower]
                fraction = (clipped_t - self.times_s[lower]) / span
                orientation = quaternion_slerp(
                    self.orientations_xyzw[lower],
                    self.orientations_xyzw[upper],
                    fraction,
                )
        else:
            yaw = float(np.interp(t_s, self.times_s, self.yaw)) if self.replay_yaw else 0.0
        if not self.replay_full_orientation and self.align_tool_z:
            orientation = surface_aligned_orientation(
                (0.0, 0.0, 1.0), self.capture_yaw_rad + yaw,
                self.tool_normal_axis,
            )
        elif not self.replay_full_orientation:
            orientation = (
                quaternion_multiply(yaw_quaternion(yaw), self.capture_orientation_xyzw)
                if self.replay_yaw
                else self.capture_orientation_xyzw.copy()
            )
        return TrajectoryPoint(
            position_base=np.array(
                [position_xy[0], position_xy[1], self.surface_z_m], dtype=float
            ),
            orientation_xyzw=orientation,
            normal_base=np.array([0.0, 0.0, 1.0], dtype=float),
        )


class PlanarWaypointTrajectory(NpzPlanarTrajectory):
    """Expose the original planar targets without interpolating their XY positions."""

    @property
    def waypoint_count(self) -> int:
        return len(self.positions_xy)

    def sample_waypoint(self, index: int) -> TrajectoryPoint:
        if not 0 <= index < self.waypoint_count:
            raise IndexError(f"Waypoint index out of range: {index}")
        return self._point_at(self.positions_xy[index], float(self.times_s[index]))

    def sample(self, t_s: float) -> TrajectoryPoint:
        index = int(np.searchsorted(self.times_s, t_s, side="right")) - 1
        index = max(0, min(index, self.waypoint_count - 1))
        return self.sample_waypoint(index)


# Public name used by the execution-layer interface.  The NPZ-backed provider
# is the first concrete Push-Wiper planar implementation and can later be
# replaced by an ASPI provider without changing HybridRunner.
PlanarPushWiperTrajectory = NpzPlanarTrajectory
