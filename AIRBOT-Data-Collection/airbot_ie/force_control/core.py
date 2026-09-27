"""Pure, hardware-independent force-position control primitives.

The module deliberately knows nothing about AIRBOT or serial ports.  A
trajectory provider supplies a desired base-frame pose and outward surface
normal.  The executor can then reuse the same admittance controller for the
standalone fixed-XY test and for a later Push-Wiper planar trajectory.
"""

from __future__ import annotations

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


def surface_aligned_orientation(normal_base, yaw_rad: float) -> np.ndarray:
    """Build an orientation whose TCP Z axis points into the surface.

    The yaw is measured in the base frame, matching ``ACTION_DEFINITION``.
    The tangent heading is projected onto the surface plane so this helper is
    also ready for a future curved-surface trajectory provider.
    """

    normal = normalize_unit_vector(normal_base, "normal_base")
    z_axis = -normal
    heading = np.array([np.cos(yaw_rad), np.sin(yaw_rad), 0.0], dtype=float)
    x_axis = heading - z_axis * float(np.dot(heading, z_axis))
    if np.linalg.norm(x_axis) < 1e-9:
        fallback = np.array([1.0, 0.0, 0.0], dtype=float)
        x_axis = fallback - z_axis * float(np.dot(fallback, z_axis))
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    return matrix_to_quaternion(np.column_stack((x_axis, y_axis, z_axis)))


def normal_force_from_fz(fz_raw: float, bias_fz: float, force_sign: float = -1.0) -> float:
    """Return a positive inward contact force from the sensor FZ channel."""

    if not np.isfinite([fz_raw, bias_fz, force_sign]).all():
        raise ValueError("Force conversion received a non-finite value")
    if force_sign not in (-1, 1):
        raise ValueError("force_sign must be +1 or -1")
    return float(force_sign * (fz_raw - bias_fz))


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
        rotation = quaternion_to_matrix(self.start_orientation)
        if np.hypot(rotation[0, 0], rotation[1, 0]) < 1e-8:
            raise ValueError("Dragged starting pose has undefined base-Z yaw")
        self.start_yaw_rad = float(np.arctan2(rotation[1, 0], rotation[0, 0]))

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
                (0.0, 0.0, 1.0), self.start_yaw_rad
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
        self.replay_yaw = replay_yaw
        self.align_tool_z = bool(align_tool_z)
        capture_rotation = quaternion_to_matrix(self.capture_orientation_xyzw)
        if np.hypot(capture_rotation[0, 0], capture_rotation[1, 0]) < 1e-8:
            raise ValueError("Capture reference pose has undefined base-Z yaw")
        self.capture_yaw_rad = float(
            np.arctan2(capture_rotation[1, 0], capture_rotation[0, 0])
        )
        self.yaw = np.zeros(len(positions), dtype=float)

    @classmethod
    def from_npz(
        cls,
        path: str | Path,
        duration_s: float,
        surface_z_m: float | None = None,
        replay_yaw: bool = True,
        align_tool_z: bool = True,
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
            )
            instance.yaw = np.unwrap(actions[:, 2])
            return instance

    @property
    def duration_s(self) -> float:
        return float(self.times_s[-1])

    def set_z_reference(self, surface_z_m: float) -> None:
        if not np.isfinite(surface_z_m):
            raise ValueError("surface_z_m must be finite")
        self.surface_z_m = float(surface_z_m)

    def sample(self, t_s: float) -> TrajectoryPoint:
        if self.surface_z_m is None:
            raise RuntimeError("Set the planar trajectory Z reference before sampling")
        position_xy = _cubic_hermite(self.positions_xy, self.times_s, t_s)
        yaw = float(np.interp(t_s, self.times_s, self.yaw)) if self.replay_yaw else 0.0
        if self.align_tool_z:
            orientation = surface_aligned_orientation(
                (0.0, 0.0, 1.0), self.capture_yaw_rad + yaw
            )
        else:
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


# Public name used by the execution-layer interface.  The NPZ-backed provider
# is the first concrete Push-Wiper planar implementation and can later be
# replaced by an ASPI provider without changing HybridRunner.
PlanarPushWiperTrajectory = NpzPlanarTrajectory
