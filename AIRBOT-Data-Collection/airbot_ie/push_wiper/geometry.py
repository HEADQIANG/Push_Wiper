"""SDK poses use metres and quaternion XYZW. No tool offset is applied."""

import numpy as np

ACTION_DEFINITION = {
    "version": 2,
    "components": ["x", "y", "delta_yaw"],
    "position_reference": "follower SDK end reference in follower base",
    "yaw_reference": "references.observation.follow.orientation",
    "yaw_axis": "follower base Z",
    "yaw_convention": "ZYX yaw(R)=atan2(R[1,0], R[0,0]); unwrap(wrap(yaw(R)-yaw(R_cap)))",
    "units": {"position": "m", "angle": "rad"},
    "quaternion_order": "xyzw",
}


def vector(value, size):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"Expected {size} finite values")
    return result


def quaternion(value):
    result = vector(value, 4)
    length = np.linalg.norm(result)
    if not 0.9 <= length <= 1.1:
        raise ValueError("Invalid SDK quaternion")
    return result / length


def rotation(value):
    x, y, z, w = quaternion(value)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def pose_error(current, reference):
    distance = np.linalg.norm(
        vector(current["position"], 3) - vector(reference["position"], 3)
    )
    dot = abs(
        float(quaternion(current["orientation"]) @ quaternion(reference["orientation"]))
    )
    angle = np.degrees(2 * np.arccos(np.clip(dot, 0, 1)))
    return float(distance), float(angle)


def base_yaw(orientation):
    """ZYX yaw: heading of SDK end-frame X projected onto the base XY plane."""
    matrix = rotation(orientation)
    if np.hypot(matrix[0, 0], matrix[1, 0]) < 1e-8:
        raise ValueError("Base-Z yaw is undefined: SDK end-frame X is vertical")
    return np.arctan2(matrix[1, 0], matrix[0, 0])


def planar_actions(positions, orientations, capture_orientation):
    """Base XY and unwrapped yaw relative to the fixed capture pose p_cap.

    The paper specifies p_cap as the reference, but not an Euler convention.
    We use base-frame ZYX yaw differences, keeping changes of roll/pitch out
    of this label. Full positions and quaternions remain in the raw data.
    """
    reference_yaw = base_yaw(capture_orientation)
    angles = [base_yaw(orientation) - reference_yaw for orientation in orientations]
    angles = np.arctan2(np.sin(angles), np.cos(angles))
    positions = np.asarray(positions, dtype=float)
    return np.column_stack((positions[:, :2], np.unwrap(angles)))


def resample_actions(stamps_ns, actions, count=16):
    stamps = np.asarray(stamps_ns, dtype=np.int64)
    if len(stamps) < 2 or np.any(np.diff(stamps) <= 0):
        raise ValueError("A stroke needs at least two strictly increasing timestamps")
    elapsed = (stamps - stamps[0]).astype(float) / 1e9
    target = np.linspace(0, elapsed[-1], count)
    return np.column_stack(
        [np.interp(target, elapsed, actions[:, index]) for index in range(3)]
    )
