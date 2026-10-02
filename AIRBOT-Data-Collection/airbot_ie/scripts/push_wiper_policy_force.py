"""Run Push-Wiper policy inference followed by planar admittance force control.

The loop follows the paper's gathering cycle: observe at the fixed O pose,
predict one complete planar segment, execute it with normal force control,
return to O, and observe again.  Curved-surface adaptation and post-processing
motions are deliberately outside this entry point.
"""

from __future__ import annotations

import argparse
import copy
import json
import logging
import time
from pathlib import Path

import numpy as np

from airbot_ie.force_control.core import NpzPlanarTrajectory, PlanarWaypointTrajectory
from airbot_ie.force_control.hardware import AirbotPlayAdapter, make_force_reader
from airbot_ie.force_control.policy_client import (
    ACTION_DEFINITION_VERSION,
    PolicyClient,
    inspect_planar_actions,
    validate_planar_actions,
)
from airbot_ie.force_control.runner import HybridRunner, parse_runner_config
from airbot_ie.push_wiper.config import CameraConfig
from airbot_ie.push_wiper.clock import acquisition_ns
from airbot_ie.push_wiper.devices import CameraWorker
from airbot_ie.push_wiper.export import MaskConfig, stain_mask

LOGGER = logging.getLogger(__name__)


def load_json(path: str | Path) -> dict:
    value = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def resolve_config_paths(mapping: dict, config_dir: Path) -> dict:
    """Resolve paths relative to the JSON file, independent of the shell cwd."""

    result = copy.deepcopy(mapping)
    for section, key in ((None, "references"), (None, "mask_config"), ("policy", "checkpoint"), ("policy", "python"), ("policy", "root")):
        owner = result if section is None else result.get(section, {})
        if key in owner and not Path(owner[key]).expanduser().is_absolute():
            candidate = config_dir / owner[key]
            # Keep the virtualenv interpreter symlink intact.  Path.resolve()
            # would turn ``.venv/bin/python`` into the system interpreter and
            # silently lose torch/dill from the policy environment.
            owner[key] = str(candidate.absolute() if section == "policy" and key == "python" else candidate.resolve())
    return result


def _finite_vector(value, size: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {size} finite values")
    return result


def load_references(path: str | Path, mapping: dict) -> tuple[np.ndarray, float]:
    references = load_json(path)
    observation = references.get("observation", {}).get("follow")
    if not isinstance(observation, dict):
        raise ValueError("references.observation.follow is required")
    capture = np.r_[
        _finite_vector(observation.get("position"), 3, "observation.follow.position"),
        _finite_vector(observation.get("orientation"), 4, "observation.follow.orientation"),
    ]
    height = references.get("work_height", {}).get("z_m")
    if height is None or not np.isfinite(float(height)):
        raise ValueError("references.work_height.z_m is required for planar force control")
    bindings = references.get("bindings", {})
    robot = mapping.get("robot", {})
    bound_follow = bindings.get("follow")
    if bound_follow is not None and list(bound_follow) != [robot.get("url", "localhost"), int(robot.get("port", 50051))]:
        raise ValueError("Robot endpoint differs from the endpoint stored in references")
    camera = mapping.get("camera", {})
    bound_camera = bindings.get("camera", {})
    for key in ("serial", "width", "height", "fps", "stale_s", "roi"):
        if key in bound_camera and bound_camera.get(key) != camera.get(key):
            raise ValueError(f"Camera binding differs from references for {key!r}")
    return capture, float(height)


def validate_actions(actions, safety: dict) -> np.ndarray:
    """Apply hard limits to one model segment; reject rather than clip."""

    return validate_planar_actions(actions, safety)


def inspect_trajectory_workspace(trajectory: NpzPlanarTrajectory, safety: dict) -> dict:
    """Inspect the planar targets or interpolated spline without clipping."""

    workspace = safety["workspace"]
    times = (
        trajectory.times_s
        if isinstance(trajectory, PlanarWaypointTrajectory)
        else np.linspace(0.0, trajectory.duration_s, 129)
    )
    positions = np.stack([trajectory.sample(float(t)).position_base[:2] for t in times])
    limits = {
        key: float(workspace[key])
        for key in ("x_min_m", "x_max_m", "y_min_m", "y_max_m")
    }
    metrics = {
        "x_min_m": float(positions[:, 0].min()),
        "x_max_m": float(positions[:, 0].max()),
        "y_min_m": float(positions[:, 1].min()),
        "y_max_m": float(positions[:, 1].max()),
    }
    violations = []
    if (
        metrics["x_min_m"] < limits["x_min_m"]
        or metrics["x_max_m"] > limits["x_max_m"]
        or metrics["y_min_m"] < limits["y_min_m"]
        or metrics["y_max_m"] > limits["y_max_m"]
    ):
        violations.append("Planar trajectory exceeds the configured workspace")
    return {
        "ok": not violations,
        "violations": violations,
        "metrics": metrics,
        "limits": limits,
    }


def validate_trajectory_workspace(trajectory: NpzPlanarTrajectory, safety: dict) -> None:
    """Check the executed targets, including spline overshoot for interpolated providers."""

    report = inspect_trajectory_workspace(trajectory, safety)
    if not report["ok"]:
        raise ValueError(report["violations"][0])


def _validate_mask(mask) -> np.ndarray:
    value = np.asarray(mask)
    if value.shape != (480, 640):
        raise ValueError(f"Offline mask must have shape (480, 640), got {value.shape}")
    if value.dtype != np.uint8:
        value = value.astype(np.uint8)
    if not np.isin(value, (0, 1)).all() or not np.any(value == 0):
        raise ValueError("Offline mask must be binary and contain dirt pixels")
    return value


def _mask_stats(mask: np.ndarray) -> dict:
    """Compute the same quality fields used by ``stain_mask`` for stored masks."""

    import cv2

    dirty = (mask == 0).astype(np.uint8)
    count, _, stats, _ = cv2.connectedComponentsWithStats(dirty, connectivity=8)
    components = max(0, count - 1)
    dirty_fraction = float(dirty.mean())
    return {
        "dirty_fraction": dirty_fraction,
        "components": components,
        "segment_scene": "simple" if components == 1 and dirty_fraction < 0.2 else "complex",
    }


def load_offline_input(
    path: str | Path, mask_config: MaskConfig | None = None
) -> tuple[np.ndarray, dict, np.ndarray | None]:
    """Load a mask or image, applying the deployment mask config to images."""

    import cv2

    source = Path(path).expanduser()
    image = None
    suffix = source.suffix.lower()
    if suffix == ".npy":
        mask = np.load(source, allow_pickle=False)
    elif suffix == ".npz":
        with np.load(source, allow_pickle=False) as archive:
            if "mask" not in archive:
                raise ValueError("Offline NPZ must contain a mask array")
            mask = archive["mask"]
    else:
        image = cv2.imread(str(source), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"Could not read offline image: {source}")
        mask, stats = stain_mask(image, mask_config or MaskConfig())
        return _validate_mask(mask), stats, image
    mask = _validate_mask(mask)
    return mask, _mask_stats(mask), image


def load_mask(path: str | Path, mask_config: MaskConfig | None = None) -> np.ndarray:
    """Load a binary mask; image inputs use the supplied deployment config."""

    return load_offline_input(path, mask_config)[0]


def _render_trajectory(actions: np.ndarray, trajectory: NpzPlanarTrajectory, safety: dict) -> np.ndarray:
    """Render a dependency-light XY report with the workspace and target sequence."""

    import cv2

    canvas = np.full((620, 900, 3), 255, dtype=np.uint8)
    workspace = safety["workspace"]
    bounds = np.array(
        [
            [float(workspace["x_min_m"]), float(workspace["y_min_m"])],
            [float(workspace["x_max_m"]), float(workspace["y_max_m"])],
        ]
    )
    margin = 0.04
    low = bounds[0] - margin
    high = bounds[1] + margin
    plot_origin = np.array([70, 560], dtype=float)
    plot_size = np.array([760, 480], dtype=float)

    def pixel(points):
        values = np.asarray(points, dtype=float)
        normalized = (values - low) / np.maximum(high - low, 1e-9)
        result = np.empty_like(normalized)
        result[:, 0] = plot_origin[0] + normalized[:, 0] * plot_size[0]
        result[:, 1] = plot_origin[1] - normalized[:, 1] * plot_size[1]
        return np.rint(result).astype(np.int32)

    workspace_px = pixel(bounds)
    cv2.rectangle(canvas, tuple(workspace_px[0]), tuple(workspace_px[1]), (80, 160, 80), 2)
    waypoint_tracking = isinstance(trajectory, PlanarWaypointTrajectory)
    times = (
        trajectory.times_s
        if waypoint_tracking
        else np.linspace(0.0, trajectory.duration_s, 129)
    )
    spline = np.stack([trajectory.sample(float(t)).position_base[:2] for t in times])
    spline_px = pixel(spline)
    action_px = pixel(actions[:, :2])
    cv2.polylines(canvas, [spline_px.reshape(-1, 1, 2)], False, (210, 80, 40), 3)
    for point in action_px:
        cv2.circle(canvas, tuple(point), 5, (40, 70, 210), -1)
    cv2.circle(canvas, tuple(action_px[0]), 8, (40, 170, 40), 2)
    cv2.circle(canvas, tuple(action_px[-1]), 8, (40, 40, 40), 2)
    path_label = "waypoint order" if waypoint_tracking else "Hermite path"
    cv2.putText(canvas, f"blue=policy points, orange={path_label}, green=workspace",
                (25, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (30, 30, 30), 1)
    cv2.putText(canvas, "x (m)", (820, 590), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (30, 30, 30), 1)
    cv2.putText(canvas, "y (m)", (15, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (30, 30, 30), 1)
    return canvas


def _write_offline_report(
    output_dir: str | Path,
    input_path: str | Path,
    image: np.ndarray | None,
    mask: np.ndarray,
    mask_stats: dict,
    actions: np.ndarray,
    trajectory: NpzPlanarTrajectory,
    action_report: dict,
    trajectory_report: dict,
    mapping: dict,
    seed: int,
) -> dict:
    import cv2

    destination = Path(output_dir).expanduser()
    destination.mkdir(parents=True, exist_ok=False)
    if image is not None and not cv2.imwrite(str(destination / "input.png"), image):
        raise OSError(f"Could not write {destination / 'input.png'}")
    if not cv2.imwrite(str(destination / "mask.png"), mask * 255):
        raise OSError(f"Could not write {destination / 'mask.png'}")
    if image is not None:
        preview = image.copy()
        preview[mask == 0] = (0.5 * preview[mask == 0] + [0, 0, 127]).astype(np.uint8)
        if not cv2.imwrite(str(destination / "mask_preview.png"), preview):
            raise OSError(f"Could not write {destination / 'mask_preview.png'}")
    trajectory_image = _render_trajectory(actions, trajectory, mapping["safety"])
    if not cv2.imwrite(str(destination / "trajectory.png"), trajectory_image):
        raise OSError(f"Could not write {destination / 'trajectory.png'}")
    trajectory_xy = _trajectory_xy_payload(actions, trajectory)
    (destination / "trajectory_xy.json").write_text(
        json.dumps(trajectory_xy, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    report = {
        "input": str(Path(input_path).expanduser().resolve()),
        "mask": mask_stats,
        "policy": {
            "checkpoint": str(Path(mapping["policy"]["checkpoint"]).expanduser()),
            "device": str(mapping["policy"].get("device", "cuda:0")),
            "seed": int(seed),
            "action_definition_version": ACTION_DEFINITION_VERSION,
        },
        "actions": actions.tolist(),
        "action_safety": action_report,
        "trajectory": {
            "duration_s": trajectory.duration_s,
            "tracking_mode": "waypoints" if isinstance(trajectory, PlanarWaypointTrajectory) else "interpolated",
            "execution_timing": "feedback" if isinstance(trajectory, PlanarWaypointTrajectory) else "time",
            "replay_yaw": bool(trajectory.replay_yaw),
            "first_position_base": trajectory.sample(0.0).position_base.tolist(),
            "last_position_base": trajectory.sample(trajectory.duration_s).position_base.tolist(),
            "workspace": trajectory_report,
        },
        "safe_to_execute": bool(action_report["ok"] and trajectory_report["ok"]),
        "artifacts": {
            "input": "input.png" if image is not None else None,
            "mask": "mask.png",
            "mask_preview": "mask_preview.png" if image is not None else None,
            "trajectory": "trajectory.png",
            "trajectory_xy": "trajectory_xy.json",
        },
    }
    (destination / "prediction.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return report


def validate_config(mapping: dict, config_path: Path) -> tuple[np.ndarray, float]:
    required = (
        "references",
        "mask_config",
        "policy",
        "plane",
        "trajectory",
        "safety",
        "gathering",
    )
    missing = [key for key in required if key not in mapping]
    if missing:
        raise ValueError(f"Missing required configuration sections: {missing}")
    camera = mapping.get("camera", {})
    CameraConfig.model_validate(camera)
    if int(camera.get("width", 640)) != 640 or int(camera.get("height", 480)) != 480:
        raise ValueError("Policy deployment requires the training camera resolution 640x480")
    if camera.get("roi") is not None:
        raise ValueError("Policy deployment requires the full 640x480 camera frame (roi=null)")
    clean_threshold = float(mapping["gathering"].get("clean_threshold", -1))
    if not 0.0 <= clean_threshold <= 1.0:
        raise ValueError("gathering.clean_threshold must be explicitly configured in [0, 1]")
    if int(mapping["gathering"].get("max_segments", 0)) <= 0:
        raise ValueError("gathering.max_segments must be explicitly configured")
    frame_timeout_s = float(mapping["gathering"].get("frame_timeout_s", 5.0))
    if frame_timeout_s <= 0 or not np.isfinite(frame_timeout_s):
        raise ValueError("gathering.frame_timeout_s must be positive and finite")
    safety = mapping["safety"]
    workspace = safety.get("workspace", {})
    for key in ("x_min_m", "x_max_m", "y_min_m", "y_max_m"):
        if key not in workspace:
            raise ValueError(f"safety.workspace.{key} must be configured")
    for key in ("max_action_step_m", "max_action_yaw_rad", "max_abs_yaw_rad"):
        if key not in safety:
            raise ValueError(f"safety.{key} must be configured as a positive number or null")
        if safety[key] is not None:
            try:
                value = float(safety[key])
            except (TypeError, ValueError) as exc:
                raise ValueError(f"safety.{key} must be a positive number or null") from exc
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"safety.{key} must be a positive number or null")
    policy = mapping["policy"]
    for key in ("checkpoint", "python", "root"):
        if not Path(policy[key]).expanduser().exists():
            raise FileNotFoundError(f"policy.{key} does not exist: {policy[key]}")
    capture, surface_z = load_references(mapping["references"], mapping)
    mask_cfg = load_json(mapping["mask_config"])
    MaskConfig.model_validate(mask_cfg)
    plane = mapping["plane"]
    if float(plane.get("duration_s", 0)) <= 0:
        raise ValueError("plane.duration_s must be positive")
    if str(plane.get("tool_normal_axis", "z")).strip().lower() not in {"x", "y", "z"}:
        raise ValueError("plane.tool_normal_axis must be x, y, or z")
    parse_runner_config(mapping)
    print(f"配置有效: {config_path}\n  references={mapping['references']}\n  surface_z_m={surface_z:.6f}\n  policy_checkpoint={policy['checkpoint']}")
    return capture, surface_z


def _create_live_run_directory(mapping: dict) -> Path:
    root = Path(mapping.get("logging", {}).get("directory", "data/push_wiper_policy_force"))
    root.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    candidate = root / f"live_{stamp}"
    suffix = 0
    while True:
        destination = candidate if suffix == 0 else root / f"live_{stamp}_{suffix:02d}"
        try:
            destination.mkdir()
            return destination
        except FileExistsError:
            suffix += 1


def _segment_mapping(mapping: dict, index: int, run_directory: str | Path | None = None) -> dict:
    result = copy.deepcopy(mapping)
    if run_directory is None:
        log_path = Path(mapping.get("logging", {}).get("directory", "data/push_wiper_policy_force"))
        log_path = log_path / f"segment_{index:04d}.csv"
    else:
        log_path = Path(run_directory) / f"segment_{index:04d}" / "force.csv"
    result.setdefault("logging", {})["path"] = str(log_path)
    return result


def move_to_observation(robot: AirbotPlayAdapter, capture: np.ndarray) -> None:
    position, orientation = capture[:3], capture[3:]
    if not robot.move_to_pose((position, orientation)):
        raise RuntimeError("AIRBOT rejected the return to observation pose O")


def _capture_observation(
    robot: AirbotPlayAdapter,
    camera: CameraWorker,
    capture: np.ndarray,
    settle_s: float,
    frame_timeout_s: float,
) -> dict:
    """Move to O, settle, then return a frame acquired after settling."""

    move_to_observation(robot, capture)
    if settle_s > 0:
        time.sleep(settle_s)
    after_ns = acquisition_ns()
    return camera.wait_for_frame_after(after_ns, timeout_s=frame_timeout_s)


def _write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def _write_observation_artifacts(
    destination: str | Path,
    frame: np.ndarray,
    mask: np.ndarray,
    mask_stats: dict,
    frame_t_ns: int,
    status: str,
) -> dict:
    """Save an observation even when no policy segment is executed."""

    import cv2

    destination = Path(destination).expanduser()
    destination.mkdir(parents=True, exist_ok=False)
    if not cv2.imwrite(str(destination / "input.png"), frame):
        raise OSError(f"Could not write {destination / 'input.png'}")
    if not cv2.imwrite(str(destination / "mask.png"), mask * 255):
        raise OSError(f"Could not write {destination / 'mask.png'}")
    preview = frame.copy()
    preview[mask == 0] = (0.5 * preview[mask == 0] + [0, 0, 127]).astype(np.uint8)
    if not cv2.imwrite(str(destination / "mask_preview.png"), preview):
        raise OSError(f"Could not write {destination / 'mask_preview.png'}")
    report = {
        "status": status,
        "frame_t_ns": int(frame_t_ns),
        "mask": mask_stats,
        "artifacts": {
            "input": "input.png",
            "mask": "mask.png",
            "mask_preview": "mask_preview.png",
        },
    }
    _write_json(destination / "observation.json", report)
    return report


def _build_policy_trajectory(
    actions: np.ndarray,
    capture: np.ndarray,
    surface_z: float,
    mapping: dict,
) -> PlanarWaypointTrajectory:
    """Keep all 16 policy XY targets for feedback-driven tracking without interpolation."""

    plane = mapping["plane"]
    replay_yaw = bool(plane.get("replay_yaw", True))
    trajectory_actions = actions if replay_yaw else np.asarray(actions, dtype=float)[:, :2]
    return PlanarWaypointTrajectory.from_actions(
        trajectory_actions,
        capture,
        float(plane["duration_s"]),
        surface_z,
        replay_yaw=replay_yaw,
        align_tool_z=bool(plane.get("align_tool_z", True)),
        tool_normal_axis=str(plane.get("tool_normal_axis", "z")),
    )


def _trajectory_xy_payload(
    actions: np.ndarray,
    trajectory: NpzPlanarTrajectory,
    source_prediction: str | None = None,
    sample_count: int = 121,
) -> dict:
    """Serialize only the horizontal XY trajectory, without pose yaw."""

    values = np.asarray(actions, dtype=float)
    if values.shape not in {(16, 2), (16, 3)}:
        raise ValueError("Trajectory actions must have shape (16, 2) or (16, 3)")
    if sample_count < 2:
        raise ValueError("sample_count must be at least two")
    control_xy = values[:, :2]
    control_times = np.asarray(trajectory.times_s, dtype=float)
    waypoint_tracking = isinstance(trajectory, PlanarWaypointTrajectory)
    sample_times = (
        control_times
        if waypoint_tracking
        else np.linspace(0.0, trajectory.duration_s, sample_count)
    )
    samples = np.stack(
        [trajectory.sample(float(time_s)).position_base[:2] for time_s in sample_times]
    )
    payload = {
        "dimensions": ["x_m", "y_m"],
        "duration_s": float(trajectory.duration_s),
        "tracking_mode": "waypoints" if waypoint_tracking else "interpolated",
        "execution_timing": "feedback" if waypoint_tracking else "time",
        "replay_yaw": bool(trajectory.replay_yaw),
        "control_points": [
            {"t_s": float(time_s), "x_m": float(point[0]), "y_m": float(point[1])}
            for time_s, point in zip(control_times, control_xy)
        ],
        "samples": [
            {"t_s": float(time_s), "x_m": float(point[0]), "y_m": float(point[1])}
            for time_s, point in zip(sample_times, samples)
        ],
    }
    if trajectory.capture_reference_pose is not None:
        payload["capture_reference_pose"] = trajectory.capture_reference_pose.tolist()
    if trajectory.surface_z_m is not None:
        payload["surface_z_m"] = float(trajectory.surface_z_m)
    if source_prediction is not None:
        payload["source_prediction"] = str(Path(source_prediction).expanduser().resolve())
    return payload


def run_offline(mapping: dict, capture: np.ndarray, surface_z: float, mask_path: str | Path) -> int:
    mask_cfg = MaskConfig.model_validate(load_json(mapping["mask_config"]))
    mask = load_mask(mask_path, mask_cfg)
    policy = mapping["policy"]
    with PolicyClient(
        mapping["policy"]["checkpoint"], policy["python"], policy["root"],
        device=policy.get("device", "cuda:0"),
        startup_timeout_s=float(policy.get("startup_timeout_s", 120.0)),
        request_timeout_s=float(policy.get("request_timeout_s", 120.0)),
    ) as client:
        actions = validate_actions(client.predict(mask, capture, seed=int(policy.get("seed", 42))), mapping["safety"])
    trajectory = _build_policy_trajectory(actions, capture, surface_z, mapping)
    validate_trajectory_workspace(trajectory, mapping["safety"])
    print(json.dumps({"dirty_fraction": float(1.0 - mask.mean()), "actions": actions.tolist(), "first": trajectory.sample(0).position_base.tolist(), "last": trajectory.sample(trajectory.duration_s).position_base.tolist()}, ensure_ascii=False))
    return 0


def run_offline_report(
    mapping: dict,
    capture: np.ndarray,
    surface_z: float,
    image_path: str | Path,
    output_dir: str | Path,
    seed: int | None = None,
) -> int:
    """Predict one image and save all artifacts, including unsafe predictions."""

    mask_cfg = MaskConfig.model_validate(load_json(mapping["mask_config"]))
    mask, mask_stats, image = load_offline_input(image_path, mask_cfg)
    policy_cfg = mapping["policy"]
    prediction_seed = int(policy_cfg.get("seed", 42) if seed is None else seed)
    with PolicyClient(
        policy_cfg["checkpoint"],
        policy_cfg["python"],
        policy_cfg["root"],
        device=policy_cfg.get("device", "cuda:0"),
        startup_timeout_s=float(policy_cfg.get("startup_timeout_s", 120.0)),
        request_timeout_s=float(policy_cfg.get("request_timeout_s", 120.0)),
    ) as client:
        actions = client.predict(
            mask,
            capture,
            seed=prediction_seed,
        )
    action_report = inspect_planar_actions(actions, mapping["safety"])
    trajectory = _build_policy_trajectory(actions, capture, surface_z, mapping)
    trajectory_report = inspect_trajectory_workspace(trajectory, mapping["safety"])
    report = _write_offline_report(
        output_dir,
        image_path,
        image,
        mask,
        mask_stats,
        actions,
        trajectory,
        action_report,
        trajectory_report,
        mapping,
        prediction_seed,
    )
    print(json.dumps({
        "output_dir": str(Path(output_dir).expanduser().resolve()),
        "dirty_fraction": mask_stats["dirty_fraction"],
        "safe_to_execute": report["safe_to_execute"],
        "action_violations": action_report["violations"],
        "trajectory_violations": trajectory_report["violations"],
    }, ensure_ascii=False, indent=2))
    return 0 if report["safe_to_execute"] else 2


def run_offline_prediction(
    mapping: dict,
    capture: np.ndarray,
    surface_z: float,
    prediction_path: str | Path,
    output_dir: str | Path,
) -> int:
    """Construct the configured XY-only trajectory from saved DP actions."""

    source = load_json(prediction_path)
    actions = np.asarray(source.get("actions"), dtype=np.float32)
    if actions.shape != (16, 3) or not np.isfinite(actions).all():
        raise ValueError("prediction.json must contain finite actions with shape (16, 3)")
    trajectory = _build_policy_trajectory(actions, capture, surface_z, mapping)
    action_report = inspect_planar_actions(actions, mapping["safety"])
    trajectory_report = inspect_trajectory_workspace(trajectory, mapping["safety"])
    destination = Path(output_dir).expanduser()
    destination.mkdir(parents=True, exist_ok=False)
    trajectory_xy = _trajectory_xy_payload(
        actions,
        trajectory,
        source_prediction=prediction_path,
    )
    trajectory_xy.update(
        {
            "workspace": trajectory_report,
            "safe_to_execute": bool(action_report["ok"] and trajectory_report["ok"]),
        }
    )
    (destination / "trajectory_xy.json").write_text(
        json.dumps(trajectory_xy, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    import cv2

    trajectory_image = _render_trajectory(actions, trajectory, mapping["safety"])
    if not cv2.imwrite(str(destination / "trajectory.png"), trajectory_image):
        raise OSError(f"Could not write {destination / 'trajectory.png'}")
    print(json.dumps({
        "output_dir": str(destination.resolve()),
        "duration_s": trajectory.duration_s,
        "tracking_mode": trajectory_xy["tracking_mode"],
        "execution_timing": trajectory_xy["execution_timing"],
        "replay_yaw": trajectory.replay_yaw,
        "safe_to_execute": trajectory_xy["safe_to_execute"],
        "action_violations": action_report["violations"],
        "trajectory_violations": trajectory_report["violations"],
    }, ensure_ascii=False, indent=2))
    return 0 if trajectory_xy["safe_to_execute"] else 2


def run_live(mapping: dict, capture: np.ndarray, surface_z: float) -> int:
    camera_cfg = CameraConfig.model_validate(mapping["camera"])
    mask_cfg = MaskConfig.model_validate(load_json(mapping["mask_config"]))
    robot_cfg, sensor_cfg = mapping["robot"], mapping["sensor"]
    robot = AirbotPlayAdapter(robot_cfg.get("url", "localhost"), int(robot_cfg.get("port", 50051)))
    reader = make_force_reader(sensor_cfg)
    camera = CameraWorker(camera_cfg)
    policy_cfg = mapping["policy"]
    policy = PolicyClient(policy_cfg["checkpoint"], policy_cfg["python"], policy_cfg["root"], device=policy_cfg.get("device", "cuda:0"), startup_timeout_s=float(policy_cfg.get("startup_timeout_s", 120.0)), request_timeout_s=float(policy_cfg.get("request_timeout_s", 120.0)))
    run_directory = _create_live_run_directory(mapping)
    LOGGER.info("Live run artifacts will be written to %s", run_directory)
    segment_reports: list[dict] = []
    run_status = "STARTING"
    run_error = None
    try:
        robot.connect()
        camera.start()
        policy.start()
        clean_threshold = float(mapping["gathering"]["clean_threshold"])
        max_segments = int(mapping["gathering"]["max_segments"])
        settle_s = float(mapping["gathering"].get("observation_settle_s", 0.5))
        frame_timeout_s = float(mapping["gathering"].get("frame_timeout_s", 5.0))
        run_status = "RUNNING"
        for index in range(max_segments):
            segment_directory = run_directory / f"segment_{index:04d}"
            frame_record = _capture_observation(
                robot,
                camera,
                capture,
                settle_s=settle_s,
                frame_timeout_s=frame_timeout_s,
            )
            frame = frame_record["data"]
            mask, stats = stain_mask(frame, mask_cfg)
            dirty_fraction = float(stats["dirty_fraction"])
            LOGGER.info(
                "segment=%d dirty_fraction=%.5f frame_t_ns=%d",
                index,
                dirty_fraction,
                frame_record["t"],
            )
            if dirty_fraction <= clean_threshold:
                LOGGER.info("clean threshold reached before segment %d", index)
                observation_report = _write_observation_artifacts(
                    segment_directory,
                    frame,
                    mask,
                    stats,
                    frame_record["t"],
                    "CLEAN_THRESHOLD_REACHED",
                )
                observation_report["dirty_fraction"] = dirty_fraction
                _write_json(segment_directory / "observation.json", observation_report)
                segment_reports.append(
                    {
                        "index": index,
                        "status": "CLEAN_THRESHOLD_REACHED",
                        "dirty_fraction": dirty_fraction,
                        "directory": str(segment_directory),
                    }
                )
                run_status = "CLEAN"
                break

            try:
                actions = policy.predict(
                    mask,
                    seed=int(policy_cfg.get("seed", 42)) + index,
                    capture_reference_pose=capture,
                )
                action_report = inspect_planar_actions(actions, mapping["safety"])
                trajectory = _build_policy_trajectory(actions, capture, surface_z, mapping)
                trajectory_report = inspect_trajectory_workspace(
                    trajectory, mapping["safety"]
                )
            except Exception as exc:
                observation_report = _write_observation_artifacts(
                    segment_directory,
                    frame,
                    mask,
                    stats,
                    frame_record["t"],
                    "PREDICTION_FAULT",
                )
                observation_report.update(
                    {
                        "dirty_fraction": dirty_fraction,
                        "error": str(exc),
                    }
                )
                _write_json(segment_directory / "observation.json", observation_report)
                segment_reports.append(
                    {
                        "index": index,
                        "status": "PREDICTION_FAULT",
                        "dirty_fraction": dirty_fraction,
                        "error": str(exc),
                        "directory": str(segment_directory),
                    }
                )
                raise

            report = _write_offline_report(
                segment_directory,
                segment_directory / "input.png",
                frame,
                mask,
                stats,
                actions,
                trajectory,
                action_report,
                trajectory_report,
                mapping,
                int(policy_cfg.get("seed", 42)) + index,
            )
            report.update(
                {
                    "mode": "live",
                    "status": "PREDICTED",
                    "segment_index": index,
                    "frame_t_ns": int(frame_record["t"]),
                    "dirty_fraction": dirty_fraction,
                    "artifacts": {
                        **report["artifacts"],
                        "force_log": "force.csv",
                    },
                }
            )
            _write_json(segment_directory / "prediction.json", report)

            if not report["safe_to_execute"]:
                report.update(
                    {
                        "status": "REJECTED_SAFETY",
                        "controller_state": None,
                        "controller_error": None,
                    }
                )
                _write_json(segment_directory / "prediction.json", report)
                segment_reports.append(
                    {
                        "index": index,
                        "status": "REJECTED_SAFETY",
                        "dirty_fraction": dirty_fraction,
                        "directory": str(segment_directory),
                        "violations": report["action_safety"]["violations"]
                        + report["trajectory"]["workspace"]["violations"],
                    }
                )
                LOGGER.error(
                    "segment=%d rejected by safety checks: %s",
                    index,
                    segment_reports[-1]["violations"],
                )
                run_status = "SAFETY_REJECTED"
                return 2

            runner_config = parse_runner_config(
                _segment_mapping(mapping, index, run_directory)
            )
            runner = HybridRunner(robot, reader, trajectory, runner_config)
            final_state = runner.run(close_resources=False)
            controller_error = runner.error or None
            report.update(
                {
                    "status": "DONE" if final_state == "DONE" else "FAULT",
                    "controller_state": final_state,
                    "controller_error": controller_error,
                }
            )
            _write_json(segment_directory / "prediction.json", report)
            segment_reports.append(
                {
                    "index": index,
                    "status": report["status"],
                    "dirty_fraction": dirty_fraction,
                    "directory": str(segment_directory),
                    "controller_error": controller_error,
                }
            )
            if final_state != "DONE":
                LOGGER.error(
                    "segment %d ended in %s: %s",
                    index,
                    final_state,
                    "force controller fault",
                )
                run_status = "FAULT"
                run_error = controller_error
                return 2
            try:
                move_to_observation(robot, capture)
            except Exception as exc:
                report.update(
                    {
                        "status": "OBSERVATION_RETURN_FAULT",
                        "controller_error": str(exc),
                    }
                )
                _write_json(segment_directory / "prediction.json", report)
                segment_reports[-1].update(
                    {"status": "OBSERVATION_RETURN_FAULT", "controller_error": str(exc)}
                )
                run_status = "FAULT"
                run_error = str(exc)
                return 2
            if settle_s > 0:
                time.sleep(settle_s)
        else:
            LOGGER.warning("max_segments=%d reached before clean threshold", max_segments)
            run_status = "MAX_SEGMENTS"
        if run_status == "RUNNING":
            run_status = "MAX_SEGMENTS"
        return 0
    except Exception as exc:
        run_status = "FAULT"
        run_error = str(exc)
        LOGGER.exception("Online policy force loop failed: %s", exc)
        return 2
    finally:
        _write_json(
            run_directory / "run_summary.json",
            {
                "directory": str(run_directory),
                "status": run_status,
                "error": run_error,
                "segments": segment_reports,
                "max_segments": int(mapping["gathering"]["max_segments"]),
                "clean_threshold": float(mapping["gathering"]["clean_threshold"]),
            },
        )
        policy.close()
        camera.close()
        try:
            robot.stop()
        finally:
            robot.close()
            reader.close()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--validate", action="store_true")
    offline = parser.add_mutually_exclusive_group()
    offline.add_argument(
        "--dry-run-mask",
        help="Run model and trajectory validation without camera, robot, or force hardware",
    )
    offline.add_argument(
        "--offline-image",
        help="Predict one image and save a mask/trajectory safety report without hardware",
    )
    offline.add_argument(
        "--offline-prediction",
        help="Construct the configured XY trajectory from an existing prediction.json",
    )
    parser.add_argument(
        "--offline-output-dir",
        help="Output directory for offline reports; it must not already exist",
    )
    parser.add_argument("--seed", type=int, help="Override policy seed for offline inference")
    parser.add_argument("--device", help="Override policy.device (use cpu for an offline smoke test)")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    config_path = Path(args.config).expanduser().resolve()
    mapping = resolve_config_paths(load_json(config_path), config_path.parent)
    if args.device:
        mapping.setdefault("policy", {})["device"] = args.device
    try:
        capture, surface_z = validate_config(mapping, config_path)
        if args.validate:
            return 0
        if args.dry_run_mask:
            return run_offline(mapping, capture, surface_z, args.dry_run_mask)
        if args.offline_prediction:
            prediction_path = Path(args.offline_prediction).expanduser()
            output_dir = args.offline_output_dir
            if output_dir is None:
                output_dir = prediction_path.with_name(prediction_path.stem + "_xy_trajectory")
            return run_offline_prediction(
                mapping,
                capture,
                surface_z,
                prediction_path,
                output_dir,
            )
        if args.offline_image:
            output_dir = args.offline_output_dir
            if output_dir is None:
                image_path = Path(args.offline_image).expanduser()
                output_dir = image_path.with_name(image_path.stem + "_dp_prediction")
            return run_offline_report(
                mapping,
                capture,
                surface_z,
                args.offline_image,
                output_dir,
                seed=args.seed,
            )
        return run_live(mapping, capture, surface_z)
    except KeyboardInterrupt:
        LOGGER.warning("Policy force loop cancelled by operator")
        return 130
    except Exception as exc:
        LOGGER.error("Policy force loop failed: %s", exc, exc_info=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
