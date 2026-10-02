import numpy as np
import pytest
import cv2
import json
from types import SimpleNamespace
from threading import Lock

from airbot_ie.force_control.core import NpzPlanarTrajectory, PlanarWaypointTrajectory
from airbot_ie.force_control.policy_client import (
    encode_mask,
    inspect_planar_actions,
    validate_planar_actions,
)
from airbot_ie.push_wiper.export import MaskConfig, MaskRule
from airbot_ie.push_wiper.clock import acquisition_ns
from airbot_ie.push_wiper.devices import CameraWorker
from airbot_ie.scripts.push_wiper_policy_force import (
    _build_policy_trajectory,
    _write_observation_artifacts,
    load_offline_input,
)
import airbot_ie.scripts.push_wiper_policy_force as policy_script


def _safety():
    return {
        "workspace": {"x_min_m": 0.0, "x_max_m": 1.0, "y_min_m": -1.0, "y_max_m": 1.0},
        "max_action_step_m": 0.2,
        "max_action_yaw_rad": 0.2,
        "max_abs_yaw_rad": 1.0,
    }


def test_mask_encoding_rejects_wrong_shape_and_accepts_binary():
    mask = np.ones((480, 640), dtype=np.uint8)
    mask[10, 10] = 0
    encoded = encode_mask(mask)
    assert isinstance(encoded, str) and encoded
    with pytest.raises(ValueError, match="shape"):
        encode_mask(np.zeros((10, 10), dtype=np.uint8))


def test_action_safety_rejects_workspace_and_yaw_step_without_clipping():
    actions = np.zeros((16, 3), dtype=np.float32)
    actions[:, 0] = 0.5
    actions[8, 2] = 1.1
    with pytest.raises(ValueError, match="absolute"):
        validate_planar_actions(actions, _safety())
    actions[8, 2] = 0.0
    actions[9, 0] = 0.8
    with pytest.raises(ValueError, match="step"):
        validate_planar_actions(actions, _safety())


def test_action_safety_report_preserves_all_violations_for_offline_review():
    actions = np.zeros((16, 3), dtype=np.float32)
    actions[:, 0] = 1.2
    actions[1, :2] = [0.5, 0.5]
    actions[:, 2] = 1.5
    report = inspect_planar_actions(actions, _safety())
    assert report["ok"] is False
    assert any("workspace" in value for value in report["violations"])
    assert any("XY step" in value for value in report["violations"])
    assert any("absolute" in value for value in report["violations"])
    assert report["metrics"]["delta_yaw_abs_max_rad"] == pytest.approx(1.5)


def test_action_step_and_yaw_limits_can_be_disabled_without_disabling_workspace():
    safety = _safety()
    safety.update(
        max_action_step_m=None,
        max_action_yaw_rad=None,
        max_abs_yaw_rad=None,
    )
    actions = np.zeros((16, 3), dtype=np.float32)
    actions[:, 0] = np.linspace(0.2, 0.8, 16)
    actions[:, 2] = np.linspace(-2.5, 2.5, 16)
    report = inspect_planar_actions(actions, safety)
    assert report["ok"] is True
    assert report["limits"]["max_action_step_m"] is None
    assert report["limits"]["max_action_yaw_rad"] is None
    assert report["limits"]["max_abs_yaw_rad"] is None
    assert np.allclose(validate_planar_actions(actions, safety), actions)


def test_offline_image_uses_the_supplied_mask_config(tmp_path):
    image = np.full((480, 640, 3), 255, dtype=np.uint8)
    cv2.rectangle(image, (100, 100), (120, 120), (0, 0, 0), -1)
    image_path = tmp_path / "before.png"
    assert cv2.imwrite(str(image_path), image)
    config = MaskConfig(
        rules=[MaskRule(space="gray", lower=[0], upper=[10])],
        morphology_kernel=1,
        min_component_area=10,
    )
    mask, stats, loaded = load_offline_input(image_path, config)
    assert loaded is not None and loaded.shape == image.shape
    assert mask[110, 110] == 0
    assert stats["components"] == 1
    assert stats["dirty_fraction"] > 0.0


def test_policy_actions_build_planar_trajectory_with_reference_yaw():
    actions = np.zeros((16, 3), dtype=np.float32)
    actions[:, 0] = np.linspace(0.2, 0.3, 16)
    actions[:, 2] = np.linspace(0.0, 0.1, 16)
    capture = np.array([0.1, 0.0, 0.4, 0.0, 0.0, np.sin(0.2), np.cos(0.2)])
    trajectory = NpzPlanarTrajectory.from_actions(actions, capture, 2.0, 0.1)
    first = trajectory.sample(0.0)
    last = trajectory.sample(2.0)
    assert np.allclose(first.position_base[:2], actions[0, :2])
    assert np.allclose(last.position_base[:2], actions[-1, :2])
    assert np.allclose(last.normal_base, [0.0, 0.0, 1.0])
    # A 0.1 rad action relative to the O yaw must affect the final orientation.
    assert not np.allclose(first.orientation_xyzw, last.orientation_xyzw)


def test_xy_only_policy_trajectory_uses_thirty_seconds_and_fixed_orientation():
    actions = np.zeros((16, 3), dtype=np.float32)
    actions[:, 0] = np.linspace(0.2, 0.3, 16)
    actions[:, 1] = np.linspace(-0.1, 0.1, 16)
    actions[:, 2] = np.linspace(-2.0, 2.0, 16)
    capture = np.array([0.1, 0.0, 0.4, 0.0, 0.0, np.sin(0.2), np.cos(0.2)])
    trajectory = NpzPlanarTrajectory.from_actions(
        actions[:, :2], capture, 30.0, 0.1, replay_yaw=False
    )
    assert trajectory.duration_s == pytest.approx(30.0)
    assert np.allclose(
        trajectory.sample(0.0).orientation_xyzw,
        trajectory.sample(30.0).orientation_xyzw,
    )
    assert np.allclose(trajectory.sample(30.0).position_base[:2], actions[-1, :2])


def test_camera_wait_for_frame_after_rejects_old_frame():
    worker = CameraWorker.__new__(CameraWorker)
    worker.config = SimpleNamespace(stale_s=1.0)
    worker.lock = Lock()
    worker.error = None
    worker.frame = {"data": np.zeros((480, 640, 3), dtype=np.uint8), "t": acquisition_ns()}
    with pytest.raises(TimeoutError):
        worker.wait_for_frame_after(worker.frame["t"], timeout_s=0.01)


def test_camera_wait_for_frame_after_returns_new_frame():
    worker = CameraWorker.__new__(CameraWorker)
    worker.config = SimpleNamespace(stale_s=1.0)
    worker.lock = Lock()
    worker.error = None
    old_t = acquisition_ns()
    worker.frame = {"data": np.zeros((480, 640, 3), dtype=np.uint8), "t": old_t + 1}
    frame = worker.wait_for_frame_after(old_t, timeout_s=0.1)
    assert frame["t"] == old_t + 1
    assert frame["data"].shape == (480, 640, 3)


def test_policy_trajectory_passes_tool_normal_axis_and_drops_yaw():
    actions = np.zeros((16, 3), dtype=np.float32)
    actions[:, 0] = np.linspace(0.2, 0.3, 16)
    actions[:, 1] = np.linspace(-0.1, 0.1, 16)
    actions[:, 2] = np.linspace(-2.0, 2.0, 16)
    capture = np.array([0.1, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0])
    mapping = {
        "plane": {
            "duration_s": 30.0,
            "replay_yaw": False,
            "align_tool_z": True,
            "tool_normal_axis": "x",
        }
    }
    trajectory = _build_policy_trajectory(actions, capture, 0.1, mapping)
    assert isinstance(trajectory, PlanarWaypointTrajectory)
    assert trajectory.waypoint_count == 16
    for index in range(16):
        np.testing.assert_array_equal(trajectory.sample_waypoint(index).position_base[:2], actions[index, :2])
    assert trajectory.duration_s == pytest.approx(30.0)
    assert trajectory.tool_normal_axis == "x"
    assert np.allclose(
        trajectory.sample(0.0).orientation_xyzw,
        trajectory.sample(30.0).orientation_xyzw,
    )


def test_policy_waypoint_payload_contains_no_intermediate_samples():
    actions = np.zeros((16, 3), dtype=np.float32)
    actions[:, 0] = np.linspace(0.2, 0.3, 16)
    actions[:, 1] = np.linspace(-0.1, 0.1, 16)
    capture = np.array([0.1, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0])
    trajectory = _build_policy_trajectory(
        actions, capture, 0.1, {"plane": {"duration_s": 30.0, "replay_yaw": False}}
    )
    payload = policy_script._trajectory_xy_payload(actions, trajectory, sample_count=1000)
    assert payload["tracking_mode"] == "waypoints"
    assert payload["execution_timing"] == "feedback"
    assert len(payload["samples"]) == 16
    assert payload["samples"] == payload["control_points"]


def test_waypoint_workspace_report_checks_every_original_target():
    actions = np.zeros((16, 3), dtype=np.float32)
    actions[:, 0] = 0.3
    actions[7, 0] = 1.1
    capture = np.array([0.1, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0])
    trajectory = _build_policy_trajectory(
        actions, capture, 0.1, {"plane": {"duration_s": 30.0, "replay_yaw": False}}
    )
    report = policy_script.inspect_trajectory_workspace(trajectory, _safety())
    assert report["ok"] is False
    assert report["metrics"]["x_max_m"] == pytest.approx(1.1)


def test_observation_artifacts_write_input_mask_preview_and_report(tmp_path):
    frame = np.full((480, 640, 3), 255, dtype=np.uint8)
    mask = np.ones((480, 640), dtype=np.uint8)
    mask[20:30, 30:40] = 0
    destination = tmp_path / "segment_0000"
    report = _write_observation_artifacts(
        destination,
        frame,
        mask,
        {"dirty_fraction": 100 / (480 * 640)},
        123,
        "CLEAN_THRESHOLD_REACHED",
    )
    assert report["status"] == "CLEAN_THRESHOLD_REACHED"
    assert (destination / "input.png").is_file()
    assert (destination / "mask.png").is_file()
    assert (destination / "mask_preview.png").is_file()
    assert (destination / "observation.json").is_file()


def test_live_loop_writes_segment_artifacts_and_runs_one_segment(tmp_path, monkeypatch):
    config_path = (
        __import__("pathlib").Path(__file__).parents[1]
        / "airbot_ie/configs/push_wiper_policy_force.json"
    )
    mapping = json.loads(config_path.read_text(encoding="utf-8"))
    mapping["mask_config"] = str(
        config_path.parents[2]
        / "data/push_wiper_export_assisted_complete_20260922/mask_config.json"
    )
    mapping["logging"] = {"directory": str(tmp_path / "runs")}
    mapping["gathering"].update(
        {"max_segments": 1, "observation_settle_s": 0.0, "frame_timeout_s": 0.1}
    )
    mapping["policy"].update(
        {"checkpoint": "unused.ckpt", "python": "python", "root": "."}
    )
    capture = np.array([0.1, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0])
    frame = np.full((480, 640, 3), 255, dtype=np.uint8)
    mask = np.ones((480, 640), dtype=np.uint8)
    mask[20:30, 30:40] = 0

    class FakeRobot:
        def __init__(self, *args, **kwargs):
            pass

        def connect(self):
            pass

        def move_to_pose(self, pose):
            return True

        def stop(self):
            pass

        def close(self):
            pass

    class FakeReader:
        def close(self):
            pass

    class FakeCamera:
        def __init__(self, config):
            pass

        def start(self):
            pass

        def wait_for_frame_after(self, timestamp_ns, timeout_s):
            return {"data": frame.copy(), "t": int(timestamp_ns) + 1}

        def close(self):
            pass

    class FakePolicy:
        def __init__(self, *args, **kwargs):
            pass

        def start(self):
            pass

        def predict(self, *args, **kwargs):
            actions = np.zeros((16, 3), dtype=np.float32)
            actions[:, 0] = np.linspace(0.2, 0.3, 16)
            actions[:, 1] = np.linspace(-0.1, 0.1, 16)
            return actions

        def close(self):
            pass

    class FakeRunner:
        def __init__(self, *args, **kwargs):
            assert isinstance(args[2], PlanarWaypointTrajectory)
            self.error = ""

        def run(self, close_resources=False):
            return "DONE"

    monkeypatch.setattr(policy_script, "AirbotPlayAdapter", FakeRobot)
    monkeypatch.setattr(policy_script, "make_force_reader", lambda config: FakeReader())
    monkeypatch.setattr(policy_script, "CameraWorker", FakeCamera)
    monkeypatch.setattr(policy_script, "PolicyClient", FakePolicy)
    monkeypatch.setattr(policy_script, "HybridRunner", FakeRunner)
    monkeypatch.setattr(
        policy_script,
        "stain_mask",
        lambda image, config: (mask.copy(), {"dirty_fraction": 0.01}),
    )
    monkeypatch.setattr(policy_script.time, "sleep", lambda seconds: None)

    assert policy_script.run_live(mapping, capture, 0.1) == 0
    run_directories = list((tmp_path / "runs").glob("live_*"))
    assert len(run_directories) == 1
    segment = run_directories[0] / "segment_0000"
    for name in (
        "input.png",
        "mask.png",
        "mask_preview.png",
        "prediction.json",
        "trajectory_xy.json",
        "trajectory.png",
    ):
        assert (segment / name).is_file(), name
    prediction = json.loads((segment / "prediction.json").read_text(encoding="utf-8"))
    assert prediction["status"] == "DONE"
    assert prediction["controller_state"] == "DONE"
    assert prediction["trajectory"]["replay_yaw"] is False
    assert prediction["trajectory"]["tracking_mode"] == "waypoints"
    assert prediction["trajectory"]["execution_timing"] == "feedback"
    trajectory_xy = json.loads((segment / "trajectory_xy.json").read_text(encoding="utf-8"))
    assert trajectory_xy["tracking_mode"] == "waypoints"
    assert len(trajectory_xy["samples"]) == 16
    assert trajectory_xy["samples"] == trajectory_xy["control_points"]
    summary = json.loads(
        (run_directories[0] / "run_summary.json").read_text(encoding="utf-8")
    )
    assert summary["status"] == "MAX_SEGMENTS"
