import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from airbot_ie.force_control.core import NpzPlanarTrajectory
from airbot_ie.force_control.runner import parse_runner_config


class NpzCapturePoseTests(unittest.TestCase):
    def test_npz_keeps_capture_reference_pose_for_prepositioning(self):
        capture = np.array([0.1, 0.2, 0.4, 0.0, 0.0, 0.0, 1.0])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sample.npz"
            np.savez(
                path,
                actions=np.array([[0.2, 0.1, 0.0], [0.21, 0.11, 0.1]]),
                capture_reference_pose=capture,
                work_height_m=np.float64(0.1),
            )
            trajectory = NpzPlanarTrajectory.from_npz(path, 2.0)
        np.testing.assert_allclose(trajectory.capture_reference_pose, capture)

    def test_runner_config_reads_capture_pose_switch(self):
        config = parse_runner_config(
            {
                "trajectory": {
                    "mode": "npz",
                    "move_to_capture_pose": True,
                    "staged_move_to_trajectory": True,
                    "orientation_transition_s": 1.5,
                    "safe_descent_s": 2.0,
                    "high_z_m": 0.3,
                },
                "sensor": {"rezero_after_preposition": True},
            }
        )
        self.assertTrue(config.move_to_capture_pose)
        self.assertTrue(config.staged_move_to_trajectory)
        self.assertAlmostEqual(config.orientation_transition_s, 1.5)
        self.assertAlmostEqual(config.safe_descent_s, 2.0)
        self.assertAlmostEqual(config.high_z_m, 0.3)
        self.assertTrue(config.rezero_after_preposition)

    def test_xy_json_loads_time_parameterized_points_without_yaw(self):
        capture = np.array([0.1, 0.2, 0.4, 0.0, 0.0, 0.0, 1.0])
        payload = {
            "dimensions": ["x_m", "y_m"],
            "duration_s": 30.0,
            "capture_reference_pose": capture.tolist(),
            "surface_z_m": 0.12,
            "control_points": [
                {"t_s": 0.0, "x_m": 0.2, "y_m": -0.1},
                {"t_s": 15.0, "x_m": 0.25, "y_m": 0.0},
                {"t_s": 30.0, "x_m": 0.3, "y_m": 0.1},
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trajectory_xy.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            trajectory = NpzPlanarTrajectory.from_xy_json(path)
        self.assertAlmostEqual(trajectory.duration_s, 30.0)
        self.assertFalse(trajectory.replay_yaw)
        np.testing.assert_allclose(trajectory.capture_reference_pose, capture)
        self.assertAlmostEqual(trajectory.surface_z_m, 0.12)
        np.testing.assert_allclose(
            trajectory.sample(0.0).position_base[:2], [0.2, -0.1]
        )
        np.testing.assert_allclose(
            trajectory.sample(30.0).position_base[:2], [0.3, 0.1]
        )
        np.testing.assert_allclose(
            trajectory.sample(0.0).orientation_xyzw,
            trajectory.sample(30.0).orientation_xyzw,
        )


if __name__ == "__main__":
    unittest.main()
