import csv
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np

from airbot_ie.force_control.core import PlanarWaypointTrajectory
from airbot_ie.force_control.hardware import CsvLogger, WrenchSample
from airbot_ie.force_control.runner import HybridRunner, RunnerConfig, parse_runner_config


class WaypointClock:
    def __init__(self, sleep_jitter=0.0):
        self.now = 100.0
        self.sleep_jitter = sleep_jitter
        self.state = lambda: "PRECHECK"

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration
        if duration > 0 and self.state() == "FORCE_HOLD":
            self.now += self.sleep_jitter


class ForceHybridWaypointTests(unittest.TestCase):
    def run_waypoints(
        self,
        *,
        stalled_index=None,
        force_fault_index=None,
        stale_index=None,
        rejected_index=None,
        sleep_jitter=0.0,
        frozen_z_feedback=False,
    ):
        clock = WaypointClock(sleep_jitter)
        actions = np.zeros((16, 2))
        actions[:, 0] = np.linspace(0.2, 0.35, 16)
        actions[:, 1] = 0.005 * np.sin(np.arange(16))
        capture = np.array([0.2, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0])
        trajectory = PlanarWaypointTrajectory.from_actions(
            actions, capture, 0.005, 0.1, replay_yaw=False, align_tool_z=False
        )
        position = capture[:3].copy()
        orientation = capture[3:].copy()
        robot = MagicMock()
        robot.read_pose.side_effect = lambda: (position.copy(), orientation.copy())
        robot.start_pose_servo.return_value = True
        active_index = None
        commands = []

        def move_to_pose(pose):
            position[:] = pose[0]
            return True

        robot.move_to_pose.side_effect = move_to_pose

        def send_pose(pose):
            nonlocal active_index
            target = np.asarray(pose[0]).copy()
            if clock.state() == "FORCE_HOLD":
                active_index = int(np.argmin(np.linalg.norm(actions - target[:2], axis=1)))
                commands.append((active_index, target, position.copy()))
                if active_index == rejected_index:
                    return False
                if active_index != stalled_index:
                    error = target[:2] - position[:2]
                    distance = float(np.linalg.norm(error))
                    position[:2] += error * min(1.0, 0.003 / max(distance, 1e-12))
                if not frozen_z_feedback:
                    position[2] = target[2]
            else:
                position[:] = target
            return True

        robot.send_pose.side_effect = send_pose
        sampler = MagicMock()

        def latest(_max_age):
            if clock.state() == "FORCE_HOLD" and active_index == stale_index and stale_index is not None:
                raise RuntimeError("Force sensor sample is stale")
            force = 9.0 if active_index == force_fault_index and force_fault_index is not None else 1.0
            return WrenchSample(clock.monotonic(), np.array([0, 0, force, 0, 0, 0]))

        sampler.latest.side_effect = latest
        logger = MagicMock()
        runner = HybridRunner(
            robot,
            MagicMock(),
            trajectory,
            RunnerConfig(
                control_hz=100.0,
                target_force_n=2.0,
                contact_threshold_n=0.5,
                contact_confirm_s=0.01,
                filter_cutoff_hz=1000.0,
                settle_s=0.02,
                retract_s=0.02,
                hold_duration_s=0.005,
                waypoint_xy_tolerance_m=0.0005,
                waypoint_timeout_s=0.2,
                max_tangential_step_m=0.005,
                max_tangential_speed_m_s=0.03,
                log_path=None,
            ),
            logger,
        )
        clock.state = lambda: runner.state
        updates = []
        original_update = runner.admittance.update

        def record_update(force_error, dt):
            updates.append((clock.monotonic(), force_error, dt))
            return original_update(force_error, dt)

        with (
            patch("airbot_ie.force_control.runner.time", clock),
            patch("airbot_ie.force_control.runner.ForceSampler", return_value=sampler),
            patch.object(runner, "_zero_and_collect_bias", return_value=np.zeros(6)),
            patch.object(runner.admittance, "update", side_effect=record_update),
            patch.object(runner.admittance, "reset", wraps=runner.admittance.reset) as reset,
            patch.object(runner, "_wait_for_force_hold_xy_feedback") as final_wait,
        ):
            state = runner.run()
        rows = [call.args[0] for call in logger.write.call_args_list]
        return SimpleNamespace(
            state=state,
            runner=runner,
            trajectory=trajectory,
            actions=actions,
            robot=robot,
            sampler=sampler,
            commands=commands,
            rows=[row for row in rows if row["state"] == "FORCE_HOLD"],
            updates=updates,
            reset_count=reset.call_count,
            final_wait_count=final_wait.call_count,
        )

    def test_waypoint_provider_never_interpolates_xy(self):
        actions = np.zeros((16, 2))
        actions[:, 0] = np.linspace(0.2, 0.35, 16)
        capture = np.array([0.2, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0])
        trajectory = PlanarWaypointTrajectory.from_actions(
            actions, capture, 30.0, 0.1, replay_yaw=False
        )
        with patch("airbot_ie.force_control.core._cubic_hermite", side_effect=AssertionError("Interpolation is forbidden")):
            for index in range(16):
                np.testing.assert_array_equal(trajectory.sample_waypoint(index).position_base[:2], actions[index])
            np.testing.assert_array_equal(trajectory.sample(1.0).position_base[:2], actions[0])
            np.testing.assert_array_equal(trajectory.sample(3.0).position_base[:2], actions[1])
            np.testing.assert_array_equal(trajectory.sample(-1.0).position_base[:2], actions[0])
            np.testing.assert_array_equal(trajectory.sample(31.0).position_base[:2], actions[-1])
        with self.assertRaises(IndexError):
            trajectory.sample_waypoint(16)

    def test_all_sixteen_targets_are_sent_directly_with_admittance(self):
        result = self.run_waypoints()
        self.assertEqual(result.state, "DONE", result.runner.error)
        self.assertEqual(sorted({index for index, _, _ in result.commands}), list(range(16)))
        self.assertEqual(len(result.updates), len(result.rows))
        self.assertEqual(result.reset_count, 1)
        self.assertEqual(result.final_wait_count, 0)
        for index, target, _ in result.commands:
            np.testing.assert_allclose(target[:2], result.actions[index], atol=1e-12, rtol=0)
        self.assertTrue(any(np.linalg.norm(target[:2] - actual[:2]) > 0.005 for _, target, actual in result.commands))
        for row in result.rows:
            self.assertAlmostEqual(row["command_x_m"], row["target_x_m"])
            self.assertAlmostEqual(row["command_y_m"], row["target_y_m"])
            self.assertAlmostEqual(row["command_z_m"], result.trajectory.surface_z_m - row["delta_n_m"])
        last_rows = [row for row in result.rows if row["waypoint_index"] == 15]
        self.assertGreater(len(last_rows), 1)
        self.assertGreater(last_rows[-1]["delta_n_m"], 0)
        self.assertLessEqual(last_rows[-1]["xy_error_m"], result.runner.config.waypoint_xy_tolerance_m)
        self.assertGreater(result.rows[-1]["t_s"] - result.rows[0]["t_s"], result.runner.config.hold_duration_s)

    def test_target_changes_only_after_previous_feedback_is_within_tolerance(self):
        result = self.run_waypoints()
        self.assertEqual(result.state, "DONE", result.runner.error)
        for index in range(1, 16):
            previous_rows = [row for row in result.rows if row["waypoint_index"] == index - 1]
            current_rows = [row for row in result.rows if row["waypoint_index"] == index]
            self.assertLessEqual(previous_rows[-1]["xy_error_m"], result.runner.config.waypoint_xy_tolerance_m)
            self.assertGreater(current_rows[0]["xy_error_m"], result.runner.config.waypoint_xy_tolerance_m)
            self.assertGreater(current_rows[0]["t_s"], previous_rows[-1]["t_s"])

    def test_stalled_waypoint_faults_without_skipping_points(self):
        result = self.run_waypoints(stalled_index=3)
        self.assertEqual(result.state, "FAULT")
        self.assertIn("waypoint 4/16 timed out", result.runner.error)
        self.assertEqual(sorted({index for index, _, _ in result.commands}), list(range(4)))
        self.assertEqual(len(result.updates), len(result.rows))
        self.assertEqual(result.final_wait_count, 0)
        self.assertGreater(result.robot.start_pose_servo.call_count, 1)
        result.sampler.stop.assert_called_once()

    def test_last_waypoint_force_limit_is_still_enforced(self):
        result = self.run_waypoints(force_fault_index=15)
        self.assertEqual(result.state, "FAULT")
        self.assertIn("Fz magnitude limit exceeded during force hold", result.runner.error)
        self.assertEqual(result.rows[-1]["waypoint_index"], 15)
        self.assertEqual(result.final_wait_count, 0)

    def test_last_waypoint_sensor_failure_is_still_enforced(self):
        result = self.run_waypoints(stale_index=15)
        self.assertEqual(result.state, "FAULT")
        self.assertIn("Force sensor sample is stale", result.runner.error)
        self.assertEqual(result.rows[-1]["waypoint_index"], 15)
        self.assertEqual(result.final_wait_count, 0)

    def test_rejected_waypoint_command_enters_fault(self):
        result = self.run_waypoints(rejected_index=1)
        self.assertEqual(result.state, "FAULT")
        self.assertIn("AIRBOT rejected a Cartesian pose command", result.runner.error)
        self.assertEqual(sorted({index for index, _, _ in result.commands}), [0, 1])

    def test_admittance_uses_actual_elapsed_time_between_waypoint_ticks(self):
        result = self.run_waypoints(sleep_jitter=0.002)
        self.assertEqual(result.state, "DONE", result.runner.error)
        timestamps = np.array([update[0] for update in result.updates])
        timesteps = np.array([update[2] for update in result.updates])
        np.testing.assert_allclose(timesteps[1:], np.diff(timestamps), atol=1e-12, rtol=0)

    def test_xy_acceptance_does_not_require_z_to_match_reference(self):
        result = self.run_waypoints(frozen_z_feedback=True)
        self.assertEqual(result.state, "DONE", result.runner.error)
        self.assertGreater(result.rows[-1]["position_z_m"] - result.rows[-1]["command_z_m"], 0.001)

    def test_waypoint_safety_config_is_parsed_and_validated(self):
        config = parse_runner_config({"safety": {"waypoint_xy_tolerance_m": 0.001, "waypoint_timeout_s": 4.0}})
        self.assertEqual(config.waypoint_xy_tolerance_m, 0.001)
        self.assertEqual(config.waypoint_timeout_s, 4.0)
        for name in ("waypoint_xy_tolerance_m", "waypoint_timeout_s"):
            for value in (0.0, -1.0, float("nan"), float("inf")):
                with self.subTest(name=name, value=value):
                    with self.assertRaisesRegex(ValueError, name):
                        RunnerConfig(**{name: value})

    def test_csv_preserves_waypoint_diagnostics(self):
        result = self.run_waypoints()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "force.csv"
            logger = CsvLogger(path)
            logger.write(result.rows[-1])
            logger.close()
            with path.open() as stream:
                row = next(csv.DictReader(stream))
        self.assertEqual(row["waypoint_index"], "15")
        for name in ("target_x_m", "target_y_m", "command_x_m", "command_y_m", "xy_error_m", "fz_magnitude_n"):
            self.assertAlmostEqual(float(row[name]), result.rows[-1][name])


if __name__ == "__main__":
    unittest.main()
