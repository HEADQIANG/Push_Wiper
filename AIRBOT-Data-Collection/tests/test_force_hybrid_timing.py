"""Deterministic runner checks without sleeping or connecting hardware."""

import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from airbot_ie.force_control.core import FixedXYTrajectory
from airbot_ie.force_control.hardware import WrenchSample
from airbot_ie.force_control.runner import HybridRunner, RunnerConfig


class FakeClock:
    def __init__(self, hold_sleep_overshoots=()):
        self.now = 100.0
        self.state = lambda: "PRECHECK"
        self.hold_sleep_overshoots = iter(hold_sleep_overshoots)

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration
        if duration > 0 and self.state() == "FORCE_HOLD":
            self.now += next(self.hold_sleep_overshoots, 0.0)


class ForceHybridTimingTests(unittest.TestCase):
    def run_with_clock(self, hold_sleep_overshoots=(), hold_send_delays=()):
        clock = FakeClock(hold_sleep_overshoots)
        robot = MagicMock()
        position = np.array([0.3, 0.0, 0.2])
        orientation = np.array([0.0, 0.0, 0.0, 1.0])
        robot.read_pose.side_effect = lambda: (position.copy(), orientation.copy())

        def move_to_pose(pose):
            position[:] = pose[0]
            return True

        robot.move_to_pose.side_effect = move_to_pose
        # Keep feedback at the safe pose, simulating servo tracking lag.  This
        # makes measured positions distinguishable from requested positions.
        send_delays = iter(hold_send_delays)

        def send_pose(_pose):
            if clock.state() == "FORCE_HOLD":
                clock.now += next(send_delays, 0.0)
            return True

        robot.send_pose.side_effect = send_pose
        robot.start_pose_servo.return_value = True
        reader = MagicMock()
        sampler = MagicMock()
        sample_count = 0

        def latest(_max_age):
            nonlocal sample_count
            sample_count += 1
            # The fake end pose has identity orientation, so an upward contact
            # force is positive on its sensor Z axis.
            force = 0.0 if sample_count < 4 else 1.0
            return WrenchSample(clock.monotonic(), np.array([0, 0, force, 0, 0, 0]))

        sampler.latest.side_effect = latest
        logger = MagicMock()
        trajectory = FixedXYTrajectory([0.3, 0.0], orientation)
        trajectory.set_z_reference(0.15)
        runner = HybridRunner(
            robot,
            reader,
            trajectory,
            RunnerConfig(
                control_hz=100.0,
                target_force_n=2.0,
                filter_cutoff_hz=1000.0,
                zero_settle_s=0.001,
                settle_s=0.05,
                hold_duration_s=0.055,
                retract_s=0.01,
                log_path=None,
            ),
            logger,
        )
        clock.state = lambda: runner.state
        updates = []
        original_update = runner.admittance.update

        def record_update(force_error, dt):
            updates.append((clock.monotonic(), dt))
            return original_update(force_error, dt)

        with (
            patch("airbot_ie.force_control.runner.time", clock),
            patch("airbot_ie.force_control.runner.collect_bias", return_value=np.zeros(6)),
            patch("airbot_ie.force_control.runner.ForceSampler", return_value=sampler),
            patch.object(runner.admittance, "update", side_effect=record_update),
        ):
            self.assertEqual(runner.run(), "DONE", runner.error)
        rows = [call.args[0] for call in logger.write.call_args_list]
        return updates, rows, position

    def test_force_hold_does_not_catch_up_ticks_from_contact_settling(self):
        updates, _, _ = self.run_with_clock()
        self.assertGreaterEqual(len(updates), 3)
        times = np.array([timestamp for timestamp, _ in updates])
        np.testing.assert_allclose(np.diff(times), 0.01, atol=1e-12, rtol=0)

    def test_admittance_uses_actual_elapsed_time_after_sleep_jitter(self):
        updates, _, _ = self.run_with_clock(hold_sleep_overshoots=[0.002])
        self.assertGreaterEqual(len(updates), 3)
        times, timesteps = np.array(updates).T
        self.assertAlmostEqual(timesteps[0], 0.01)
        self.assertAlmostEqual(timesteps[1], 0.012)
        self.assertAlmostEqual(timesteps[2], 0.008)
        np.testing.assert_allclose(timesteps[1:], np.diff(times), atol=1e-12, rtol=0)

    def test_slow_command_does_not_replay_missed_control_ticks(self):
        updates, _, _ = self.run_with_clock(hold_send_delays=[0.035])
        self.assertGreaterEqual(len(updates), 3)
        times, timesteps = np.array(updates).T
        self.assertAlmostEqual(times[1] - times[0], 0.035)
        np.testing.assert_allclose(np.diff(times[1:]), 0.01, atol=1e-12, rtol=0)
        np.testing.assert_allclose(timesteps[1:], np.diff(times), atol=1e-12, rtol=0)

    def test_approach_and_hold_logs_keep_feedback_separate_from_commands(self):
        _, rows, feedback = self.run_with_clock()
        for state in ("APPROACH", "FORCE_HOLD"):
            with self.subTest(state=state):
                phase_rows = [row for row in rows if row["state"] == state]
                self.assertTrue(phase_rows)
                for row in phase_rows:
                    np.testing.assert_allclose(
                        [row["position_x_m"], row["position_y_m"], row["position_z_m"]],
                        feedback,
                    )
                self.assertTrue(
                    any(abs(row["command_z_m"] - row["position_z_m"]) > 1e-6 for row in phase_rows)
                )


if __name__ == "__main__":
    unittest.main()
