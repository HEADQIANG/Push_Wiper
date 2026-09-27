"""Run with unittest; no robot or camera hardware is imported/connected."""

import json
import time
import unittest
from contextlib import redirect_stdout
from copy import deepcopy
from io import BytesIO, StringIO
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Event
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import av
import numpy as np
from mcap.reader import make_reader

from airbot_ie.push_wiper.collect import (
    MockCamera,
    bindings,
    check_control_status,
    load_references,
    main,
    run_collection,
    run_demo,
)
from airbot_ie.push_wiper.config import (
    CollectionConfig,
    ResetConfig,
    WorkHeightReference,
    atomic_json,
)
from airbot_ie.push_wiper.clock import AcquisitionClock, acquisition_ns
from airbot_ie.push_wiper.controller import (
    CommandError,
    ControllerRunner,
    DualArmController,
    SingleArmDragController,
)
from airbot_ie.push_wiper.devices import (
    CameraWorker, MockArm, SDKArm, lock_robot_endpoints,
)
from airbot_ie.push_wiper.export import MaskConfig, MaskRule, export_dataset, stain_mask
from airbot_ie.push_wiper.geometry import (
    planar_actions,
    pose_error,
    resample_actions,
)
from airbot_ie.push_wiper.session import CollectionSession


class ClockTests(unittest.TestCase):
    def test_wall_clock_steps_do_not_change_elapsed_acquisition_time(self):
        with (
            patch("time.time_ns", return_value=1_800_000_000_000_000_000) as wall,
            patch("time.monotonic_ns", return_value=1_000_000_000) as monotonic,
        ):
            clock = AcquisitionClock()
            start = clock.now_ns()
            wall.return_value -= 5_000_000_000
            monotonic.return_value += 50_000_000
            self.assertEqual(clock.now_ns() - start, 50_000_000)
            wall.return_value += 20_000_000_000
            monotonic.return_value += 50_000_000
            self.assertEqual(clock.now_ns() - start, 100_000_000)
            self.assertEqual(clock.now_ns() - start, 100_000_001)
            wall.assert_called_once()

    def test_cached_camera_frame_ages_despite_wall_clock_rollback(self):
        with (
            patch("time.time_ns", return_value=1_800_000_000_000_000_000) as wall,
            patch("time.monotonic_ns", return_value=1_000_000_000) as monotonic,
            patch("airbot_ie.push_wiper.clock._clock", AcquisitionClock()),
        ):
            camera = CameraWorker(CollectionConfig().camera)
            frame = {"data": np.zeros((4, 4, 3), dtype=np.uint8), "t": wall()}
            camera._publish_frame(frame)
            stamp = camera.latest()["t"]
            wall.return_value -= 5_000_000_000
            monotonic.return_value += 100_000_000
            self.assertEqual(camera.latest()["t"], stamp)
            monotonic.return_value += 500_000_000
            with self.assertRaisesRegex(RuntimeError, "frame is stale"):
                camera.latest()
            # Only a newly captured frame can clear the stale-frame condition.
            camera._publish_frame(frame | {"t": wall()})
            self.assertGreater(camera.latest()["t"], stamp)


class RunnerTests(unittest.TestCase):
    def test_slow_mode_switch_is_busy_without_publishing_old_feedback_as_new(self):
        arm = MockArm()
        controller = SingleArmDragController(arm, ResetConfig())
        runner = ControllerRunner(controller)
        entered, release = Event(), Event()
        original = arm.set_mode

        def delayed_mode(mode):
            if mode == "gravity":
                entered.set()
                if not release.wait(2):
                    raise RuntimeError("test release timed out")
            return original(mode)

        with patch.object(arm, "set_mode", side_effect=delayed_mode):
            runner.start()
            try:
                future = runner.submit("resume_follow")
                self.assertTrue(entered.wait(2))
                snapshot = runner.snapshot()
                active = snapshot["active_command"]
                self.assertEqual(active["name"], "resume_follow")
                self.assertFalse(check_control_status(
                    snapshot, CollectionConfig().robot,
                    now_ns=active["started_ns"] + 600_000_000,
                ))
                again = runner.snapshot()
                self.assertEqual(again["published_ns"], snapshot["published_ns"])
                self.assertEqual(again["samples"], snapshot["samples"])
                release.set()
                future.result(timeout=2)
                ready = runner.snapshot()
                self.assertIsNone(ready["active_command"])
                self.assertEqual(ready["state"], "following")
                self.assertGreater(
                    ready["samples"]["follow"]["t_ns"], active["started_ns"]
                )
                self.assertTrue(check_control_status(ready, CollectionConfig().robot))
            finally:
                release.set()
                runner.close()

    def test_command_deadline_cancels_remaining_reset_motion(self):
        arm = MockArm()
        controller = SingleArmDragController(arm, ResetConfig())
        controller.tick(0)
        controller.observation = deepcopy(controller.samples)
        runner = ControllerRunner(controller)
        entered, release = Event(), Event()
        original = arm.set_mode

        def delayed_mode(mode):
            if mode == "planning":
                entered.set()
                if not release.wait(2):
                    raise RuntimeError("test release timed out")
            return original(mode)

        with patch.object(arm, "set_mode", side_effect=delayed_mode):
            runner.start()
            try:
                future = runner.submit("reset_to_observation")
                self.assertTrue(entered.wait(2))
                snapshot = runner.snapshot()
                with self.assertRaisesRegex(RuntimeError, "timed out: reset_to_observation"):
                    check_control_status(
                        snapshot, CollectionConfig().robot,
                        now_ns=snapshot["active_command"]["started_ns"] + 11_000_000_000,
                    )
                runner.request_stop("Command timed out")
                release.set()
                with self.assertRaisesRegex(RuntimeError, "motion command cancelled"):
                    future.result(timeout=2)
            finally:
                release.set()
                runner.close()
        self.assertFalse(any(command[0] == "move" for command in arm.commands))
        self.assertEqual(arm.mode, "servo")

    def test_queued_command_does_not_mask_stalled_control_tick(self):
        arm = MockArm()
        runner = ControllerRunner(SingleArmDragController(arm, ResetConfig()))
        entered, release = Event(), Event()
        original = arm.read

        def delayed_read():
            entered.set()
            if not release.wait(2):
                raise RuntimeError("test release timed out")
            return original()

        with patch.object(arm, "read", side_effect=delayed_read):
            runner.start()
            try:
                self.assertTrue(entered.wait(2))
                future = runner.submit("resume_follow")
                snapshot = runner.snapshot()
                self.assertIsNone(snapshot["active_command"])
                with self.assertRaisesRegex(RuntimeError, "unresponsive.*no command running"):
                    check_control_status(
                        snapshot, CollectionConfig().robot,
                        now_ns=snapshot["published_ns"] + 600_000_000,
                    )
                runner.request_stop()
                release.set()
            finally:
                release.set()
                runner.close()
        with self.assertRaisesRegex(RuntimeError, "Controller stopped"):
            future.result(timeout=1)
        self.assertNotIn(("mode", "gravity"), arm.commands)

    def test_non_motion_command_keeps_short_feedback_deadline(self):
        snapshot = {
            "state": "idle", "published_ns": 100,
            "active_command": {"name": "register_work_height", "started_ns": 100},
        }
        with self.assertRaisesRegex(RuntimeError, "register_work_height.*limit=0.500s"):
            check_control_status(snapshot, CollectionConfig().robot, now_ns=600_000_100)


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.lead, self.follow = MockArm(), MockArm()
        self.control = DualArmController(
            self.lead, self.follow, ResetConfig(stable_s=0.1, timeout_s=1)
        )
        self.control.tick(0)
        self.control.tick(0.2)
        self.control.register_observation_pose(0.2)
        self.control.tick(0.21)
        self.control.tick(0.4)
        self.assertEqual(self.control.state, "observing")

    def test_reference_mapping_and_no_gripper(self):
        self.assertEqual(
            self.control.observation["lead"]["joints"],
            self.control.observation["follow"]["joints"],
        )
        # MockArm intentionally has no gripper methods.
        self.control.resume_follow(0.5)
        self.lead.joints[0] = 0.01
        self.control.tick(0.6)
        self.control.tick(0.7)
        self.assertAlmostEqual(self.follow.joints[0], 0.01)

    def test_no_follow_commands_during_reset(self):
        self.control.resume_follow(0.5)
        self.lead.joints[0] = 0.01
        self.control.tick(0.6)
        self.control.tick(0.7)
        self.control.reset_to_observation(0.8)
        counts = [len(arm.commands) for arm in (self.lead, self.follow)]
        with self.assertRaises(CommandError):
            self.control.reset_to_observation(0.81)
        self.control.tick(0.85)
        self.control.tick(1.0)
        self.assertEqual(self.control.state, "observing")
        self.assertEqual(
            counts, [len(arm.commands) for arm in (self.lead, self.follow)]
        )

    def test_far_reset_allowed_but_waits_for_each_arm_to_arrive(self):
        self.control.resume_follow(0.5)
        self.lead.joints[0] = 0.8
        self.control.tick(0.6)
        self.control.tick(0.7)
        self.follow.stuck = True
        self.control.reset_to_observation(0.8)
        self.assertEqual(self.control.state, "resetting")
        self.control.tick(0.9)
        self.control.tick(1.1)
        self.assertEqual(self.control.state, "resetting")
        self.follow.stuck = False
        self.control.tick(1.2)
        self.assertEqual(self.control.state, "resetting")
        self.control.tick(1.4)
        self.assertEqual(self.control.state, "observing")

    def test_velocity_feedback_does_not_gate_follow_registration_or_reset(self):
        self.control.state = "idle"
        for arm in (self.lead, self.follow):
            read = arm.read
            arm.read = lambda read=read: read() | {"velocity": [0.03663] * 6}
        self.control.tick(0.5)
        self.control.resume_follow(0.5)
        self.assertEqual(self.control.state, "following")
        self.control.register_observation_pose(0.5)
        self.control.tick(0.6)
        self.control.tick(0.8)
        self.assertEqual(self.control.state, "observing")
        self.control.reset_to_observation(0.8)
        self.control.tick(0.9)
        self.control.tick(1.1)
        self.assertEqual(self.control.state, "observing")

    def test_reset_without_reference_is_rejected(self):
        self.control.state = "idle"
        self.control.observation = None
        with self.assertRaisesRegex(CommandError, "Register observation"):
            self.control.reset_to_observation(0.5)

    def test_work_height_records_follower_z_without_motion(self):
        self.control.state = "idle"
        self.lead.joints[2] = 0.4
        self.follow.joints[2] = 0.12
        self.control.tick(0.5)
        counts = [len(arm.commands) for arm in (self.lead, self.follow)]
        height = self.control.register_work_height(0.5)
        self.assertAlmostEqual(height["z_m"], 0.42)
        self.assertEqual(height["t_ns"], self.control.samples["follow"]["t_ns"])
        self.assertEqual(height["frame"], "follower_base")
        self.assertEqual(height["point"], "sdk_end_reference")
        self.assertEqual(
            counts, [len(arm.commands) for arm in (self.lead, self.follow)]
        )
        self.assertEqual(self.control.state, "idle")
        self.control.state = "resetting"
        with self.assertRaises(CommandError):
            self.control.register_work_height(0.6)

    def test_single_arm_motion_failure_holds_both(self):
        self.follow.reject_move = True
        with self.assertRaises(RuntimeError):
            self.control.reset_to_observation(0.5)
        self.assertEqual(self.control.state, "fault")
        self.assertEqual(set(self.control.hold_targets), {"lead", "follow"})
        with self.assertRaises(CommandError):
            self.control.resume_follow(0.6)

    def test_timeout_holds_current_pose_and_does_not_resume(self):
        self.control.resume_follow(0.5)
        self.lead.joints[0] = 0.01
        self.control.tick(0.6)
        self.control.tick(0.7)
        self.follow.stuck = True
        self.control.reset_to_observation(0.8)
        self.control.tick(2)
        self.assertEqual(self.control.state, "fault")
        self.assertAlmostEqual(self.control.hold_targets["follow"][0], 0.01)
        self.assertIn("timed out", self.control.error)

    def test_disconnect_still_holds_other_arm(self):
        self.lead.fail_read = True
        self.control.tick(0.5)
        self.assertEqual(self.control.state, "fault")
        self.assertIn("lead", self.control.hold_errors)
        self.assertIn("follow", self.control.hold_targets)

    def test_resume_allows_joint_mismatch_and_limits_command_slew(self):
        self.control.state = "idle"
        self.lead.joints[0] = 0.8
        self.control.tick(0.5)
        self.control.resume_follow(0.6)
        self.assertEqual(self.control.state, "following")
        np.testing.assert_allclose(self.follow.commands[-1][1], np.zeros(6))
        self.control.tick(0.61)
        first = self.follow.commands[-1][1][0]
        self.assertAlmostEqual(first, 0.005)
        self.control.tick(1.61)
        self.assertLessEqual(self.follow.commands[-1][1][0] - first, 0.025 + 1e-12)

    def test_observation_preserves_distinct_arm_targets_and_reloads(self):
        self.control.state = "idle"
        self.lead.joints[0] = 0.4
        self.follow.joints[0] = -0.2
        self.control.tick(0.5)
        reference = self.control.register_observation_pose(0.6)
        self.assertEqual(reference["lead"]["joints"][0], 0.4)
        self.assertEqual(reference["follow"]["joints"][0], -0.2)
        self.assertEqual(
            self.follow.commands[-1], ("move", reference["follow"]["joints"])
        )
        self.control.tick(0.7)
        self.control.tick(0.9)
        self.assertEqual(self.control.state, "observing")
        with TemporaryDirectory() as directory:
            config = CollectionConfig()
            path = Path(directory) / "references.json"
            atomic_json(
                path,
                {
                    "schema_version": 1,
                    "bindings": bindings(config, True),
                    "observation": reference,
                },
            )
            loaded = load_references(path, config, True)
            self.assertEqual(loaded["observation"], reference)

    def test_observation_drift_pauses_capture_then_requires_stability(self):
        reference = deepcopy(self.control.observation)
        counts = [len(arm.commands) for arm in (self.lead, self.follow)]
        self.follow.joints[0] = 0.01
        self.control.tick(0.5)
        self.assertEqual(self.control.state, "settling")
        self.assertEqual(self.control.events[-1]["name"], "observation_unstable")
        self.follow.joints[0] = 0
        self.control.tick(0.55)
        self.assertEqual(self.control.state, "settling")
        self.follow.joints[0] = 0.01
        self.control.tick(0.6)
        self.follow.joints[0] = 0
        self.control.tick(0.65)
        self.control.tick(0.7)
        self.assertEqual(self.control.state, "settling")
        self.control.tick(0.8)
        self.assertEqual(self.control.state, "observing")
        self.assertEqual(self.control.events[-1]["name"], "observation_recovered")
        self.assertEqual(self.control.observation, reference)
        self.assertEqual(counts, [len(arm.commands) for arm in (self.lead, self.follow)])

    def test_persistent_observation_drift_fault_includes_measured_errors(self):
        self.follow.joints[0] = 0.01
        self.control.tick(0.5)
        self.control.tick(1.6)
        self.assertEqual(self.control.state, "fault")
        self.assertIn("recovery timed out", self.control.error)
        self.assertIn("follow: position=10.000mm (limit 2.000mm)", self.control.error)
        self.assertIn("angle=", self.control.error)
        self.assertIn("joint=", self.control.error)
        self.assertEqual(set(self.control.hold_targets), {"lead", "follow"})

    def test_explicit_reset_can_restart_while_observation_is_unstable(self):
        self.follow.joints[0] = 0.01
        self.control.tick(0.5)
        self.control.reset_to_observation(0.6)
        self.control.tick(0.61)
        self.control.tick(0.8)
        self.assertEqual(self.control.state, "observing")

    def test_disconnect_during_stability_wait_still_faults_immediately(self):
        self.follow.joints[0] = 0.01
        self.control.tick(0.5)
        self.follow.fail_read = True
        self.control.tick(0.51)
        self.assertEqual(self.control.state, "fault")
        self.assertIn("follow", self.control.hold_errors)


class SingleArmDragTests(unittest.TestCase):
    def setUp(self):
        self.arm = MockArm()
        self.control = SingleArmDragController(
            self.arm, ResetConfig(stable_s=0.1, timeout_s=1)
        )
        self.control.tick(0)

    def observe(self, now):
        reference = self.control.register_observation_pose(now)
        self.control.tick(now + 0.01)
        self.control.tick(now + 0.2)
        self.assertEqual(self.control.state, "observing")
        return reference

    def test_drag_reads_real_motion_without_position_commands(self):
        self.assertEqual(self.arm.commands, [])  # Idle does not move the arm.
        self.control.resume_follow(0.1)
        self.assertEqual(self.arm.commands, [("mode", "gravity")])
        for index in range(1, 6):
            self.arm.joints[0] = index * 0.05
            self.control.tick(0.1 + index * 0.1)
            self.assertEqual(set(self.control.samples), {"follow"})
            self.assertAlmostEqual(
                self.control.samples["follow"]["position"][0], 0.3 + index * 0.05
            )
        self.assertEqual(self.arm.commands, [("mode", "gravity")])
        self.assertEqual(self.control.events[-1]["name"], "drag_resumed")

    def test_observation_height_reset_and_resume(self):
        self.arm.joints[0] = 0.2
        self.control.tick(0.1)
        reference = self.observe(0.2)
        self.assertEqual(set(reference), {"follow"})
        self.control.resume_follow(0.5)
        self.arm.joints[2] = -0.1
        self.control.tick(0.6)
        commands = list(self.arm.commands)
        height = self.control.register_work_height(0.6)
        self.assertAlmostEqual(height["z_m"], 0.2)
        self.assertEqual(self.arm.commands, commands)
        self.control.reset_to_observation(0.7)
        self.assertEqual(self.arm.mode, "planning")
        with self.assertRaises(CommandError):
            self.control.resume_follow(0.71)
        self.control.tick(0.72)
        self.assertEqual(self.control.state, "resetting")
        self.control.tick(0.9)
        self.assertEqual(self.control.state, "observing")
        np.testing.assert_allclose(self.arm.joints, reference["follow"]["joints"])
        count = len(self.arm.commands)
        self.control.resume_follow(1.0)
        self.control.tick(1.1)
        self.assertEqual(self.arm.commands[count:], [("mode", "gravity")])

    def test_reset_timeout_and_exit_hold_single_arm(self):
        self.observe(0.1)
        self.control.resume_follow(0.4)
        self.arm.joints[0] = 0.5
        self.control.tick(0.5)
        self.arm.stuck = True
        self.control.reset_to_observation(0.6)
        self.control.tick(1.7)
        self.assertEqual(self.control.state, "fault")
        self.assertIn("timed out", self.control.error)
        self.assertEqual(set(self.control.hold_targets), {"follow"})
        self.assertEqual(self.arm.commands[-2][0:2], ("mode", "servo"))
        self.assertAlmostEqual(self.arm.commands[-1][1][0], 0.5)
        with self.assertRaises(CommandError):
            self.control.resume_follow(1.8)
        # The same stop path used on Esc holds the measured pose during dragging.
        arm = MockArm()
        control = SingleArmDragController(arm, ResetConfig())
        control.tick(0)
        control.resume_follow(0.1)
        arm.joints[0] = 0.3
        control.fault("Collector stopped")
        self.assertAlmostEqual(control.hold_targets["follow"][0], 0.3)

    def test_gravity_rejection_and_disconnect_are_faults(self):
        with patch.object(self.arm, "set_mode", side_effect=[False, True]):
            with self.assertRaisesRegex(RuntimeError, "gravity"):
                self.control.resume_follow(0.1)
        self.assertEqual(self.control.state, "fault")
        self.assertIn("follow", self.control.hold_targets)
        arm = MockArm()
        control = SingleArmDragController(arm, ResetConfig())
        control.tick(0)
        control.resume_follow(0.1)
        arm.fail_read = True
        control.tick(0.2)
        self.assertEqual(control.state, "fault")
        self.assertEqual(set(control.hold_errors), {"follow"})

    def test_mock_keys_move_tool_arm(self):
        with self.assertRaises(CommandError):
            self.control.simulate_lead(0, [0.01, 0, 0, 0, 0, 0])
        self.control.resume_follow(0.1)
        self.control.simulate_lead(0.2, [0.01, 0, 0, 0, 0, 0])
        self.control.tick(0.3)
        self.assertAlmostEqual(self.control.samples["follow"]["position"][0], 0.31)

    def test_g_can_release_single_arm_during_observation_stability_wait(self):
        self.observe(0.1)
        self.arm.joints[0] = 0.01
        self.control.tick(0.4)
        self.assertEqual(self.control.state, "settling")
        count = len(self.arm.commands)
        self.control.resume_follow(0.5)
        self.control.tick(0.6)
        self.assertEqual(self.control.state, "following")
        self.assertEqual(self.arm.commands[count:], [("mode", "gravity")])


class GeometryTests(unittest.TestCase):
    def test_quaternion_sign_does_not_change_pose(self):
        reference = {"position": [0, 0, 0], "orientation": [0, 0, 0, 1]}
        current = {"position": [0.001, 0, 0], "orientation": [0, 0, 0, -1]}
        self.assertEqual(pose_error(reference, current), (0.001, 0.0))

    def test_base_yaw_with_tilted_flange_and_wrap(self):
        # Capture frame is rotated 180 degrees about base X, not identity.
        capture = [1, 0, 0, 0]
        angles = np.deg2rad([170, 175, -179, -170])
        # q_z(yaw) * q_x(pi)
        orientations = [[np.cos(a / 2), np.sin(a / 2), 0, 0] for a in angles]
        positions = [[0.3 + i * 0.01, 0.2, 0.1] for i in range(4)]
        actions = planar_actions(positions, orientations, capture)
        np.testing.assert_allclose(actions[:, 2], np.deg2rad([170, 175, 181, 190]))
        result = resample_actions([0, 100, 230, 400], actions)
        self.assertEqual(result.shape, (16, 3))
        np.testing.assert_allclose(result[[0, -1]], actions[[0, -1]])
        self.assertTrue(np.all(np.diff(result[:, 2]) > 0))

    def test_capture_yaw_reference_is_independent_of_roll(self):
        def yaw_roll(yaw_deg, roll_deg):
            y, r = np.deg2rad([yaw_deg, roll_deg]) / 2
            return [
                np.cos(y) * np.sin(r),
                np.sin(y) * np.sin(r),
                np.sin(y) * np.cos(r),
                np.cos(y) * np.cos(r),
            ]

        capture = yaw_roll(30, 60)
        orientations = [yaw_roll(70, 180), yaw_roll(80, 180)]
        positions = [[0.3, 0.1, 0.2], [0.4, 0.2, 0.2]]
        actions = planar_actions(positions, orientations, capture)
        np.testing.assert_allclose(actions[:, 2], np.deg2rad([40, 50]))
        np.testing.assert_allclose(actions[:, :2], np.asarray(positions)[:, :2])
        flipped = planar_actions(
            positions, -np.asarray(orientations), -np.asarray(capture)
        )
        np.testing.assert_allclose(flipped, actions)

    def test_capture_yaw_difference_wraps_before_resampling(self):
        def yaw(degrees):
            a = np.deg2rad(degrees) / 2
            return [0, 0, np.sin(a), np.cos(a)]

        actions = planar_actions(
            [[0, 0, 0]] * 3, [yaw(170), yaw(-179), yaw(-170)], yaw(-170)
        )
        np.testing.assert_allclose(actions[:, 2], np.deg2rad([-20, -9, 0]), atol=1e-12)
        sampled = resample_actions([0, 100, 200], actions)
        np.testing.assert_allclose(
            sampled[[0, -1], 2], np.deg2rad([-20, 0]), atol=1e-12
        )

    def test_vertical_capture_heading_is_rejected(self):
        vertical = [0, np.sqrt(0.5), 0, np.sqrt(0.5)]
        with self.assertRaisesRegex(ValueError, "yaw is undefined"):
            planar_actions([[0, 0, 0]], [[0, 0, 0, 1]], vertical)

    def test_non_monotonic_time_rejected(self):
        with self.assertRaises(ValueError):
            resample_actions([5, 5], np.zeros((2, 3)))

    def test_invalid_quaternion_rejected(self):
        with self.assertRaises(ValueError):
            planar_actions([[0, 0, 0]], [[0, 0, 0, 0]], [0, 0, 0, 1])

    def test_gray_and_hsv_mask_rules(self):
        image = np.full((40, 40, 3), 255, np.uint8)
        image[10:20, 10:20] = [0, 0, 180]
        mask, info = stain_mask(image, MaskConfig(morphology_kernel=1))
        self.assertEqual(mask[15, 15], 0)
        self.assertEqual(mask[0, 0], 1)
        self.assertEqual(info["components"], 1)
        mask, _ = stain_mask(
            image, MaskConfig(rules=[MaskRule(space="gray", lower=[0], upper=[120])])
        )
        self.assertEqual(mask[15, 15], 0)


class AdapterTests(unittest.TestCase):
    def test_existing_adapter_accepts_empty_eef_and_stamps_pose_once(self):
        from airbot_ie.robots.airbot_play import AIRBOTPlay

        robot = AIRBOTPlay(components=["arm"])
        sdk = MagicMock()
        sdk.connect.return_value = True
        sdk.get_product_info.return_value = {"product_type": "play", "eef_types": []}
        sdk.get_joint_pos.return_value = [0.0] * 6
        sdk.get_joint_vel.return_value = [0.0] * 6
        sdk.get_joint_eff.return_value = [0.0] * 6
        sdk.get_end_pose.return_value = [[0.3, 0, 0.3], [0, 0, 0, 1]]
        robot.interface = sdk
        self.assertTrue(robot.on_configure())
        robot.set_post_capture(None, {})
        observation = robot.capture_observation()
        self.assertEqual(
            observation["eef/pose/position"]["t"],
            observation["eef/pose/orientation"]["t"],
        )
        sdk.get_eef_pos.assert_not_called()
        sdk.servo_eef_pos.assert_not_called()

    def test_sdk_reads_do_not_send_commands_and_detect_stale_cache(self):
        arm = SDKArm.__new__(SDKArm)
        sdk = MagicMock()
        sdk._feedback_jointstates = object()
        sdk._feedbacking = True
        sdk.get_joint_pos.return_value = [0] * 6
        sdk.get_joint_vel.return_value = [0] * 6
        sdk.get_joint_eff.return_value = [0] * 6
        sdk.get_end_pose.return_value = [[0.3, 0, 0.3], [0, 0, 0, 1]]
        arm.sdk = sdk
        arm.last_feedback = None
        arm.feedback_changed = 0
        arm.feedback_timeout_s = 0.5
        arm.read()
        sdk.switch_mode.assert_not_called()
        sdk.set_params.assert_not_called()
        sdk.servo_joint_pos.assert_not_called()
        sdk.get_eef_pos.assert_not_called()
        arm.feedback_changed -= 1
        with self.assertRaisesRegex(RuntimeError, "stale"):
            arm.read()

    def test_legacy_arm_only_follow_skips_gripper(self):
        from airbot_ie.scripts.task_follow import _follow_connected

        lead, follow = MagicMock(), MagicMock()
        lead.get_joint_pos.return_value = [0.0] * 6
        follow.get_joint_pos.return_value = [0.0] * 6
        mode = SimpleNamespace(PLANNING_POS=1, GRAVITY_COMP=2, SERVO_JOINT_POS=3)
        speed = SimpleNamespace(DEFAULT=0)
        with (
            patch("time.sleep", side_effect=KeyboardInterrupt),
            self.assertRaises(KeyboardInterrupt),
        ):
            _follow_connected(
                SimpleNamespace(no_eef=True),
                MagicMock(),
                [(lead, follow)],
                mode,
                speed,
            )
        follow.servo_joint_pos.assert_called_once_with([0.0] * 6)
        lead.get_eef_pos.assert_not_called()
        follow.servo_eef_pos.assert_not_called()


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.temp = TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.config = CollectionConfig(output=self.temp.name)
        sample = MockArm().read()
        self.snapshot = {
            "state": "observing",
            "samples": {"lead": deepcopy(sample), "follow": deepcopy(sample)},
            "events": [],
            "errors": {},
        }
        self.references = {
            "observation": deepcopy(self.snapshot["samples"]),
            "work_height": WorkHeightReference(
                z_m=sample["position"][2], t_ns=sample["t_ns"]
            ).model_dump(),
        }
        self.session = CollectionSession(self.config, self.references, simulated=True)
        self.session.new_task()
        self.camera = MockCamera(self.config.camera)
        self.addCleanup(lambda: self.session.abort("test cleanup"))

    def test_missing_observation_prevents_task_and_identifies_required_key(self):
        self.session.end_task()
        self.references.pop("observation")
        with self.assertRaisesRegex(
            CommandError, r"Missing reference: observation \(O\)"
        ):
            self.session.new_task()
        self.assertIsNone(self.session.task)
        self.assertIsNone(self.session.writer)

    def test_work_height_required_once_and_reused_across_tasks(self):
        with self.assertRaisesRegex(CommandError, "between tasks"):
            self.session.allow_reference_update()
        self.session.end_task()
        self.session.allow_reference_update()
        height = self.references.pop("work_height")
        with self.assertRaisesRegex(CommandError, r"Missing work height \(Z\)"):
            self.session.new_task()
        self.assertIsNone(self.session.task)
        self.references["work_height"] = height
        for _ in range(2):
            self.session.new_task()
            self.assertEqual(self.session.task["references"]["work_height"], height)
            self.session.end_task()

    def test_segment_uses_height_frozen_at_task_start(self):
        saved_height = deepcopy(self.session.task["references"]["work_height"])
        # Even an external mutation cannot relabel an existing task's segments.
        self.references["work_height"]["z_m"] = 0.9
        self.session.capture(self.snapshot, self.camera.latest())
        metadata = self.session.writer.metadata
        self.assertEqual(metadata["references"]["work_height"], saved_height)

    def test_requires_observation_hold_and_two_boundaries(self):
        for state in ("following", "settling"):
            wrong = deepcopy(self.snapshot) | {"state": state}
            with self.assertRaises(CommandError):
                self.session.capture(wrong, self.camera.latest())
        self.session.capture(self.snapshot, self.camera.latest())
        with self.assertRaises(CommandError):
            self.session.finish_segment()
        self.snapshot["state"] = "following"
        self.session.toggle_push(self.snapshot, self.camera.latest())
        with self.assertRaises(CommandError):
            self.session.allow_reset()
        with self.assertRaises(CommandError):
            self.session.allow_follow()
        self.session.toggle_push(self.snapshot, self.camera.latest())
        self.session.allow_reset()
        with self.assertRaises(CommandError):
            self.session.capture(self.snapshot, self.camera.latest())
        self.snapshot["state"] = "observing"
        self.session.capture(self.snapshot, self.camera.latest())
        result = self.session.finish_segment()
        self.assertEqual(
            json.loads((result / "meta.json").read_text())["status"], "accepted"
        )

    def test_old_camera_frame_after_reset_rejected(self):
        frame = self.camera.latest()
        for event in ("reset_completed", "observation_recovered"):
            self.snapshot["events"] = [{"name": event, "t_ns": frame["t"] + 1}]
            with self.assertRaises(CommandError):
                self.session.capture(self.snapshot, frame)
        self.assertIsNone(self.session.writer)

    def test_busy_command_cannot_capture_or_repeat_stale_samples(self):
        busy = deepcopy(self.snapshot) | {
            "active_command": {"name": "resume_follow", "started_ns": acquisition_ns()}
        }
        with self.assertRaisesRegex(CommandError, "robot command"):
            self.session.capture(busy, self.camera.latest())
        self.assertIsNone(self.session.writer)
        self.session.capture(self.snapshot, self.camera.latest())
        count = self.session.writer.metadata["sample_count"]
        with self.assertRaisesRegex(CommandError, "robot command"):
            self.session.sample(busy, self.camera.latest())
        self.assertEqual(self.session.writer.metadata["sample_count"], count)

    def test_abort_preserves_incomplete_segment(self):
        self.session.capture(self.snapshot, self.camera.latest())
        self.session.abort("Camera disconnected")
        metadata = json.loads((self.session.last_result / "meta.json").read_text())
        self.assertEqual(metadata["status"], "incomplete")
        self.assertIn("Camera disconnected", metadata["reason"])
        self.assertTrue((self.session.last_result / "raw.mcap").exists())

    def test_timeout_not_accepted(self):
        self.session.capture(self.snapshot, self.camera.latest())
        self.session.started_monotonic -= self.config.max_segment_s + 1
        with self.assertRaises(CommandError):
            self.session.sample(self.snapshot, self.camera.latest())
        self.assertIsNone(self.session.writer)
        self.assertEqual(
            json.loads((self.session.last_result / "meta.json").read_text())["status"],
            "incomplete",
        )

    def test_save_failure_stays_partial_and_incomplete(self):
        self.session.capture(self.snapshot, self.camera.latest())
        writer = self.session.writer
        with (
            patch.object(writer.sampler, "save", side_effect=OSError("disk full")),
            self.assertRaises(OSError),
        ):
            writer.finish("rejected")
        self.session.writer = None
        self.assertTrue(writer.directory.name.endswith(".partial"))
        self.assertEqual(
            json.loads((writer.directory / "meta.json").read_text())["status"],
            "incomplete",
        )


class IntegrationTests(unittest.TestCase):
    def setUp(self):
        self.temp = TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def demo(self, mode="teleop"):
        with redirect_stdout(StringIO()):
            run_demo(
                CollectionConfig(output=str(self.root / "raw"), robot={"mode": mode})
            )

    def test_wall_clock_rollback_during_capture_still_exports_valid_stroke(self):
        config = CollectionConfig(
            robot={"mode": "single_drag"}, output=str(self.root / "raw")
        )
        with (
            patch("time.time_ns", return_value=1_800_000_000_000_000_000) as wall,
            patch("time.monotonic_ns", return_value=1_000_000_000) as monotonic,
            patch("airbot_ie.push_wiper.clock._clock", AcquisitionClock()),
        ):
            arm = MockArm()
            controller = SingleArmDragController(arm, config.reset)
            camera = MockCamera(config.camera)
            controller.tick(0)
            observation = controller.register_observation_pose(0)
            controller.tick(0.01)
            controller.tick(0.6)
            session = CollectionSession(
                config,
                {
                    "observation": observation,
                    "work_height": controller.register_work_height(0.6),
                },
                simulated=True,
            )
            self.addCleanup(lambda: session.abort("test cleanup"))
            session.new_task()
            session.capture(controller.get_status(), camera.latest())
            controller.resume_follow(0.7)
            session.toggle_push(controller.get_status(), camera.latest())
            # Reproduce both backwards and forwards host wall-clock changes
            # while the arm, camera, event and recording timelines keep running.
            for index, jump in enumerate((-5_000_000_000, 20_000_000_000, 0, 0)):
                wall.return_value += jump
                monotonic.return_value += 50_000_000
                arm.joints[0] = 0.01 * (index + 1)
                controller.tick(0.8 + index * 0.05)
                snapshot = controller.get_status()
                self.assertLess(acquisition_ns() - snapshot["published_ns"], 1_000)
                session.sample(snapshot, camera.latest())
            monotonic.return_value += 50_000_000
            controller.tick(1.0)
            session.toggle_push(controller.get_status(), camera.latest())
            controller.reset_to_observation(1.1)
            controller.tick(1.11)
            controller.tick(1.7)
            session.capture(controller.get_status(), camera.latest())
            segment = session.finish_segment()
            session.end_task()
        metadata = json.loads((segment / "meta.json").read_text())
        self.assertIn("monotonic_ns", metadata["timestamp_basis"])
        result = export_dataset(
            self.root / "raw", self.root / "export", allow_simulated=True
        )
        self.assertEqual(result["exported"], 1)
        self.assertEqual(result["segments"][0]["quality"]["review_flags"], [])

    def test_explicit_out_of_order_stamps_remain_rejected(self):
        from airbot_ie.push_wiper.recording import SegmentWriter

        writer = SegmentWriter(self.root, {})
        snapshot = {"state": "following", "samples": {"follow": MockArm().read()}}
        frame = MockCamera(CollectionConfig().camera).latest()
        try:
            writer.append(snapshot, frame, "pushing", 100)
            for stamp in (100, 99):
                with self.assertRaisesRegex(ValueError, f"current={stamp}, previous=100"):
                    writer.append(snapshot, frame, "pushing", stamp)
            self.assertEqual(writer.metadata["sample_count"], 1)
        finally:
            writer.finish("incomplete", "test cleanup")

    def test_round_trip_and_grouped_split(self):
        self.assert_round_trip("teleop")

    def test_single_drag_round_trip_and_grouped_split(self):
        self.assert_round_trip("single_drag")

    def assert_round_trip(self, mode):
        self.demo(mode)
        result = export_dataset(
            self.root / "raw", self.root / "export", allow_simulated=True
        )
        self.assertEqual(result["exported"], 4)
        splits = {}
        for segment in result["segments"]:
            splits.setdefault(segment["task_id"], set()).add(segment["split"])
            sample = np.load(
                self.root / "export" / segment["sample"], allow_pickle=False
            )
            self.assertEqual(sample["actions"].shape, (16, 3))
            self.assertEqual(set(np.unique(sample["mask"])), {0, 1})
            # Return to x=0.3 is outside the stroke: exported end remains pushed.
            self.assertGreater(sample["actions"][-1, 0], 0.318)
            self.assertEqual(sample["capture_pose"].shape, (7,))
            self.assertAlmostEqual(float(sample["work_height_m"]), 0.3)
            source = Path(segment["source"])
            metadata = json.loads((source / "meta.json").read_text())
            self.assertEqual(metadata["config"]["robot"]["mode"], mode)
            rows = [
                json.loads(line)
                for line in (source / "samples.jsonl").read_text().splitlines()
            ]
            self.assertTrue(all(("lead" in row) == (mode == "teleop") for row in rows))
            self.assertEqual(
                "lead" in metadata["references"]["observation"], mode == "teleop"
            )
            with (source / "raw.mcap").open("rb") as stream:
                reader = make_reader(stream)
                topics = {channel.topic for _, channel, _ in reader.iter_messages()}
                self.assertIn("/follow/eef/pose/orientation", topics)
                self.assertEqual(
                    any(topic.startswith("/lead/") for topic in topics), mode == "teleop"
                )
                self.assertFalse(any("eef/joint_state" in topic for topic in topics))
                videos = [
                    item
                    for item in reader.iter_attachments()
                    if item.media_type == "video/mp4"
                ]
                self.assertEqual(len(videos), 1)
                with av.open(BytesIO(videos[0].data)) as video:
                    frames = list(video.decode(video=0))
                    self.assertGreater(len(frames), 20)
                    image = frames[0].to_ndarray(format="bgr24")
                    self.assertGreater(int(image[240, 320, 2]), int(image[240, 320, 0]))
        self.assertTrue(all(len(value) == 1 for value in splits.values()))
        self.assertEqual(
            {next(iter(value)) for value in splits.values()}, {"train", "validation"}
        )

    def test_mock_excluded_by_default_and_output_not_overwritten(self):
        self.demo()
        result = export_dataset(self.root / "raw", self.root / "export")
        self.assertEqual(result["exported"], 0)
        self.assertEqual(result["excluded"], 4)
        with self.assertRaises(FileExistsError):
            export_dataset(self.root / "raw", self.root / "export")

    def test_export_uses_saved_observation_yaw_for_old_raw_data(self):
        self.demo()
        metadata_path = next((self.root / "raw").glob("task_*/segment_*/meta.json"))
        metadata = json.loads(metadata_path.read_text())
        metadata.pop("action_definition", None)  # Older raw segments remain usable.
        capture = metadata["references"]["observation"]["follow"]
        capture["orientation"] = [0, 0, np.sin(np.pi / 12), np.cos(np.pi / 12)]
        # Historical contact data must not affect labels or quality checks.
        metadata["references"]["contact"] = {
            "position": [0, 0, -10],
            "orientation": [0, 0, 0, 0],
        }
        atomic_json(metadata_path, metadata)
        raw_before = metadata_path.read_bytes()
        result = export_dataset(
            self.root / "raw", self.root / "export", allow_simulated=True
        )
        self.assertEqual(result["exported"], 4)
        report = next(
            row
            for row in result["segments"]
            if row["segment_id"] == metadata["segment_id"]
        )
        path = self.root / "export" / report["sample"]
        with np.load(path, allow_pickle=False) as sample:
            self.assertEqual(int(sample["action_definition_version"]), 2)
            np.testing.assert_allclose(
                sample["actions_full"][:, 2],
                2
                * np.arctan2(
                    sample["orientations_xyzw"][:, 2], sample["orientations_xyzw"][:, 3]
                )
                - np.pi / 6,
                atol=1e-12,
            )
            np.testing.assert_allclose(
                sample["capture_reference_pose"][3:], capture["orientation"]
            )
            np.testing.assert_allclose(sample["capture_pose"][3:], [0, 0, 0, 1])
            self.assertLess(sample["actions"][0, 2], -0.5)
        exported_meta = json.loads(path.with_name("meta.json").read_text())
        self.assertEqual(
            exported_meta["action_definition"]["yaw_reference"],
            "references.observation.follow.orientation",
        )
        self.assertEqual(report["quality"]["review_flags"], [])
        manifest = json.loads((self.root / "export/manifest.json").read_text())
        self.assertEqual(manifest["action_definition"]["version"], 2)
        self.assertEqual(metadata_path.read_bytes(), raw_before)

    def test_export_missing_observation_is_excluded(self):
        self.demo()
        path = next((self.root / "raw").glob("task_*/segment_*/meta.json"))
        metadata = json.loads(path.read_text())
        del metadata["references"]["observation"]
        atomic_json(path, metadata)
        result = export_dataset(
            self.root / "raw", self.root / "export", allow_simulated=True
        )
        self.assertEqual(result["exported"], 3)
        excluded = next(row for row in result["segments"] if not row["exported"])
        self.assertIn("Missing fixed observation reference O", excluded["reason"])

    def test_quality_violation_excluded_and_raw_preserved(self):
        self.demo()
        metadata_path = next((self.root / "raw").glob("task_*/segment_*/meta.json"))
        samples_path = metadata_path.parent / "samples.jsonl"
        rows = [json.loads(line) for line in samples_path.read_text().splitlines()]
        for row in rows:
            row["camera_t_ns"] -= 1_000_000_000
        samples_path.write_text("".join(json.dumps(row) + "\n" for row in rows))
        result = export_dataset(
            self.root / "raw", self.root / "export", allow_simulated=True
        )
        self.assertEqual(result["exported"], 3)
        excluded = [row for row in result["segments"] if not row["exported"]]
        self.assertIn("camera_robot_skew", excluded[0]["reason"])
        self.assertTrue((metadata_path.parent / "raw.mcap").exists())

    def test_without_contact_reference_height_and_tilt_do_not_filter_export(self):
        self.demo()
        path = next((self.root / "raw").glob("task_*/segment_*/meta.json"))
        metadata = json.loads(path.read_text())
        self.assertNotIn("contact", metadata["references"])
        self.assertNotIn("contact_z_tolerance_m", metadata["config"])
        samples_path = path.parent / "samples.jsonl"
        rows = [json.loads(line) for line in samples_path.read_text().splitlines()]
        for row in rows:
            row["follow"]["position"][2] += 0.1
            row["follow"]["orientation"] = [np.sin(np.pi / 6), 0, 0, np.cos(np.pi / 6)]
        samples_path.write_text("".join(json.dumps(row) + "\n" for row in rows))
        result = export_dataset(
            self.root / "raw", self.root / "export", allow_simulated=True
        )
        self.assertEqual(result["exported"], 4)
        report = next(
            row
            for row in result["segments"]
            if row["segment_id"] == metadata["segment_id"]
        )
        self.assertEqual(report["quality"]["review_flags"], [])
        self.assertNotIn("max_contact_z_error_m", report["quality"])
        self.assertNotIn("max_contact_tilt_deg", report["quality"])
        with np.load(
            self.root / "export" / report["sample"], allow_pickle=False
        ) as sample:
            np.testing.assert_allclose(sample["positions_full"][:, 2], 0.4)
            np.testing.assert_allclose(
                sample["orientations_xyzw"][:, 0], np.sin(np.pi / 6)
            )

    def test_legacy_reference_file_loads_without_contact_validation(self):
        config = CollectionConfig()
        path = self.root / "references.json"
        sample = MockArm().read()
        observation = {"lead": sample, "follow": sample}
        legacy_bindings = bindings(config, True)
        legacy_bindings.pop("mode")
        atomic_json(
            path,
            {
                "schema_version": 1,
                "bindings": legacy_bindings,
                "observation": observation,
                "contact": {"invalid": True},
            },
        )
        before = path.read_bytes()
        references = load_references(path, config, True)
        self.assertEqual(references["observation"], observation)
        self.assertNotIn("contact", references)
        self.assertEqual(path.read_bytes(), before)

    def test_work_height_persists_and_rejects_invalid_saved_values(self):
        config = CollectionConfig()
        path = self.root / "references.json"
        references = {
            "schema_version": 1,
            "bindings": bindings(config, True),
            "work_height": WorkHeightReference(z_m=0.0, t_ns=123).model_dump(),
        }
        atomic_json(path, references)
        loaded = load_references(path, config, True)
        self.assertEqual(loaded["work_height"], references["work_height"])
        references["work_height"]["z_m"] = "not-a-height"
        atomic_json(path, references)
        with self.assertRaises(ValueError):
            load_references(path, config, True)
        with self.assertRaises(ValueError):
            WorkHeightReference(z_m=float("nan"), t_ns=123)

    def test_legacy_segments_without_work_height_still_export(self):
        self.demo()
        path = next((self.root / "raw").glob("task_*/segment_*/meta.json"))
        metadata = json.loads(path.read_text())
        del metadata["references"]["work_height"]
        atomic_json(path, metadata)
        result = export_dataset(
            self.root / "raw", self.root / "export", allow_simulated=True
        )
        self.assertEqual(result["exported"], 4)
        for row in result["segments"]:
            sample_path = self.root / "export" / row["sample"]
            has_height = row["segment_id"] != metadata["segment_id"]
            with np.load(sample_path, allow_pickle=False) as sample:
                self.assertEqual("work_height_m" in sample, has_height)
            exported_meta = json.loads(sample_path.with_name("meta.json").read_text())
            self.assertEqual(exported_meta["work_height_available"], has_height)

    def test_references_reject_camera_changes(self):
        config = CollectionConfig()
        path = self.root / "references.json"
        atomic_json(path, {"schema_version": 1, "bindings": bindings(config, True)})
        load_references(path, config, True)
        config = CollectionConfig(camera={"width": 848})
        with self.assertRaises(ValueError):
            load_references(path, config, True)

    def test_endpoint_lock(self):
        endpoint = "test:" + self.temp.name
        with lock_robot_endpoints([endpoint]), self.assertRaises(RuntimeError):
            lock_robot_endpoints([endpoint])
        with lock_robot_endpoints([endpoint]):
            pass

    def test_single_drag_reference_reuse_and_mode_isolation(self):
        config = CollectionConfig(robot={"mode": "single_drag"})
        path = self.root / "references.json"
        reference = {
            "schema_version": 1,
            "bindings": bindings(config, True),
            "observation": {"follow": MockArm().read()},
            "work_height": WorkHeightReference(z_m=0.2, t_ns=123).model_dump(),
        }
        atomic_json(path, reference)
        self.assertEqual(load_references(path, config, True), reference)
        # An unused leader endpoint is neither bound nor connected in drag mode.
        config.robot.lead_port = config.robot.follow_port
        self.assertEqual(load_references(path, config, True), reference)
        with self.assertRaisesRegex(ValueError, "Reference configuration differs"):
            load_references(path, CollectionConfig(), True)
        atomic_json(
            path, {"schema_version": 1, "bindings": bindings(CollectionConfig(), True)}
        )
        with self.assertRaisesRegex(ValueError, "Reference configuration differs"):
            load_references(path, config, True)

    def test_real_entry_single_drag_only_opens_follow_endpoint(self):
        config = CollectionConfig(
            robot={
                "mode": "single_drag", "follow_url": "test-robot", "follow_port": 50123
            },
            camera={"serial": "test-camera"},
            references=str(self.root / "references.json"),
            output=str(self.root / "raw"),
        )
        module = "airbot_ie.push_wiper.collect."
        arm = MockArm()
        with (
            patch.dict("os.environ", {"DISPLAY": ":test"}),
            patch(module + "SDKArm", return_value=arm) as sdk,
            patch(module + "lock_robot_endpoints") as locks,
            patch(module + "CameraWorker", side_effect=MockCamera),
            patch(module + "cv2.namedWindow"),
            patch(module + "cv2.imshow"),
            patch(module + "cv2.waitKeyEx", return_value=27),
            patch(module + "cv2.destroyAllWindows"),
            redirect_stdout(StringIO()),
        ):
            run_collection(config)
        sdk.assert_called_once_with("test-robot", 50123, config.robot.feedback_timeout_s)
        locks.assert_called_once_with(["test-robot:50123"])
        self.assertEqual(arm.mode, "servo")  # Esc requests a hold.

    def test_cli_mode_overrides_config_and_validates_endpoints(self):
        path = self.root / "config.json"
        original = CollectionConfig(sample_hz=15, camera={"serial": "camera"})
        atomic_json(path, original.model_dump(mode="json"))
        output = StringIO()
        with redirect_stdout(output):
            self.assertEqual(
                main(["--config", str(path), "--mode", "single_drag", "--check-config"]),
                0,
            )
        actual = json.loads(output.getvalue())
        self.assertEqual(actual["robot"].pop("mode"), "single_drag")
        expected = original.model_dump(mode="json")
        expected["robot"].pop("mode")
        self.assertEqual(actual, expected)
        config = CollectionConfig(robot={"mode": "single_drag", "follow_port": 50050})
        self.assertEqual(config.robot.arm_names, ("follow",))
        with self.assertRaises(ValueError):
            CollectionConfig(robot={"follow_port": 50050})


if __name__ == "__main__":
    unittest.main()
