import tempfile
import time
import types
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np

from airbot_ie.force_control.core import (
    AdmittanceConfig,
    AdmittanceController,
    DragYSweepTrajectory,
    FixedXYTrajectory,
    NpzPlanarTrajectory,
    TrajectoryPoint,
    fz_magnitude_from_wrench,
    normal_force_from_fz,
    normal_force_from_wrench,
    quaternion_to_matrix,
    quaternion_slerp,
)
from airbot_ie.force_control.runner import HybridRunner, RunnerConfig
from airbot_ie.force_control.hardware import AirbotPlayAdapter


class FakeReader:
    def __init__(self, robot=None):
        self.robot = robot
        self.zeroed = False
        self.zero_count = 0
        self.read_count = 0
        self.closed = False

    def zero(self):
        self.zeroed = True
        self.zero_count += 1

    def read(self):
        time.sleep(0.0001)
        self.read_count += 1
        # The first thousand samples represent an unloaded zeroing interval.  The
        if self.read_count <= 1000:
            force = np.zeros(3)
        else:
            orientation = (
                self.robot.orientation
                if self.robot is not None
                else np.array([0.0, 0.0, 0.0, 1.0])
            )
            inward_tool = quaternion_to_matrix(orientation).T @ np.array(
                [0.0, 0.0, -1.0]
            )
            # Simulate a 1 N upward base-frame contact force in sensor axes.
            force = -inward_tool
        return tuple(np.r_[force, [0.0, 0.0, 0.0]])

    def close(self):
        self.closed = True


class FakeRobot:
    def __init__(self):
        self.position = np.array([0.3, 0.0, 0.2], dtype=float)
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self.connected = False
        self.closed = False
        self.commands = []
        self.servo_starts = 0

    def connect(self):
        self.connected = True

    def read_pose(self):
        return self.position.copy(), self.orientation.copy()

    def move_to_pose(self, pose):
        self.position = np.asarray(pose[0], dtype=float).copy()
        self.orientation = np.asarray(pose[1], dtype=float).copy()
        self.commands.append(("move", self.position.copy()))
        return True

    def start_pose_servo(self):
        self.servo_starts += 1
        return True

    def send_pose(self, pose):
        self.position = np.asarray(pose[0], dtype=float).copy()
        self.orientation = np.asarray(pose[1], dtype=float).copy()
        self.commands.append(("servo", self.position.copy()))
        return True

    def stop(self):
        pass

    def close(self):
        self.closed = True


class ForceHybridControlTests(unittest.TestCase):
    def test_airbot_adapter_applies_force_control_servo_params_on_connect(self):
        sdk = MagicMock()
        sdk.connect.return_value = True
        sdk_class = MagicMock(return_value=sdk)
        fake_arm = types.ModuleType("airbot_py.arm")
        fake_arm.AIRBOTPlay = sdk_class
        fake_arm.RobotMode = object()
        fake_package = types.ModuleType("airbot_py")
        fake_package.arm = fake_arm
        with patch.dict(
            "sys.modules",
            {"airbot_py": fake_package, "airbot_py.arm": fake_arm},
        ):
            adapter = AirbotPlayAdapter()

            adapter.connect()

            sdk.set_params.assert_called_once_with(
                {
                    "servo_node.moveit_servo.scale.linear": 0.1,
                    "servo_node.moveit_servo.scale.rotational": 0.3,
                    "servo_node.moveit_servo.scale.joint": 0.1,
                    "sdk_server.max_velocity_scaling_factor": 0.5,
                    "sdk_server.max_acceleration_scaling_factor": 0.1,
                }
            )
            sdk.set_speed_profile.assert_not_called()

    def test_pose_feedback_retransmits_target_until_feedback_catches_up(self):
        class DelayedFeedbackRobot(FakeRobot):
            def __init__(self):
                super().__init__()
                self.send_count = 0

            def send_pose(self, pose):
                self.send_count += 1
                self.commands.append(("servo", np.asarray(pose[0], dtype=float).copy()))
                # Simulate a servo controller that needs multiple target
                # updates before its feedback reaches the requested pose.
                if self.send_count >= 3:
                    self.position = np.asarray(pose[0], dtype=float).copy()
                    self.orientation = np.asarray(pose[1], dtype=float).copy()
                return True

        robot = DelayedFeedbackRobot()
        runner = HybridRunner(
            robot,
            FakeReader(robot),
            FixedXYTrajectory(robot.position[:2], robot.orientation),
            RunnerConfig(
                control_hz=100.0,
                pose_feedback_timeout_s=0.2,
                pose_position_tolerance_m=1e-6,
                pose_orientation_tolerance_deg=0.01,
                log_path=None,
            ),
        )
        target_position = np.array([0.31, -0.02, 0.24])
        target_orientation = np.array(
            [0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)]
        )

        runner._wait_for_pose_feedback(
            target_position, target_orientation, "delayed target"
        )

        self.assertGreaterEqual(robot.send_count, 3)
        np.testing.assert_allclose(robot.position, target_position)
        np.testing.assert_allclose(robot.orientation, target_orientation)

    def test_force_hold_xy_feedback_retransmits_until_xy_tolerance(self):
        class DelayedXYRobot(FakeRobot):
            def __init__(self):
                super().__init__()
                self.send_count = 0

            def send_pose(self, pose):
                self.send_count += 1
                target_position, target_orientation = pose
                self.commands.append(("servo", np.asarray(target_position).copy()))
                if self.send_count >= 3:
                    self.position = np.asarray(target_position, dtype=float).copy()
                    self.orientation = np.asarray(target_orientation, dtype=float).copy()
                return True

        robot = DelayedXYRobot()
        runner = HybridRunner(
            robot,
            FakeReader(robot),
            FixedXYTrajectory(robot.position[:2], robot.orientation),
            RunnerConfig(
                control_hz=100,
                force_hold_xy_tolerance_m=0.02,
                force_hold_xy_feedback_timeout_s=0.2,
                log_path=None,
            ),
        )
        runner._wait_for_force_hold_xy_feedback(
            np.array([0.4, 0.1, 0.2]),
            robot.orientation,
        )
        self.assertGreaterEqual(robot.send_count, 3)

    def test_vertical_orientation_feedback_checks_only_tcp_x_axis(self):
        class RollLagRobot(FakeRobot):
            def send_pose(self, pose):
                self.commands.append(("servo", np.asarray(pose[0], dtype=float).copy()))
                return True

        robot = RollLagRobot()
        ten_degrees = np.deg2rad(10.0) / 2.0
        robot.orientation = np.array(
            [np.sin(ten_degrees), 0.0, 0.0, np.cos(ten_degrees)]
        )
        runner = HybridRunner(
            robot,
            FakeReader(robot),
            FixedXYTrajectory(robot.position[:2], [0.0, 0.0, 0.0, 1.0]),
            RunnerConfig(
                control_hz=100,
                pose_feedback_timeout_s=0.05,
                pose_orientation_tolerance_deg=8.0,
                vertical_axis_tolerance_deg=12.0,
                log_path=None,
            ),
        )
        runner._wait_for_pose_feedback(
            robot.position,
            np.array([0.0, 0.0, 0.0, 1.0]),
            "vertical axis",
            orientation_axis=0,
            orientation_axis_tolerance_deg=12.0,
        )

    def test_quaternion_slerp_uses_shortest_normalized_path(self):
        start = np.array([0.0, 0.0, 0.0, 1.0])
        end = np.array([0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)])
        middle = quaternion_slerp(start, end, 0.5)
        self.assertAlmostEqual(np.linalg.norm(middle), 1.0, places=12)
        self.assertAlmostEqual(middle[2], np.sin(np.pi / 8), places=12)
        self.assertAlmostEqual(middle[3], np.cos(np.pi / 8), places=12)

    def test_normal_offset_does_not_accumulate_across_planar_modes(self):
        class LaggingRobot(FakeRobot):
            def send_pose(self, pose):
                self.position += 0.2 * (pose[0] - self.position)
                return True

        for robot_type in (FakeRobot, LaggingRobot):
            for mode in ("fixed_xy", "drag_y_sweep", "npz"):
                with self.subTest(robot=robot_type.__name__, mode=mode):
                    robot = robot_type()
                    if mode == "fixed_xy":
                        trajectory = FixedXYTrajectory(robot.position[:2], robot.orientation)
                    elif mode == "drag_y_sweep":
                        trajectory = DragYSweepTrajectory(
                            robot.position.copy(), robot.orientation, duration_s=2.0
                        )
                    else:
                        trajectory = NpzPlanarTrajectory(
                            np.array([[0.3, 0.0], [0.3, 0.1]]),
                            np.array([0.0, 2.0]),
                            0.2,
                            robot.orientation,
                        )
                    trajectory.set_z_reference(0.2)
                    runner = HybridRunner(
                        robot, FakeReader(), trajectory, RunnerConfig(log_path=None)
                    )
                    # A constant 0.5 N error balances the virtual spring at 1 mm.
                    runner.admittance.delta_m = 0.001
                    for step in range(200):
                        delta = runner.admittance.update(0.5, 0.01)
                        current, _ = robot.read_pose()
                        command = runner._send(trajectory.sample(step * 0.01), current, delta)
                        self.assertAlmostEqual(command[2], 0.199, places=12)
                    self.assertFalse(runner.admittance.displacement_limited)

    def test_normal_offset_changes_and_reset_return_to_reference(self):
        robot = FakeRobot()
        trajectory = FixedXYTrajectory(robot.position[:2], robot.orientation)
        trajectory.set_z_reference(0.2)
        runner = HybridRunner(robot, FakeReader(), trajectory, RunnerConfig(log_path=None))
        for delta in (0.001, 0.002, 0.0005, -0.001):
            current, _ = robot.read_pose()
            command = runner._send(trajectory.sample(0.0), current, delta)
            self.assertAlmostEqual(command[2], 0.2 - delta, places=12)
        runner.admittance.reset()
        # Feedback disturbances must not redefine the normal reference.
        robot.position[2] += 0.005
        command = runner._send(
            trajectory.sample(0.0), robot.position.copy(), runner.admittance.delta_m
        )
        self.assertAlmostEqual(command[2], 0.2, places=12)

    def test_constant_force_error_produces_bounded_normal_position(self):
        for force_error in (0.5, -0.5):
            with self.subTest(force_error=force_error):
                robot = FakeRobot()
                trajectory = FixedXYTrajectory(robot.position[:2], robot.orientation)
                trajectory.set_z_reference(0.2)
                config = RunnerConfig(log_path=None)
                runner = HybridRunner(robot, FakeReader(), trajectory, config)
                positions = []
                for _ in range(1000):
                    delta = runner.admittance.update(force_error, 0.01)
                    current, _ = robot.read_pose()
                    command = runner._send(trajectory.sample(0.0), current, delta)
                    normal_step = current[2] - command[2]
                    self.assertLessEqual(
                        normal_step, config.admittance.max_press_speed_m_s * 0.01 + 1e-12
                    )
                    self.assertGreaterEqual(
                        normal_step, -config.admittance.max_retract_speed_m_s * 0.01 - 1e-12
                    )
                    self.assertLessEqual(
                        abs(command[2] - 0.2), config.admittance.max_displacement_m
                    )
                    positions.append(command[2])
                np.testing.assert_allclose(
                    positions[-100:],
                    0.2 - force_error / config.admittance.stiffness_n_m,
                    rtol=0.0,
                    atol=1e-12,
                )

    def test_normal_projection_preserves_tangential_speed_limit(self):
        robot = FakeRobot()
        normal = np.array([0.6, 0.0, 0.8])
        tangent = np.array([0.8, 0.0, -0.6])
        reference = robot.position + tangent * 0.05
        point = TrajectoryPoint(reference, robot.orientation, normal)
        trajectory = FixedXYTrajectory(robot.position[:2], robot.orientation)
        config = RunnerConfig(log_path=None)
        runner = HybridRunner(robot, FakeReader(), trajectory, config)
        for _ in range(10):
            current, _ = robot.read_pose()
            command = runner._send(point, current, 0.001)
            self.assertAlmostEqual(np.dot(command - reference, normal), -0.001, places=12)
            self.assertAlmostEqual(
                np.dot(command - current, tangent),
                config.max_tangential_speed_m_s / config.control_hz,
                places=12,
            )

    def test_negative_sensor_fz_becomes_positive_normal_force(self):
        self.assertAlmostEqual(normal_force_from_fz(-3.2, 0.1, -1), 3.3)
        self.assertAlmostEqual(normal_force_from_fz(0.1, 0.1, -1), 0.0)

    def test_contact_force_uses_absolute_bias_corrected_fz(self):
        wrench = np.array([1.0, -2.0, -5.6, 0.0, 0.0, 0.0])
        bias = np.array([0.0, 0.0, -0.6, 0.0, 0.0, 0.0])
        self.assertAlmostEqual(fz_magnitude_from_wrench(wrench, bias), 5.0)

    def test_tilted_tool_projects_contact_force_from_all_force_axes(self):
        orientation = np.array([-0.050871, 0.733857, 0.029987, 0.676732])
        from airbot_ie.force_control.core import quaternion_to_matrix

        inward_tool = quaternion_to_matrix(orientation).T @ np.array([0.0, 0.0, -1.0])
        wrench = np.r_[-3.0 * inward_tool, [0.0, 0.0, 0.0]]
        self.assertAlmostEqual(
            normal_force_from_wrench(
                wrench,
                np.zeros(6),
                orientation,
                force_sign=-1,
            ),
            3.0,
            places=6,
        )

    def test_second_order_admittance_and_limits(self):
        controller = AdmittanceController(
            AdmittanceConfig(
                mass_kg=0.05,
                damping_ns_m=10.0,
                stiffness_n_m=500.0,
                max_press_speed_m_s=0.003,
                max_retract_speed_m_s=0.008,
                max_displacement_m=0.02,
            )
        )
        first = controller.update(5.0, 0.01)
        self.assertAlmostEqual(first, 0.00003, places=8)
        for _ in range(1000):
            controller.update(5.0, 0.01)
        self.assertLessEqual(controller.delta_m, 0.02)
        self.assertGreater(controller.delta_m, 0.0)
        # At steady state the spring term balances the force error: delta=F/k.
        self.assertAlmostEqual(controller.delta_m, 5.0 / 500.0, places=4)
        for _ in range(1000):
            controller.update(-100.0, 0.01)
        self.assertGreaterEqual(controller.delta_m, -0.02)

    def test_fixed_xy_trajectory(self):
        trajectory = FixedXYTrajectory([0.2, -0.1], [0, 0, 0, 1])
        trajectory.set_z_reference(0.15)
        point = trajectory.sample(3.0)
        np.testing.assert_allclose(point.position_base, [0.2, -0.1, 0.15])
        np.testing.assert_allclose(point.normal_base, [0, 0, 1])

    def test_drag_y_sweep_returns_to_dragged_start(self):
        trajectory = DragYSweepTrajectory(
            [0.4, -0.2, 0.3], [0.0, 0.0, 0.0, 1.0], distance_m=0.1, speed_m_s=0.01
        )
        trajectory.set_z_reference(0.12)
        start = trajectory.sample(0.0)
        middle = trajectory.sample(15.0)
        end = trajectory.sample(30.0)
        np.testing.assert_allclose(start.position_base, [0.4, -0.2, 0.12])
        np.testing.assert_allclose(middle.position_base, [0.4, -0.1, 0.12])
        np.testing.assert_allclose(end.position_base, [0.4, -0.2, 0.12])
        self.assertAlmostEqual(trajectory.duration_s, 30.0)
        from airbot_ie.force_control.core import quaternion_to_matrix

        np.testing.assert_allclose(
            quaternion_to_matrix(middle.orientation_xyzw)[:, 2], [0.0, 0.0, -1.0]
        )

    def test_drag_to_pose_enters_gravity_and_returns_confirmed_pose(self):
        class FakeModes:
            GRAVITY_COMP = "gravity"
            PLANNING_POS = "planning"

        class FakeSdk:
            def __init__(self):
                self.modes = []

            def switch_mode(self, mode):
                self.modes.append(mode)
                return True

            def get_end_pose(self):
                return [[0.4, -0.2, 0.3], [0.0, 0.0, 0.0, 1.0]]

        adapter = AirbotPlayAdapter()
        adapter.sdk = FakeSdk()
        adapter.robot_mode = FakeModes
        with patch("builtins.input", return_value=""):
            position, orientation = adapter.drag_to_pose("drag")
        self.assertEqual(adapter.sdk.modes, ["gravity", "planning"])
        np.testing.assert_allclose(position, [0.4, -0.2, 0.3])
        np.testing.assert_allclose(orientation, [0.0, 0.0, 0.0, 1.0])

    def test_drag_y_sweep_preserves_dragged_orientation_when_alignment_disabled(self):
        orientation = np.array([0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)])
        trajectory = DragYSweepTrajectory(
            [0.4, -0.2, 0.3],
            orientation,
            distance_m=0.1,
            speed_m_s=0.01,
            align_tool_z=False,
        )
        trajectory.set_z_reference(0.12)
        np.testing.assert_allclose(
            trajectory.sample(15.0).orientation_xyzw, orientation
        )

    def test_npz_planar_trajectory_replays_yaw(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sample.npz"
            np.savez(
                path,
                actions=np.array([[0.1, 0.2, 0.0], [0.2, 0.3, np.pi / 2]]),
                timestamps_ns=np.array([10, 20], dtype=np.int64),
                capture_reference_pose=np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0]),
            )
            trajectory = NpzPlanarTrajectory.from_npz(
                path, 1.0, 0.1, align_tool_z=False
            )
            point = trajectory.sample(1.0)
            np.testing.assert_allclose(point.position_base, [0.2, 0.3, 0.1])
            self.assertAlmostEqual(point.orientation_xyzw[2], np.sqrt(0.5), places=6)

    def test_npz_can_replay_exported_full_orientation_with_slerp(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sample.npz"
            np.savez(
                path,
                actions=np.array([[0.1, 0.2, 0.0], [0.2, 0.3, 0.0]]),
                timestamps_ns=np.array([10, 20], dtype=np.int64),
                capture_reference_pose=np.array(
                    [0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0]
                ),
                orientations_xyzw=np.array(
                    [
                        [0.0, 0.0, 0.0, 1.0],
                        [0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)],
                    ]
                ),
            )
            trajectory = NpzPlanarTrajectory.from_npz(
                path,
                1.0,
                0.1,
                replay_full_orientation=True,
            )
            middle = trajectory.sample(0.5).orientation_xyzw
            np.testing.assert_allclose(
                middle,
                [0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8)],
                atol=1e-12,
            )

    def test_npz_raw_replay_exposes_vertical_tool_alignment_orientation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sample.npz"
            np.savez(
                path,
                actions=np.array([[0.1, 0.2, 0.0], [0.2, 0.3, 0.0]]),
                capture_reference_pose=np.array(
                    [0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0]
                ),
                orientations_xyzw=np.array(
                    [
                        [0.0, 0.0, 0.0, 1.0],
                        [0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)],
                    ]
                ),
            )
            trajectory = NpzPlanarTrajectory.from_npz(
                path,
                1.0,
                0.1,
                replay_full_orientation=True,
                tool_normal_axis="x",
            )
            vertical = quaternion_to_matrix(
                trajectory.vertical_alignment_orientation()
            )
            np.testing.assert_allclose(vertical[:, 0], [0.0, 0.0, -1.0], atol=1e-12)

    def test_npz_default_orientation_points_tcp_z_into_planar_surface(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sample.npz"
            np.savez(
                path,
                actions=np.array([[0.1, 0.2, 0.0], [0.2, 0.3, np.pi / 2]]),
                capture_reference_pose=np.array(
                    [0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0]
                ),
            )
            trajectory = NpzPlanarTrajectory.from_npz(path, 1.0, 0.1)
            point = trajectory.sample(1.0)
            from airbot_ie.force_control.core import quaternion_to_matrix

            np.testing.assert_allclose(
                quaternion_to_matrix(point.orientation_xyzw)[:, 2], [0.0, 0.0, -1.0]
            )

    def test_npz_can_align_tcp_x_for_tools_mounted_along_x(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sample.npz"
            np.savez(
                path,
                actions=np.array([[0.1, 0.2, 0.0], [0.2, 0.3, 0.0]]),
                capture_reference_pose=np.array(
                    [0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0]
                ),
            )
            trajectory = NpzPlanarTrajectory.from_npz(
                path, 1.0, 0.1, tool_normal_axis="x"
            )
            rotation = quaternion_to_matrix(trajectory.sample(0.0).orientation_xyzw)
            np.testing.assert_allclose(rotation[:, 0], [0.0, 0.0, -1.0], atol=1e-12)
            self.assertAlmostEqual(rotation[2, 2], 0.0, places=12)

    def test_npz_uses_exported_work_height_when_surface_is_unspecified(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sample.npz"
            np.savez(
                path,
                actions=np.array([[0.1, 0.2, 0.0], [0.2, 0.3, 0.0]]),
                capture_reference_pose=np.array(
                    [0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0]
                ),
                work_height_m=np.float64(0.25),
            )
            trajectory = NpzPlanarTrajectory.from_npz(path, 1.0, None)
            self.assertAlmostEqual(float(trajectory.sample(0).position_base[2]), 0.25)

    def test_npz_without_work_height_waits_for_runner_reference(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sample.npz"
            np.savez(
                path,
                actions=np.array([[0.1, 0.2, 0.0], [0.2, 0.3, 0.0]]),
                capture_reference_pose=np.array(
                    [0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0]
                ),
            )
            trajectory = NpzPlanarTrajectory.from_npz(path, 1.0, None)
            with self.assertRaises(RuntimeError):
                trajectory.sample(0)
            trajectory.set_z_reference(0.15)
            self.assertAlmostEqual(float(trajectory.sample(0).position_base[2]), 0.15)

    def test_npz_rejects_incompatible_action_definition(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sample.npz"
            np.savez(
                path,
                actions=np.array([[0.1, 0.2, 0.0], [0.2, 0.3, 0.0]]),
                capture_reference_pose=np.array(
                    [0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0]
                ),
                action_definition_version=np.int64(1),
            )
            with self.assertRaisesRegex(ValueError, "action_definition_version"):
                NpzPlanarTrajectory.from_npz(path, 1.0, 0.1)

    def test_mock_runner_reaches_done_and_closes_devices(self):
        robot = FakeRobot()
        reader = FakeReader(robot)
        trajectory = FixedXYTrajectory([0.3, 0.0], [0.0, 0.0, 0.0, 1.0])
        trajectory.set_z_reference(0.15)
        config = RunnerConfig(
            control_hz=100,
            target_force_n=0.5,
            force_sign=-1,
            filter_cutoff_hz=2,
            max_force_n=8,
            sensor_stale_s=0.2,
            bias_duration_s=0.02,
            sensor_poll_interval_s=0.0,
            contact_threshold_n=0.5,
            approach_timeout_s=1.0,
            approach_speed_m_s=0.002,
            approach_depth_m=0.05,
            settle_s=0.01,
            safe_clearance_m=0.02,
            retract_s=0.01,
            hold_duration_s=0.02,
            log_path=None,
        )
        runner = HybridRunner(robot, reader, trajectory, config)
        self.assertEqual(runner.run(), "DONE")
        self.assertTrue(robot.connected)
        self.assertTrue(robot.closed)
        self.assertTrue(reader.closed)
        move_positions = [position for kind, position in robot.commands if kind == "move"]
        self.assertAlmostEqual(move_positions[0][2], 0.17, places=6)
        servo_positions = [position for kind, position in robot.commands if kind == "servo"]
        self.assertTrue(servo_positions)
        # The first servo phase is the pre-contact approach.  It must actually
        # descend from the safe height before the force hold starts.
        self.assertLess(min(position[2] for position in servo_positions), 0.2)

    def test_staged_npz_preposition_moves_high_xy_then_servo_descends(self):
        class TracingRobot(FakeRobot):
            def __init__(self):
                super().__init__()
                self.move_targets = []
                self.servo_targets = []

            def move_to_pose(self, pose):
                self.move_targets.append(
                    (
                        np.asarray(pose[0], dtype=float).copy(),
                        np.asarray(pose[1], dtype=float).copy(),
                    )
                )
                return super().move_to_pose(pose)

            def send_pose(self, pose):
                self.servo_targets.append(
                    (
                        np.asarray(pose[0], dtype=float).copy(),
                        np.asarray(pose[1], dtype=float).copy(),
                    )
                )
                return super().send_pose(pose)

        robot = TracingRobot()
        reader = FakeReader()
        trajectory = NpzPlanarTrajectory(
            np.array([[0.2, -0.1], [0.21, -0.09]]),
            np.array([0.0, 1.0]),
            0.15,
            np.array([0.0, 0.0, 0.0, 1.0]),
            replay_yaw=True,
            align_tool_z=False,
        )
        trajectory.yaw = np.array([np.pi / 2.0, np.pi / 2.0])
        trajectory.capture_reference_pose = np.array(
            [0.1, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0]
        )
        config = RunnerConfig(
            control_hz=100,
            target_force_n=0.5,
            max_force_n=8.0,
            sensor_stale_s=0.2,
            bias_duration_s=0.01,
            sensor_poll_interval_s=0.0,
            contact_threshold_n=0.5,
            approach_timeout_s=1.0,
            approach_speed_m_s=0.002,
            approach_depth_m=0.05,
            settle_s=0.01,
            safe_clearance_m=0.02,
            move_to_capture_pose=True,
            staged_move_to_trajectory=True,
            orientation_transition_s=0.01,
            safe_descent_s=0.01,
            high_z_m=0.3,
            rezero_after_preposition=True,
            retract_s=0.01,
            hold_duration_s=0.02,
            log_path=None,
        )

        self.assertEqual(HybridRunner(robot, reader, trajectory, config).run(), "DONE")
        self.assertEqual(reader.zero_count, 2)
        self.assertEqual(len(robot.move_targets), 2)
        self.assertAlmostEqual(robot.move_targets[0][0][2], 0.4, places=6)
        self.assertAlmostEqual(robot.move_targets[1][0][2], 0.3, places=6)
        self.assertTrue(robot.servo_targets)
        servo_z = [target[0][2] for target in robot.servo_targets]
        self.assertAlmostEqual(servo_z[0], 0.3, places=6)
        self.assertTrue(any(abs(value - 0.17) < 1e-6 for value in servo_z))
        self.assertTrue(any(value < 0.17 for value in servo_z))

    def test_capture_pose_falls_back_to_high_clearance_after_planner_rejection(self):
        class RejectFirstMoveRobot(FakeRobot):
            def __init__(self):
                super().__init__()
                self.rejected = False

            def move_to_pose(self, pose):
                if not self.rejected:
                    self.rejected = True
                    return False
                return super().move_to_pose(pose)

        robot = RejectFirstMoveRobot()
        reader = FakeReader(robot)
        trajectory = NpzPlanarTrajectory(
            np.array([[0.2, -0.1], [0.21, -0.09]]),
            np.array([0.0, 1.0]),
            0.15,
            np.array([0.0, 0.0, 0.0, 1.0]),
            align_tool_z=False,
        )
        trajectory.capture_reference_pose = np.array(
            [0.1, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0]
        )
        config = RunnerConfig(
            control_hz=100,
            target_force_n=0.5,
            max_force_n=8.0,
            sensor_stale_s=0.2,
            bias_duration_s=0.01,
            sensor_poll_interval_s=0.0,
            contact_threshold_n=0.5,
            approach_timeout_s=1.0,
            approach_speed_m_s=0.002,
            approach_depth_m=0.05,
            settle_s=0.01,
            safe_clearance_m=0.02,
            move_to_capture_pose=True,
            staged_move_to_trajectory=True,
            orientation_transition_s=0.01,
            safe_descent_s=0.01,
            rezero_after_preposition=True,
            retract_s=0.01,
            hold_duration_s=0.02,
            log_path=None,
        )
        self.assertEqual(HybridRunner(robot, reader, trajectory, config).run(), "DONE")
        self.assertTrue(robot.rejected)
        self.assertGreaterEqual(len([c for c in robot.commands if c[0] == "move"]), 3)

    def test_drag_sweep_approach_starts_above_confirmed_z(self):
        robot = FakeRobot()
        reader = FakeReader(robot)
        trajectory = DragYSweepTrajectory(
            [0.3, 0.0, 0.2], [0.0, 0.0, 0.0, 1.0], speed_m_s=0.01
        )
        config = RunnerConfig(
            control_hz=100,
            target_force_n=0.5,
            force_sign=-1,
            filter_cutoff_hz=2,
            max_force_n=8,
            sensor_stale_s=0.2,
            bias_duration_s=0.02,
            sensor_poll_interval_s=0.0,
            contact_threshold_n=0.5,
            approach_timeout_s=1.0,
            approach_speed_m_s=0.002,
            approach_depth_m=0.05,
            settle_s=0.01,
            safe_clearance_m=0.02,
            retract_s=0.01,
            hold_duration_s=0.02,
            log_path=None,
        )
        self.assertEqual(HybridRunner(robot, reader, trajectory, config).run(), "DONE")
        move_positions = [position for kind, position in robot.commands if kind == "move"]
        self.assertAlmostEqual(move_positions[0][2], 0.22, places=6)

    def test_sensor_failure_during_precheck_does_not_retract(self):
        class FailingReader:
            def zero(self):
                pass

            def read(self):
                raise RuntimeError("sensor disconnected")

            def close(self):
                pass

        robot = FakeRobot()
        trajectory = FixedXYTrajectory([0.3, 0.0], [0.0, 0.0, 0.0, 1.0])
        trajectory.set_z_reference(0.15)
        config = RunnerConfig(
            bias_duration_s=0.01,
            zero_settle_s=0.001,
            log_path=None,
        )
        runner = HybridRunner(robot, FailingReader(), trajectory, config)
        self.assertEqual(runner.run(), "FAULT")
        self.assertEqual(robot.servo_starts, 0)


if __name__ == "__main__":
    unittest.main()
