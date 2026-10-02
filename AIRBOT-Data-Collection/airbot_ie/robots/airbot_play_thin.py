import airbot_hardware_py
import numpy as np
import time
from typing import List, Optional
from enum import Enum
from airdc.common.utils.interpolate import Interpolate
from airdc.basis import Bcolors
from logging import getLogger
from threading import Thread


class MotorControlMode(int, Enum):
    INVALID = 0x00
    MIT = 0x01  # MIT mode
    CSP = 0x02  # Cyclic Synchronous Position mode
    CSV = 0x03  # Cyclic Synchronous Velocity mode
    PVT = 0x04  # Cyclic Synchronous Position mode with Current Threshold


class SpeedProfile:
    FAST = "fast"
    SLOW = "slow"


class RobotMode:
    GRAVITY_COMP = "gravity_comp"
    PLANNING_POS = "planning_pos"
    SERVO_JOINT_POS = "servo_joint_pos"


class AIRBOTArm:
    def __init__(
        self, url: str, port: int, motor_types: List[str], frequency: int = 1000
    ):
        assert len(motor_types) == 7, (
            "There should be 7 motors for the AIRBOT arm with eef."
        )
        self._motors = []
        self._executors = []
        self.get_logger().info(
            Bcolors.blue(
                f"Connecting to AIRBOT arm at {url}{port} with frequency {frequency}Hz."
            )
        )
        for index, motor_type in enumerate(motor_types):
            executor = airbot_hardware_py.create_asio_executor(1)
            motor = airbot_hardware_py.Motor.create(
                getattr(airbot_hardware_py.MotorType, motor_type), index + 1
            )
            assert motor is not None, (
                f"Motor type {motor_type} is not supported in index {index}."
            )
            motor.init(executor.get_io_context(), f"{url}{port}", frequency)
            self._motors.append(motor)
            self._executors.append(executor)
        self._motor_types = motor_types
        self._freq = frequency
        self._connected = False

    def connect(self) -> bool:
        self._connected = True
        for index, motor in enumerate(self._motors):
            self.get_logger().info(
                Bcolors.blue(
                    f"Connecting to motor {index + 1} of type {self._motor_types[index]}."
                )
            )
            if not motor.enable():
                return False
            time.sleep(0.2)
            # init internal state
            if not self._set_control_mode(motor, "MIT"):
                return False
            motor.mit(airbot_hardware_py.MotorCommand())
            if not self._set_control_mode(motor, "PVT"):
                return False
        self._target_arm_cmd = self.get_joint_pos()
        self._target_eef_cmd = self.get_eef_pos()
        self._motor_thread = Thread(target=self._control_loop, daemon=True)
        self._motor_thread.start()
        self.get_logger().info(Bcolors.GREEN + "Connected to AIRBOT arm.")
        return True

    def _set_control_mode(self, motor, mode: str) -> bool:
        mode_value = getattr(airbot_hardware_py.MotorControlMode, mode.upper()).value
        if not motor.set_param(
            "control_mode",
            airbot_hardware_py.ParamValue(
                airbot_hardware_py.ParamType.UINT16_LE,
                mode_value,
            ),
        ):
            self.get_logger().error(
                f"Failed to set control mode {mode} ({mode_value})."
            )
            return False
        return True

    def _control_loop(self):
        period = 1 / 250
        start = time.monotonic()
        while self._connected:
            # self._servo_joint_pos(self._target_arm_cmd)
            self._servo_eef_pos(self._target_eef_cmd)
            sleep_time = period - (time.monotonic() - start)
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                self.get_logger().warning(
                    f"Control loop is taking too long (exceeds {-sleep_time}s), skipping sleep."
                )
            start = time.monotonic()
        self.get_logger().info("Exited control loop")

    def disconnect(self) -> bool:
        self._connected = False
        self._motor_thread.join()
        # for motor in self._motors:
        #     motor.disable()
        #     motor.uninit()
        return True

    def switch_mode(self, mode: RobotMode):
        self._current_mode = mode
        return True

    def get_control_mode(self):
        return self._current_mode

    def set_speed_profile(self, speed_profile: SpeedProfile):
        self._speed = speed_profile

    def servo_joint_pos(self, position):
        self._target_arm_cmd = position

    def _servo_joint_pos(self, position):
        return self._perform_state("arm", "pvt", position)

    def move_to_joint_pos(self, position):
        duration = 2  # seconds
        way_points = Interpolate.way_points_interpolate(
            (self.get_joint_pos(), position), np.array([0, duration]), freq=self._freq
        )
        start = time.monotonic()
        period = 1 / self._freq
        self.get_logger().info(f"{way_points[0]}")
        self.get_logger().info(f"{way_points[-1]}")
        self.get_logger().info(f"number: {len(way_points)}")
        # input("Press Enter to start moving to joint position...")
        for way_point in way_points:
            self.servo_joint_pos(way_point)
            sleep_time = period - (time.monotonic() - start)
            if sleep_time > 0:
                time.sleep(sleep_time)
        print(f"Move to joint position completed in {time.monotonic() - start}s.")

    def servo_eef_pos(self, position):
        self._target_eef_cmd = position

    def _servo_eef_pos(self, position):
        return self._perform_state("eef", "pvt", position)

    def get_product_info(self):
        return {
            "product_type": "play",
            "eef_types": ["E2B"],
            "arm_joint_names": [f"joint{i}" for i in range(1, 7)],
            "eef_joint_names": ["arm_eef_gripper_joint"],
        }

    def get_joint_pos(self):
        return self._perform_state("arm", "pos")

    def get_eef_pos(self):
        return self._perform_state("eef", "pos")

    def get_joint_vel(self):
        return self._perform_state("arm", "vel")

    def get_eef_vel(self):
        return self._perform_state("eef", "vel")

    def get_joint_eff(self):
        return self._perform_state("arm", "eff")

    def get_eef_eff(self):
        return self._perform_state("eef", "eff")

    def get_logger(self):
        return getLogger(self.__class__.__name__)

    def _perform_state(
        self, comp: str, field: str = "", cmd: Optional[float] = None
    ) -> List[float]:
        if comp == "eef":
            slc = slice(6, 7)
        else:
            slc = slice(0, 6)
        motors = self._motors[slc]
        if cmd is None:
            return [getattr(motor.state(), field) for motor in motors]
        else:
            for index, motor in enumerate(motors):
                self.get_logger().info(f"Set {comp} {field} {cmd[index]}")
                getattr(motor, field)(
                    airbot_hardware_py.MotorCommand(
                        pos=cmd[index],
                        vel=0.2,
                        eff=0.2,
                        current_threshold=1.0,
                    )
                )


if __name__ == "__main__":
    from airdc.utils import init_logging
    import logging

    init_logging(logging.INFO)

    arm = AIRBOTArm(
        url="can",
        port=0,
        motor_types=["OD"] * 3 + ["ODM"] * 4,
        frequency=1000,
    )
    # connect to the arm
    if not arm.connect():
        raise RuntimeError("Failed to connect to the AIRBOT arm.")

    # get current state
    print(arm.get_joint_pos())
    print(arm.get_joint_vel())
    print(arm.get_joint_eff())
    print(arm.get_eef_pos())
    print(arm.get_eef_vel())
    print(arm.get_eef_eff())

    # set speed profile
    arm.set_speed_profile(SpeedProfile.FAST)

    # set eef pos
    arm.servo_eef_pos([-1.79])
    input("Press Enter to continue...")

    # set planning position cmd
    # arm.switch_mode(RobotMode.PLANNING_POS)
    # arm.move_to_joint_pos([0.0] * 6)

    # set servo position cmd
    # arm.switch_mode(RobotMode.SERVO_JOINT_POS)
    # arm.servo_joint_pos([0.0] * 6)

    # disconnect from the arm
    print("Disconnecting from the AIRBOT arm...")
    arm.disconnect()
