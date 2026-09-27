from enum import Enum
from pprint import pformat
from typing import Dict
import time

from bidict import bidict
from pydantic import BaseModel

import logging
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from airdc.managers.basis import DemonstrateManagerBasis, DemonstrateAction as Action
from airdc.basis import Bcolors


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class JoyAxis(Enum):
    # LEFT_X = 0
    # LEFT_Y = 1
    # RIGHT_X = 2
    # RIGHT_Y = 3
    DPAD_X = 7
    # DPAD_Y = 7
    # TRIGGERS = 6  # L2 + R2 合并轴


class JoyButton(Enum):
    A = 0
    # B = 1
    X = 2
    Y = 3
    LB = 4
    RB = 5
    # BACK = 6
    # START = 7
    # GUIDE = 8
    # LEFT_STICK = 9
    # RIGHT_STICK = 10


class JoyCallbackConfig(BaseModel):
    action_button: Dict[Action, JoyButton] = {
        Action.sample: JoyButton.Y,
        Action.save: JoyButton.A,
        Action.remove: JoyButton.LB,
        Action.finish: JoyButton.RB,
    }
    instruction_button: Dict[str, str] = {}
    action_axis: Dict[Action, JoyAxis] = {
        Action.abandon: JoyAxis.DPAD_X,
    }
    instruction_axis: Dict[str, str] = {}

    def model_post_init(self, context):
        action_info = {
            Action.sample: "Start sampling",
            Action.save: "Save sampled data in the current episode",
            Action.abandon: "Abandon current sampling without saving",
            Action.finish: "Finish the current episode and save all data",
            Action.remove: "Remove the last saved episode",
            Action.capture: "Capture current component observations",
        }
        for action, btn in self.action_button.items():
            self.instruction_button[f"button_{btn.value}"] = action_info[action]
        for action, axis in self.action_axis.items():
            self.instruction_axis[f"axis_{axis.value}"] = action_info[action]


class JoyCallbackManager(DemonstrateManagerBasis):
    config: JoyCallbackConfig

    def __init__(self):
        super().__init__()
        self.node = None
        self.ros_initialized = False
        self._init_ros2()

        # 建立按钮到动作的双向映射
        self.button_to_action = bidict(
            {btn.value: action for action, btn in self.config.action_button.items()}
        )
        self.axis_to_action = bidict(
            {axis.value: action for action, axis in self.config.action_axis.items()}
        )

    def _init_ros2(self):
        """初始化ROS2节点和订阅"""
        try:
            rclpy.init()
            self.ros_initialized = True
            self.node = Node("mmk2_robot_subscriber")

            # 订阅joy消息
            self.joy_sub = self.node.create_subscription(
                Joy, "/joy", self.joy_callback, 10
            )

            # 启动单独的线程运行ROS2回调
            import threading

            self.ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
            self.ros_thread.start()
            logger.info("ROS2节点初始化完成")
        except Exception as e:
            logger.error(f"ROS2初始化失败: {e}")
            self.node = None
            # 重新抛出异常，让调用者处理
            raise

    def _ros_spin(self):
        """在单独的线程中运行ROS2回调"""
        try:
            while rclpy.ok() and hasattr(self, "node") and self.node:
                try:
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                except Exception as e:
                    logger.error(f"ROS2回调执行错误: {e}")
                time.sleep(0.01)
        except Exception as e:
            logger.error(f"ROS2回调线程错误: {e}")

    def on_configure(self):
        self.print_round()
        self.show_instruction()
        return True

    def update(self) -> bool:
        return True

    def show_instruction(self) -> None:
        """显示用户操作说明"""
        self.get_logger().info(
            Bcolors.cyan(f"\n{pformat(self.config.instruction_button)}")
        )

    def print_round(self):
        self.get_logger().info(
            f"Current sample episode: {self.fsm.sample_info.episode}"
        )

    def joy_callback(self, msg: Joy):
        """
        处理 Joy 消息，检查按下的按钮并触发对应动作。
        """
        for button_idx in self.button_to_action:
            if msg.buttons[button_idx] == 1:  # 按下为 1
                action = self.button_to_action[button_idx]
                print(f"Button {button_idx} pressed, triggering action: {action.name}")
                self.handle_joy_action(action)

        for axis_idx in self.axis_to_action:
            if msg.axes[axis_idx] == 1:
                action = self.axis_to_action[axis_idx]
                print(f"Axis {axis_idx} pressed, triggering action: {action.name}")
                self.handle_joy_action(action)

    def handle_joy_action(self, action: Action):
        """
        执行状态机中的动作，如采样、保存等。
        """
        self.get_logger().info(Bcolors.green(f"Triggering action: {action.name}"))
        self.fsm.act(action)
        self.print_round()

    def on_shutdown(self) -> bool:
        """
        节点关闭时执行的清理操作。
        因为 JoyCallbackManager 不使用 keyboard.Listener，所以无需额外清理。
        """
        self.get_logger().info("Shutting down JoyCallbackManager.")
        return True
