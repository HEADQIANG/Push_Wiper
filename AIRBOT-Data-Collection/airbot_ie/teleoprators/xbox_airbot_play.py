from discoverse.examples.force_control_data_collect_using_joy.robot_joy_controller import (
    RobotJoyController,
)
from airdc.common.systems.basis import System
from airdc.basis import Bcolors
from pydantic import BaseModel
from threading import Thread
from time import time_ns, sleep
from pprint import pformat


class XBoxAIRBOTPlayConfig(BaseModel):
    """Configuration for XBoxAIRBOTPlay."""


class XBoxAIRBOTPlay(System):
    config: XBoxAIRBOTPlayConfig
    interface: RobotJoyController

    def on_configure(self) -> bool:
        self._joy_thread = Thread(target=self.interface.start, daemon=True)
        self._joy_thread.start()
        info = {
            "right stick": "forward, backward, left, and right movements",
            "left / right trigger": "up / down movements",
            "": "Return to initial pose",
        }
        self.get_logger().info(
            Bcolors.blue("Operating Instructions:\n" + pformat(info))
        )
        while not self.interface.got_init_force:
            self.get_logger().info("Waiting for initial force reading...")
            sleep(0.5)
        return True

    def on_switch_mode(self, mode):
        return True

    def capture_observation(self, timeout=None):
        return {
            "arm/pose": {
                "t": time_ns(),
                "data": {
                    "position": self.interface.current_position,
                    "orientation": self.interface.fixed_orientation,
                },
            },
            "arm/wrench": {
                "t": time_ns(),
                "data": {
                    "force": self.interface.current_ext_force[:3],
                    "torque": self.interface.current_ext_force[3:],
                },
            },
        }

    def send_action(self, action):
        pass

    def get_info(self):
        return {}

    def shutdown(self) -> bool:
        # daemon thread will exit when main program exits
        # self._joy_thread.join()
        return True


if __name__ == "__main__":
    xbox_play = XBoxAIRBOTPlay()
    assert xbox_play.configure()
