from discoverse.examples.force_control_data_collect_using_joy.joy.joy_controller import (
    JoyController,
)
from enum import IntEnum, auto
from airdc.managers.basis import (
    DemonstrateManagerBasis,
    ManagerConfigBasis,
    DemonstrateAction as Action,
)
from typing import Dict, Union
from functools import partial


class JoyEvent(IntEnum):
    BUTTON_A = 0
    BUTTON_B = auto()
    BUTTON_X = auto()
    BUTTON_Y = auto()
    BUTTON_LB = auto()
    BUTTON_RB = auto()
    BUTTON_BACK = auto()
    BUTTON_START = auto()
    BUTTON_LOGO = auto()
    BUTTON_LSTICK = auto()
    BUTTON_RSTICK = auto()
    AXIS_LT = auto()
    AXIS_RT = auto()
    AXIS_LX = auto()
    AXIS_LY = auto()
    AXIS_RX = auto()
    AXIS_RY = auto()


class JoyManagerConfig(ManagerConfigBasis):
    """Configuration for JoyManager."""

    id: int = 0
    action_key: Dict[Action, Union[JoyEvent, int]] = {
        Action.sample: JoyEvent.BUTTON_A,
        Action.save: JoyEvent.BUTTON_B,
        Action.abandon: JoyEvent.BUTTON_X,
        Action.remove: JoyEvent.BUTTON_Y,
        Action.capture: JoyEvent.BUTTON_LB,
        Action.finish: JoyEvent.BUTTON_RB,
    }
    # instruction: Dict[str, str] = {
    #     "b": "Back to sample the last episode (override the last saved file)",
    #     "i": "Show this instruction again",
    #     "g": "Switch passive (gravity composation) / resetting mode of the leaders",
    #     "f": "Start / stop following",
    #     "f2": "Lock / unlock the keyboard control",
    # }


class JoyManager(DemonstrateManagerBasis):
    """Handles joystick events for controlling data collection.

    This class listens for specific joystick inputs and triggers actions of the demontrate fsm.
    These actions include starting or stopping data collection, printing
    current component states, removing the last saved episode, etc.
    """

    config: JoyManagerConfig

    def on_configure(self):
        self.interface = JoyController(self.config.id)
        for action, key in self.config.action_key.items():
            # self.interface.add_button_down_callback(key, partial(self.fsm.act, action))
            self.interface.add_button_down_callback(
                key, partial(self._mock_callback, key, action)
            )
        return True

    def _mock_callback(self, key, action: Action):
        self.get_logger().info(f"Callback triggered: {key=}, {action=}")

    def update(self) -> bool:
        return True

    def on_shutdown(self) -> bool:
        return True


if __name__ == "__main__":
    joy_manager = JoyManager()
    assert joy_manager.configure()

    input("Press Enter to exit...\n")
    assert joy_manager.shutdown()
