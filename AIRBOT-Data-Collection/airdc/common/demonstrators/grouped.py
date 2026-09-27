from typing import Union, Optional
from airdc.common.systems.basis import Sensor, System, SystemMode
from airdc.common.demonstrators.basis import Demonstrator
from airdc.demonstrate.configs import DemonstrateAction
from airdc.common.systems.grouped import (
    GroupedComponentsSystemConfig,
    GroupedComponentsSystem,
    GroupsSendActionConfig,
)


Component = Union[System, Sensor]


class GroupedDemonstratorConfig(GroupedComponentsSystemConfig):
    """Configuration for GroupedDemonstrator"""


class GroupedDemonstrator(Demonstrator):
    config: GroupedDemonstratorConfig
    interface: GroupedComponentsSystem

    def on_configure(self):
        return self.interface.configure()

    def send_action(self, action: GroupsSendActionConfig) -> bool:
        """Control the leaders after some demonstrate action"""
        return self.interface.send_action(action)

    def capture_observation(self, timeout: Optional[float] = None):
        return self.interface.capture_observation(timeout)

    def get_info(self):
        return self.interface.get_info()

    def react(self, action):
        if action is DemonstrateAction.sample:
            return self.switch_mode(SystemMode.PASSIVE)
        elif action is DemonstrateAction.deactivate:
            return self.interface.handler.exit()
        elif action is DemonstrateAction.configure:
            return self.configure()
        else:
            func = getattr(self, f"_{action.value}", None)
            if func is not None:
                return func()
        return True

    def _activate(self) -> bool:
        # try to start auto control
        if self.interface.start_auto_control():
            # set leaders to resetting mode
            return self.switch_mode(SystemMode.RESETTING)

    def _finish(self) -> bool:
        return self.shutdown()

    def on_switch_mode(self, mode):
        return self.interface.switch_mode(mode)

    def shutdown(self) -> bool:
        return self.interface.shutdown()

    @property
    def handler(self):
        return self.interface.handler
