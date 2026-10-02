from typing import Dict
import time
import rclpy
from std_srvs.srv import SetBool
from airdc.managers.basis import DemonstrateManagerBasis, DemonstrateAction as Action
from airdc.common.devices.vr.quest import (
    VRQuest,
    VRQuestConfig,
    VRControllerEvent,
)
from functools import partial


class VRConfig(VRQuestConfig):
    service_names: Dict[Action, str] = {
        Action.sample: "rec_srv",
        Action.save: "stop_rec_srv",
    }


class VRManager(DemonstrateManagerBasis):
    config: VRConfig
    interface: VRQuest

    def on_configure(self):
        if self.interface.configure():
            self.interface.register_callback(self._vr_control_callback)
            self._init_ros2()
            self.show_instruction()
            return True
        return False

    def _init_ros2(self):
        self._node = self.interface.node
        srvs = self.config.service_names
        self._services = [
            self._node.create_service(
                SetBool, srv_name, partial(self._handle_service, action)
            )
            for action, srv_name in srvs.items()
        ]

    def show_instruction(self) -> None:
        """Shows the instruction for the VR control."""
        # self.get_logger().info(
        #     f"\n{pformat(self.config.instruction_button)}"
        # )

    def _handle_service(
        self, action: Action, request: SetBool.Request, response: SetBool.Response
    ):
        if request.data:
            self.get_logger().info(f"Received {action} request.")
            self.fsm.act(action)
            response.success = True
            response.message = "1"
        else:
            response.success = False
            response.message = "2"
        return response

    def _vr_control_callback(self, data: float):
        if data[VRControllerEvent.B] > 0.1:
            self.fsm.act(Action.finish)

    def update(self) -> bool:
        # TODO: choose to spine once
        return True

    def on_shutdown(self) -> bool:
        self.get_logger().info("Shutting down JoyCallbackManager.")
        return self._node.destroy_node()


def main(args=None):
    manager = VRManager()
    assert manager.configure()

    try:
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        manager.on_shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
