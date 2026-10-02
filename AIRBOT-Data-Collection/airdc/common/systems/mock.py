from airdc.common.systems.basis import System, SystemConfig
from time import time_ns


class MockSystem(System):
    """A mock system used for testing purposes."""

    config: SystemConfig

    def on_configure(self) -> bool:
        return True

    def capture_observation(self, timeout=None):
        return {"mock_data": {"t": time_ns(), "data": "This is a mock observation."}}

    def on_switch_mode(self, mode):
        self.get_logger().info(f"Switching to mode: {mode}")
        return True

    def send_action(self, action):
        self.get_logger().info(f"Sending action: {action}")

    def shutdown(self) -> bool:
        return True

    def get_info(self):
        return {}
