from airdc.common.systems.grouped import (
    GroupedComponentsSystemConfig,
    GroupedComponentsSystem,
    GroupsSendActionConfig,
)
from airdc.basis import DictDataStamped, ForceSetAttr
from pydantic import field_validator, ValidationInfo
from mcap_data_loader.datasets.dataset import RealTimeDatasetABC


class GroupedSystemDataSourceConfig(GroupedComponentsSystemConfig):
    """Configuration for Grouped Components System Data Source"""

    reset_action: GroupsSendActionConfig = GroupsSendActionConfig()
    """Action to reset the grouped components system"""

    @field_validator("reset_action")
    def validate_reset_action(cls, v: GroupsSendActionConfig, info: ValidationInfo):
        with ForceSetAttr(v):
            if v.action_values and not v.groups:
                v.groups = list(dict.fromkeys(info.data["components"].groups))
            v.model_post_init(None)
            return v


class GroupedSystemDataSource(RealTimeDatasetABC[DictDataStamped]):
    """Grouped Components System Data Source"""

    def __init__(self, config: GroupedSystemDataSourceConfig):
        self.config = config
        self.interface = GroupedComponentsSystem(config)
        if not self.interface.configure():
            raise RuntimeError("Failed to configure GroupedComponentsSystem")

    def reset(self):
        return self.interface.send_action(self.config.reset_action)

    def write(self, input: GroupsSendActionConfig):
        return self.interface.send_action(input)

    def read(self):
        return self.interface.capture_observation()

    def close(self):
        return self.interface.shutdown()


if __name__ == "__main__":
    from airdc.utils import init_logging
    from airdc.common.systems.basis import SystemMode
    from airdc.common.systems.grouped import (
        SystemSensorComponentGroupsConfig,
        AutoControlConfig,
    )
    from pprint import pprint
    from airdc.common.devices.cameras.mock import MockCamera
    from airdc.common.systems.mock import MockSystem
    import time

    init_logging()

    reset_action = GroupsSendActionConfig(
        groups=["/"], action_values=[[0.0] * 6], modes=[SystemMode.RESETTING]
    )
    system = MockSystem()
    camera = MockCamera()
    config = GroupedSystemDataSourceConfig(
        components=SystemSensorComponentGroupsConfig(
            groups=["/", "/"],
            names=["lead_arm", "left_camera"],
            roles=["l", "o"],
            instances=[system, camera],
        ),
        reset_action=reset_action,
        auto_control=AutoControlConfig(groups=[]),
    )
    pprint(config.model_dump())
    live_data = GroupedSystemDataSource(config)
    # assert live_data.configure()
    live_data.reset()
    print(live_data.read().keys())
    reset_action.modes = [SystemMode.SAMPLING] * len(reset_action.modes)
    live_data.write(reset_action)

    for i, data in enumerate(live_data):
        for key, value in data.items():
            print(f"{key}: {value['t']}")
        time.sleep(0.5)
        if i >= 5:
            break
    live_data.close()
