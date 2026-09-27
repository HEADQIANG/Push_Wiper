from airbot_ie.teleoprators.vr_quest import (
    VRQuestController,
    TeleopConfig,
    EventConfig,
)
from airdc.common.devices.vr.pico import (
    VRPico,
    VRQuestConfig,
    VREvent,
)


class VRPicoController(VRQuestController):
    """
    Controller for the Pico VR headset, inheriting from VRQuestController.
    This class is designed to handle the specific events and configurations
    for the Pico VR headset, while reusing the functionality of the VRQuestController.
    """

    def _init_vr(self):
        self._vr = VRPico(
            VRQuestConfig(
                zero_info={comp: self._event_config.zero_info for comp in self._pos}
            )
        )


if __name__ == "__main__":
    from airbot_ie.teleoprators.vr_quest import main

    main(
        VRPicoController(
            TeleopConfig(
                event_config=EventConfig(
                    zero_info=VREvent.RIGHT_GRIP,
                    success=VREvent.LEFT_SECONDARY_BUTTON,
                    failure=VREvent.LEFT_PRIMARY_BUTTON,
                    rerecord_episode=VREvent.LEFT_GRIP,
                    intervention=VREvent.RIGHT_GRIP,
                    shutdown=VREvent.RIGHT_SECONDARY_BUTTON,
                    left_eef=VREvent.LEFT_STICK_V,
                    right_eef=VREvent.RIGHT_STICK_V,
                ),
            )
        )
    )
