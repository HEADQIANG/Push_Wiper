from typing import Dict, Union
from airdc.common.utils.tk_keyboard import ButtonUILayout, TkButtonPanelConfig
from airdc.common.utils.tk_keyboard import Listener as TkKeyboardListener
from airdc.managers.basis import (
    DemonstrateManagerBasis,
    ManagerConfigBasis,
    DemonstrateAction as DAction,
    ManagerAction as MAction,
)


class TkinterManagerConfig(ManagerConfigBasis):
    """Tkinter button-based manager config."""

    key_to_action: Dict[Union[DAction, MAction], str] = {
        action.capitalize(): action
        for action in (
            DAction.sample,
            DAction.save,
            DAction.abandon,
            DAction.remove,
            DAction.capture,
            MAction.MODE,
            MAction.FOLLOW,
            MAction.LOCK,
            MAction.INSTRUCTION,
            DAction.finish,
        )
    }
    layout: ButtonUILayout = ButtonUILayout()
    """Layout settings for the tkinter button panel."""


class TkinterManager(DemonstrateManagerBasis):
    """Handles Tkinter button panel for controlling data collection."""

    config: TkinterManagerConfig

    def on_configure(self):
        self.listener = TkKeyboardListener(
            TkButtonPanelConfig(
                layout=self.config.layout,
                buttons=self.config.key_to_action.keys(),
                on_press=self._act_key,
                on_close=self._on_close,
                button_callbacks={
                    MAction.INSTRUCTION.capitalize(): self.config.instruction_str
                },
            )
        )
        self.listener.start()
        return True

    def update(self) -> bool:
        self.listener.update()
        return True

    def _on_close(self):
        return self._act(DAction.finish)

    def on_shutdown(self) -> bool:
        self.listener.stop()
        return True
