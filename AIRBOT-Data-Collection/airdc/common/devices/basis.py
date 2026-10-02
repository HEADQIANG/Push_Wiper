from airdc.utils import StrEnum
from airdc.basis import ConfigurableBasis, ConfigType
from enum import auto, Enum
from collections import defaultdict
from typing import Callable, Dict, List


class EventValueMode(StrEnum):
    NOT_ZERO = auto()
    VALUE_CHANGE = auto()
    LEAVE_ZERO = auto()
    ENTER_ZERO = auto()
    ENTER_POSITIVE = auto()
    ENTER_NEGATIVE = auto()


class EventableDeviceBasis(ConfigurableBasis):
    """Base class for devices that can generate events based on state changes."""

    def __init__(self, config: ConfigType = None):
        self._event_callbacks: Dict[EventValueMode, Dict[Enum, List[Callable]]] = (
            defaultdict(lambda: defaultdict(list))
        )
        self._callbacks = []
        self._init_judgers()

    def register_event_callback(self, event: Enum, callback: Callable, mode: Enum):
        self._event_callbacks[mode][event].append(callback)

    def register_callback(self, callback: Callable):
        """Register a callback for the VR controller events."""
        self._callbacks.append(callback)

    def _init_judgers(self):
        """Initialize the judgers for the VR controller events."""
        self._judgers = {
            EventValueMode.NOT_ZERO: lambda data, event: data != 0,
            EventValueMode.VALUE_CHANGE: lambda data, event: data
            != self._vr_control_data[event],
            EventValueMode.LEAVE_ZERO: lambda data, event: data != 0
            and self._vr_control_data[event] == 0,
            EventValueMode.ENTER_ZERO: lambda data, event: data == 0
            and self._vr_control_data[event] != 0,
            EventValueMode.ENTER_POSITIVE: lambda data, event: data > 0
            and self._vr_control_data[event] <= 0,
            EventValueMode.ENTER_NEGATIVE: lambda data, event: data < 0
            and self._vr_control_data[event] >= 0,
        }
