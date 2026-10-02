from pydantic import BaseModel, NonNegativeFloat, NonNegativeInt, ConfigDict
from typing import Any, Optional
from airdc.common.systems.basis import System, SystemMode
from more_itertools import consume, seekable
from mcap_data_loader.datasets.dataset import IterableDatasetABC


class DataPlayerConfig(BaseModel, frozen=True):
    """Configuration for the data player."""

    model_config = ConfigDict(extra="forbid")

    source: Any
    loop: bool = False
    rate: NonNegativeFloat = 1.0
    start_time: NonNegativeInt = 0
    end_time: NonNegativeInt = 0
    cache: bool = False


class IterablePlayer(System):
    """A system that plays back data from a stream."""

    config: DataPlayerConfig

    def on_configure(self) -> bool:
        interface: IterableDatasetABC = self.interface
        if self.config.cache:
            self._stream = seekable(interface)
        else:
            self._stream = iter(interface)
        return True

    def _create_interface(self, class_type: IterableDatasetABC):
        return class_type(self.config.source)

    def send_action(self, action: int):
        """Set the stream position to the action index."""
        if self.config.cache:
            self._stream.seek(action)
        else:
            self._stream = iter(self.interface)
            consume(self._stream, action)

    def on_switch_mode(self, mode: SystemMode):
        if mode == SystemMode.RESETTING:
            self.send_action(0)
        return True

    def capture_observation(self, timeout: Optional[float] = None) -> Any:
        return next(self._stream, None)

    def get_info(self) -> dict:
        return {}

    def shutdown(self) -> bool:
        return True
