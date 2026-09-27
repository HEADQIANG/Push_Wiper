from mcap_data_loader.datasets.mcap_dataset import (
    McapFlatBuffersSampleDataset,
    McapDatasetConfig,
)
from airdc.common.systems.data_player import (
    DataPlayerConfig,
    IterablePlayer,
)
from typing import Dict, Union, Any, Optional
import numpy as np


class McapPlayerConfig(DataPlayerConfig):
    source: McapDatasetConfig


class McapPlayer(IterablePlayer):
    """A system that plays back data from an MCAP dataset."""

    config: McapPlayerConfig
    interface: McapFlatBuffersSampleDataset

    def capture_observation(
        self, timeout: Optional[float] = None
    ) -> Dict[str, Union[np.ndarray, Any]]:
        return super().capture_observation(timeout)
