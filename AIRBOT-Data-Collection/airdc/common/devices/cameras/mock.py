from airdc.common.devices.cameras.utils import RGBDCameraConfig
from airdc.common.systems.basis import Sensor, DictDataStamped
from airdc.basis import force_set_attr
from typing import Optional
from time import time_ns
import numpy as np


class MockCameraConfig(RGBDCameraConfig):
    """Configuration for a mock camera device used for testing purposes."""

    random: bool = False

    @force_set_attr
    def model_post_init(self, context):
        if self.width is None:
            self.width = 640
        if self.height is None:
            self.height = 480


class MockCamera(Sensor):
    """A mock camera device used for testing purposes."""

    config: MockCameraConfig

    def on_configure(self):
        self._update_random_image()
        return True

    def capture_observation(
        self, timeout: Optional[float] = None
    ) -> DictDataStamped[np.ndarray]:
        if self.config.random:
            self._update_random_image()
        observation = {}
        if self.config.enable_color:
            observation["color/image_raw"] = {"t": time_ns(), "data": self.color_image}
        if self.config.enable_depth:
            if self.config.align_depth:
                key = "aligned_depth_to_color/image_raw"
            else:
                key = "depth/image_rect_raw"
            observation[key] = {"t": time_ns(), "data": self.depth_image}
        return observation

    def get_info(self):
        return self.config.model_dump(mode="json")

    def shutdown(self) -> bool:
        self.color_image = None
        self.depth_image = None
        return True

    def _update_random_image(self):
        if self.config.enable_color:
            self.color_image = np.random.randint(
                0, 256, (self.config.height, self.config.width, 3), dtype=np.uint8
            )
        if self.config.enable_depth:
            self.depth_image = np.random.randint(
                0,
                5000,
                (self.config.height, self.config.width),
                dtype=np.uint16,
            )
