import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes
from matplotlib.image import AxesImage
from airdc.common.visualizers.basis import (
    GUIVisualizerConfig,
    SampleInfo,
    VisualizerBasis,
)
from airdc.utils import optimal_grid
from airdc.common.visualizers.tk import get_dpi, resolution_to_inches
from airdc.basis import DictDataStamped


class PltVisualizer(VisualizerBasis):
    config: GUIVisualizerConfig

    def on_configure(self) -> bool:
        plt.ion()
        self._displays: dict[str, AxesImage] = {}
        return True

    def on_update(self, data: DictDataStamped[np.ndarray], info: SampleInfo) -> bool:
        if not self._displays:
            img_num = len(data)
            if self.config.max_num > 0:
                row, col = self.config.max_num, np.ceil(img_num / self.config.max_num)
            else:
                # TODO: should use all image ratios？
                img_shape = list(data.values())[0]["data"].shape
                img_ratio = img_shape[1] / img_shape[0]
                row, col = optimal_grid(
                    img_num,
                    self.config.screen_width,
                    self.config.screen_height,
                    img_ratio,
                )
            if self.config.axis == 0:
                row, col = col, row
            self._fig, axes = plt.subplots(
                row,
                col,
                figsize=resolution_to_inches(self.config.width, self.config.height),
                dpi=get_dpi(),
            )
            if isinstance(axes, Axes):
                axes = [axes]
            else:
                axes = axes.flatten()
            for i, (key, image) in enumerate(data.items()):
                axis = axes[i]
                axis.set_title(key)
                self._displays[key] = axis.imshow(image)
            self._axes = axes
            plt.tight_layout()
        for key, image in data.items():
            self._displays[key].set_data(image)
        self._fig.canvas.manager.set_window_title(
            f"Episode: {info.episode}, Index: {info.index}"
        )
        plt.pause(0.001)
        return True

    def shutdown(self) -> bool:
        plt.close()
        self._displays = {}
        return True


if __name__ == "__main__":
    import time

    vis = PltVisualizer()
    assert vis.configure()
    while True:
        new_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        images = {key: new_img for key in {"image1", "image2", "image3", "image4"}}
        start = time.monotonic()
        vis.update(images, SampleInfo(episode=0, index=0))
        print(f"Update time cost: {time.monotonic() - start:.4f}s")
