import numpy as np
from tkinter import Canvas, Tk, Toplevel
from typing import Union
from PIL import Image, ImageTk
from airdc.common.visualizers.basis import (
    GUIVisualizerConfig,
    SampleInfo,
    VisualizerBasis,
)
from airdc.basis import DictDataStamped


def get_dpi() -> float:
    root = Tk()
    dpi = root.winfo_fpixels("1i")  # 水平方向的DPI
    root.destroy()
    return dpi


def resolution_to_inches(width: int, height: int) -> tuple[float, float]:
    dpi = get_dpi()
    return width / dpi, height / dpi


class TkinterVisualizer(VisualizerBasis):
    config: GUIVisualizerConfig

    def on_configure(self):
        # 创建主窗口，但仅用于显示信息文本
        self.root = Tk()
        self.root.title(self.config.title or self.__class__.__name__)
        # 设置主窗口的Canvas高度为40，仅显示信息
        self.canvas = Canvas(self.root, width=self.config.width, height=40, bg="gray")
        self.canvas.pack()
        self.text_ids = {}
        self.windows: dict[str, dict[str, Union[Canvas, Toplevel]]] = {}
        return True

    def on_update(self, data: DictDataStamped[np.ndarray], info: SampleInfo):
        for title, data_dict in data.items():
            img = data_dict["data"]
            image_pil = Image.fromarray(img.astype(np.uint8), mode="RGB")
            img_width, img_height = image_pil.size
            if title not in self.windows:
                window = Toplevel(self.root)
                window.title(title)
                canvas = Canvas(window, width=img_width, height=img_height)
                canvas.pack()
                tk_image = ImageTk.PhotoImage(image_pil)
                image_id = canvas.create_image(0, 0, anchor="nw", image=tk_image)
                self.windows[title] = {
                    "window": window,
                    "canvas": canvas,
                    "image_id": image_id,
                    "tk_image": tk_image,
                    "width": img_width,
                    "height": img_height,
                }
            else:
                win_info = self.windows[title]
                if img_width != win_info["width"] or img_height != win_info["height"]:
                    win_info["canvas"].config(width=img_width, height=img_height)
                    win_info["window"].geometry(f"{img_width}x{img_height}")
                    win_info["width"] = img_width
                    win_info["height"] = img_height
                new_tk_image = ImageTk.PhotoImage(image_pil)
                win_info["canvas"].itemconfig(win_info["image_id"], image=new_tk_image)
                win_info["tk_image"] = new_tk_image

        if not self.config.ignore_info:
            info_text = f"Episode: {info.episode} | Index: {info.index}"
            if "info" in self.text_ids:
                self.canvas.itemconfig(self.text_ids["info"], text=info_text)
            else:
                self.text_ids["info"] = self.canvas.create_text(
                    10,
                    10,
                    text=info_text,
                    fill="black",
                    font=("Helvetica", 12),
                    anchor="nw",
                )
        self.root.update_idletasks()
        self.root.update()

    def shutdown(self) -> bool:
        for win_info in self.windows.values():
            win_info["window"].destroy()
        self.root.destroy()
        return True


if __name__ == "__main__":
    import time

    def create_color_image(
        color: tuple[int, int, int], shape=(480, 640, 3)
    ) -> np.ndarray:
        return np.ones(shape, dtype=np.uint8) * np.array(color, dtype=np.uint8)

    vis = TkinterVisualizer(GUIVisualizerConfig())
    assert vis.configure()

    images = {
        "Red": create_color_image((255, 0, 0)),
        "Green": create_color_image((0, 255, 0)),
        "Blue": create_color_image((0, 0, 255)),
    }

    round_num = 1
    index = 0

    while True:
        start = time.monotonic()
        vis.update(images, SampleInfo(episode=round_num, index=index))
        print(f"time cost: {time.monotonic() - start:.3f}s")
        time.sleep(1)
        index += 1
