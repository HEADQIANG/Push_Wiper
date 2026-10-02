import numpy as np


class ImageCoder:
    @staticmethod
    def rgb2yuv(rgb: np.ndarray) -> np.ndarray:
        # The coefficients were taken from OpenCV https://github.com/opencv/opencv
        # I'm not sure if the values should be clipped, in my (limited) testing it looks alright
        #   but don't hesitate to add rgb.clip(0, 1, rgb) & yuv.clip(0, 1, yuv)
        #
        # Input for these functions is a numpy array with shape (height, width, 3)
        # Change '+= 0.5' to '+= 127.5' & '-= 0.5' to '-= 127.5' for values in range [0, 255]

        m = np.array(
            [
                [0.29900, -0.147108, 0.614777],
                [0.58700, -0.288804, -0.514799],
                [0.11400, 0.435912, -0.099978],
            ]
        )
        yuv = np.dot(rgb, m)
        yuv[:, :, 1:] += 127.5
        return yuv

    @staticmethod
    def yuv2rgb(yuv: np.ndarray) -> np.ndarray:
        # The coefficients were taken from OpenCV https://github.com/opencv/opencv
        # I'm not sure if the values should be clipped, in my (limited) testing it looks alright
        #   but don't hesitate to add rgb.clip(0, 1, rgb) & yuv.clip(0, 1, yuv)
        #
        # Input for these functions is a numpy array with shape (height, width, 3)
        # Change '+= 0.5' to '+= 127.5' & '-= 0.5' to '-= 127.5' for values in range [0, 255]

        m = np.array(
            [
                [1.000, 1.000, 1.000],
                [0.000, -0.394, 2.032],
                [1.140, -0.581, 0.000],
            ]
        )
        yuv[:, :, 1:] -= 127.5
        rgb = np.dot(yuv, m)
        return rgb

    @staticmethod
    def yuyv2bgr(data: bytes, width: int, height: int) -> np.ndarray:
        """Convert YUYV image bytes data to BGR array using numpy.
        Args:
            data: bytes of YUYV image
            height: image height
            width: image width
        Returns:
            numpy image array in bgr
        """
        yuyv = np.frombuffer(data, np.uint8).reshape((height, width, 2))
        y = yuyv[:, :, 0].astype(np.float32)
        u = np.zeros((height, width), dtype=np.float32)
        v = np.zeros((height, width), dtype=np.float32)
        u[:, 0::2] = yuyv[:, 0::2, 1]
        u[:, 1::2] = u[:, 0::2]
        v[:, 1::2] = yuyv[:, 1::2, 1]
        v[:, 0::2] = v[:, 1::2]
        r = y + 1.403 * (v - 128)
        g = y - 0.344 * (u - 128) - 0.714 * (v - 128)
        b = y + 1.770 * (u - 128)
        bgr = np.zeros((height, width, 3), dtype=np.uint8)
        bgr[:, :, 0] = np.clip(b, 0, 255).astype(np.uint8)  # B
        bgr[:, :, 1] = np.clip(g, 0, 255).astype(np.uint8)  # G
        bgr[:, :, 2] = np.clip(r, 0, 255).astype(np.uint8)  # R
        return bgr
