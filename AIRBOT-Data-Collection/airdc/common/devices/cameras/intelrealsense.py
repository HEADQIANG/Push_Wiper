import math
import numpy as np
from threading import Thread
from time import time_ns, sleep, perf_counter
from typing import Union, Optional, Tuple, Dict
from contextlib import suppress
from airdc.common.devices.cameras.utils import (
    RGBDCameraConfig,
    CameraStreamConfig,
    CameraInfo,
    find_video_capture_devices,
)
from airdc.common.systems.basis import Sensor, DictDataStamped
from airdc.basis import force_set_attr
from airdc.common.utils.error import check_support
from mcap_data_loader.utils.dict import update_if
from pyrealsense2 import config as RSConfig  # noqa: N812
from pyrealsense2 import format as RSFormat  # noqa: N812
from pyrealsense2 import pipeline as RSPipeline  # noqa: N812
from pyrealsense2 import stream as RSStream  # noqa: N812
from pyrealsense2 import align as RSAlign  # noqa: N812
from pyrealsense2 import camera_info as RSCameraInfo  # noqa: N812
from pyrealsense2 import context as RSContext  # noqa: N812


def find_camera_indices(
    raise_when_empty=True, serial_number_index: int = 1
) -> list[str]:
    """
    Find the serial numbers of the Intel RealSense cameras
    connected to the computer.
    """
    camera_ids = []
    for device in RSContext().query_devices():
        serial_number = device.get_info(RSCameraInfo(serial_number_index))
        camera_ids.append(serial_number)

    if raise_when_empty and len(camera_ids) == 0:
        raise OSError(
            "No camera was detected. Try re-plugging, or re-installing `librealsense` and its python wrapper `pyrealsense2`, or updating the firmware."
        )

    return camera_ids


def find_camera_device_ids(
    bus_to_serial: bool = False,
) -> dict[str, list[Union[str, int]]]:
    """Find the video capture devices corresponding to Intel RealSense cameras."""
    ctx = RSContext()
    devices = find_video_capture_devices()

    mappings = {}

    for dev in ctx.devices:
        serial = dev.get_info(RSCameraInfo.serial_number)
        physical_port: str = dev.get_info(RSCameraInfo.physical_port)
        device_path = physical_port.rsplit("/", 1)[-1]  # Get the last part of the path
        for bus_info, video_devices in devices.items():
            if f"/dev/{device_path}" in video_devices:
                if bus_to_serial:
                    mappings[bus_info] = serial
                else:
                    mappings[serial] = video_devices
    return mappings


ALL_FORMATS = {
    "any",
    "bgr8",
    "bgra8",
    "combined_motion",
    "disparity16",
    "disparity32",
    "distance",
    "fg",
    "gpio_raw",
    "invi",
    "inzi",
    "m420",
    "mjpeg",
    "motion_raw",
    "motion_xyz32f",
    "name",
    "raw10",
    "raw16",
    "raw8",
    "rgb8",
    "rgba8",
    "six_dof",
    "uyvy",
    "value",
    "w10",
    "xyz32f",
    "y10bpack",
    "y12i",
    "y16",
    "y16i",
    "y411",
    "y8",
    "y8i",
    "yuyv",
    "z16",
    "z16h",
}


class IntelRealSenseCameraConfig(RGBDCameraConfig):
    """Configuration for Intel RealSense camera device."""

    force_hardware_reset: bool = True
    """Whether to force a hardware reset on camera initialization."""

    @force_set_attr
    def model_post_init(self, context):
        at_least_one_is_not_none = (
            self.fps is not None or self.width is not None or self.height is not None
        )
        at_least_one_is_none = (
            self.fps is None or self.width is None or self.height is None
        )
        if at_least_one_is_not_none and at_least_one_is_none:
            raise ValueError(
                "For `fps`, `width` and `height`, either all of them need to be set, or none of them, "
                f"but {self.fps=}, {self.width=}, {self.height=} were provided."
            )
        self.camera_index = (
            str(self.camera_index) if self.camera_index is not None else None
        )
        object.__setattr__(
            self.rgb_camera, "pixel_format", self.rgb_camera.pixel_format or "rgb8"
        )
        object.__setattr__(
            self.depth_module, "pixel_format", self.depth_module.pixel_format or "z16"
        )

        def check_pixel_format(pixel_format: str, module: str):
            check_support("pixel format", pixel_format, module, ALL_FORMATS)

        check_pixel_format(self.rgb_camera.pixel_format, "RGB camera")
        check_pixel_format(self.depth_module.pixel_format, "Depth module")


class IntelRealSenseCamera(Sensor):
    """Interface for Intel RealSense cameras."""

    def __init__(self, config: IntelRealSenseCameraConfig):
        self.config = config
        self._rs_pipe = None
        self.logs = {}
        self._info = {}
        self._cur_obs = None
        self._is_running = True
        self._cap_thread = None

    def on_configure(self):
        self._connect()
        if not self.config.blocking:
            self._cap_thread = Thread(target=self._capture_loop, daemon=True)
            self._cap_thread.start()
            self._capture_call = self._capture_non_blocking
            self.get_logger().info("Waiting for the first observation...")
            while self._cur_obs is None:
                sleep(0.1)
        else:
            self._capture_call = self._capture_blocking
        return True

    @staticmethod
    def intrinsics_to_camera_info(intrinsics) -> CameraInfo:
        ori_model = str(intrinsics.model).split(".")[-1]
        distortion_model = {
            "brown_conrady": "plumb_bob",
            "kannala_brandt4": "equidistant",
        }.get(ori_model, ori_model)

        k = [0.0] * 9
        k[0] = intrinsics.fx  # fx
        k[2] = intrinsics.ppx  # cx
        k[4] = intrinsics.fy  # fy
        k[5] = intrinsics.ppy  # cy
        k[8] = 1.0

        p = [0.0] * 12
        p[0] = intrinsics.fx  # fx
        p[2] = intrinsics.ppx  # cx
        p[5] = intrinsics.fy  # fy
        p[6] = intrinsics.ppy  # cy
        p[10] = 1.0

        return CameraInfo(
            width=intrinsics.width,
            height=intrinsics.height,
            distortion_model=distortion_model,
            d=intrinsics.coeffs,
            k=k,
            p=p,
        )

    @staticmethod
    def get_camera_info(device) -> Dict[str, str]:
        info_dict = {}
        for info_type in dir(RSCameraInfo):
            if not info_type.startswith("__"):
                with suppress(Exception):
                    info_value = getattr(RSCameraInfo, info_type)
                    if device.supports(info_value):
                        info_str = device.get_info(info_value)
                        info_dict[info_type] = info_str
        return info_dict

    def _process_stream_profiles(self, profile, stream_name):
        stream = profile.get_stream(stream_name)
        stream_profile = stream.as_video_stream_profile()
        stream_type = str(stream_name).split(".")[-1]
        config: CameraStreamConfig = getattr(self.config, stream_type)
        camera_info = self.intrinsics_to_camera_info(
            stream_profile.get_intrinsics()
        ).model_dump(mode="json")
        for sub_info in (config.intrinsics, config.calibration):
            if sub_info is not None:
                update_if(
                    camera_info,
                    sub_info.model_dump(mode="json", exclude_unset=True),
                )
        actual_fps = stream_profile.fps()
        actual_width = camera_info["width"]
        actual_height = camera_info["height"]
        # Using `math.isclose` since actual fps can be a float (e.g. 29.9 instead of 30)
        if config.fps is not None and not math.isclose(
            config.fps, actual_fps, rel_tol=1e-3
        ):
            # Using `OSError` since it's a broad that encompasses issues related to device communication
            raise OSError(
                f"Can't set {config.fps=} for IntelRealSenseCamera({config.camera_index}). Actual value is {actual_fps}."
            )
        if config.width is not None and config.width != actual_width:
            raise OSError(
                f"Can't set {config.width=} for IntelRealSenseCamera({config.camera_index}). Actual value is {actual_width}."
            )
        if config.height is not None and config.height != actual_height:
            raise OSError(
                f"Can't set {config.height=} for IntelRealSenseCamera({config.camera_index}). Actual value is {actual_height}."
            )
        self._info[stream_type] = {
            "camera_info": camera_info,
            "pixel_format": config.pixel_format,
            "fps": actual_fps,
        }

    def _enable_stream(self, rs_config, stream_name, rs_format):
        config = self.config
        use_full_config = config.fps and config.width and config.height
        if use_full_config:
            # TODO(rcadene): can we set rgb8 directly?
            rs_config.enable_stream(
                stream_name, config.width, config.height, rs_format, config.fps
            )
        else:
            rs_config.enable_stream(stream_name)

    def _connect(self):
        rs_config = RSConfig()
        config = self.config
        if config.camera_index:
            rs_config.enable_device(config.camera_index)

        if config.enable_color:
            self._enable_stream(
                rs_config,
                RSStream.color,
                getattr(RSFormat, config.rgb_camera.pixel_format),
            )

        if config.enable_depth:
            self._enable_stream(
                rs_config,
                RSStream.depth,
                getattr(RSFormat, config.depth_module.pixel_format),
            )

        self._rs_pipe = RSPipeline()
        try:
            profile = self._rs_pipe.start(rs_config)
        except RuntimeError as e:
            available_cam_ids = find_camera_indices()
            # If the camera doesn't work, display the camera indices corresponding to
            # valid cameras.
            # Verify that the provided `camera_index` is valid before printing the traceback
            if config.camera_index not in available_cam_ids:
                raise ValueError(
                    f"`camera_index` is expected to be one of these available cameras {available_cam_ids}, but {config.camera_index} is provided instead. "
                    "To find the camera index you should use, run `python lerobot/common/devices/cameras/intelrealsense.py`."
                ) from e
            raise OSError(
                f"Can't access IntelRealSenseCamera({config.camera_index})."
            ) from e

        if config.align_depth:
            self.align = RSAlign(RSStream.color)
        if config.enable_color:
            self._process_stream_profiles(profile, RSStream.color)
        if config.enable_depth:
            self._process_stream_profiles(profile, RSStream.depth)

        device = profile.get_device()
        self._info.update(self.get_camera_info(device))

    def _get_frame(self, frames, frame_type: str) -> Tuple[np.ndarray, int]:
        frame = getattr(frames, f"get_{frame_type}_frame")()
        stamp = time_ns()
        if not frame:
            raise OSError(
                f"Can't capture {frame_type} image from {self.config.camera_index}."
            )
        return np.asanyarray(frame.get_data()).copy(), stamp

    def capture_observation(
        self, timeout: Optional[float] = None
    ) -> DictDataStamped[np.ndarray]:
        """Capture an observation from the camera.
        Returns:
            A dictionary containing the captured images.
        Raises:
            OSError: If the image cannot be captured.
        """
        return self._capture_call(timeout)

    def _capture_blocking(
        self, timeout: Optional[float] = None
    ) -> DictDataStamped[np.ndarray]:
        config = self.config
        frames = self._rs_pipe.wait_for_frames(
            timeout_ms=int(timeout * 1000) if timeout is not None else None
        )
        if config.align_depth:
            frames = self.align.process(frames)
        outputs = {}
        if config.enable_color:
            color_image, stamp = self._get_frame(frames, "color")
            outputs["color/image_raw"] = {"t": stamp, "data": color_image}
        if config.enable_depth:
            depth_map, stamp = self._get_frame(frames, "depth")
            key = (
                "aligned_depth_to_color/image_raw"
                if config.align_depth
                else "depth/image_rect_raw"
            )
            outputs[key] = {"t": stamp, "data": depth_map}
        return outputs

    def _capture_non_blocking(self, timeout=None) -> DictDataStamped[np.ndarray]:
        return self._cur_obs

    def _capture_loop(self):
        rate = max(self.config.rgb_camera.fps, self.config.depth_module.fps)
        period = 1.0 / rate
        while self._is_running:
            start = perf_counter()
            self._cur_obs = self._capture_blocking(3.0)
            elapsed = perf_counter() - start
            sleep_duration = max(0.0, period - elapsed)
            sleep(sleep_duration)
        self.get_logger().info("Capture thread stopped.")

    def get_info(self):
        """Get the camera information."""
        return self._info

    def shutdown(self) -> bool:
        self._is_running = False
        if self._cap_thread is not None:
            self._cap_thread.join(timeout=3.0)
        self._rs_pipe.stop()
        return True


if __name__ == "__main__":
    print(find_camera_indices())
    print(find_camera_device_ids())
