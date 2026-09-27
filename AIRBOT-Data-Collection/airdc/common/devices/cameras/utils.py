import platform
from enum import Enum
from typing import List, Dict, Tuple, Optional, Union
from typing_extensions import Annotated, Self
from pydantic import (
    BaseModel,
    ConfigDict,
    NonNegativeInt,
    PositiveInt,
    ValidationInfo,
    AfterValidator,
    field_validator,
    model_validator,
    Field,
)
from collections import defaultdict
from airdc.basis import ConcurrentMode


class Intrinsics(BaseModel, frozen=True):
    """Intrinsic parameters of a camera."""

    distortion_model: str = ""
    """The distortion model of the camera."""
    d: List[float] = []
    """The distortion coefficients."""
    k: List[float] = Field([0.0] * 9, min_length=9, max_length=9)
    """The intrinsic camera matrix (3x3) stored in a row-major order."""
    binning_x: NonNegativeInt = 0
    """The binning factor in the x direction."""
    binning_y: NonNegativeInt = 0
    """The binning factor in the y direction."""


class RegionOfInterest(BaseModel, frozen=True):
    """Region of interest in an image."""

    x_offset: NonNegativeInt = 0
    """The horizontal offset of the region of interest."""
    y_offset: NonNegativeInt = 0
    """The vertical offset of the region of interest."""
    height: NonNegativeInt = 0
    """The height of the region of interest."""
    width: NonNegativeInt = 0
    """The width of the region of interest."""
    do_rectify: bool = False
    """Whether to rectify the region of interest."""


class Calibration(BaseModel, frozen=True):
    """Calibration parameters of a camera."""

    r: List[float] = Field([0.0] * 9, min_length=9, max_length=9)
    """The rectification matrix (3x3) stored in a row-major order."""
    p: List[float] = Field([0.0] * 12, min_length=12, max_length=12)
    """The projection matrix (3x4) stored in a row-major order."""
    roi: RegionOfInterest = RegionOfInterest()
    """The region of interest."""


class CameraInfo(Intrinsics, Calibration):
    """Camera information."""

    width: NonNegativeInt
    """The width of the camera image."""
    height: NonNegativeInt
    """The height of the camera image."""

    @staticmethod
    def is_meaningful(value: list) -> bool:
        """Check if the intrinsic or calibration matrix has meaningful values.
        The inspection was incomplete."""
        return bool(set(value) - {0.0})


class StreamProfile(BaseModel, frozen=True):
    """Stream profile of a camera stream."""

    model_config = ConfigDict(extra="forbid")

    fps: Optional[PositiveInt] = None
    """The frame rate of the stream."""
    width: Optional[PositiveInt] = None
    """The width of the stream."""
    height: Optional[PositiveInt] = None
    """The height of the stream."""


class CameraDeviceConfig(StreamProfile):
    """Base configuration for a camera device.
    If StreamProfile is set, it will be used for the corresponding item of a stream that is not set.
    """

    model_config = ConfigDict(validate_by_alias=True, validate_by_name=True)

    camera_index: Optional[Union[int, str]] = None
    """The index of the camera to use. If None, the default camera will be used."""
    concurrent: ConcurrentMode = ConcurrentMode.none
    """The concurrency mode of the camera."""
    blocking: bool = True
    """Whether to block until a frame is available."""


class ColorDeviceConfig(CameraDeviceConfig, frozen=True):
    """Base configuration for an RGB camera device."""

    enable_color: bool = True
    """Whether to enable color sensing."""


class DepthDeviceConfig(CameraDeviceConfig, frozen=True):
    """Base configuration for a depth camera device."""

    enable_depth: bool = False
    """Whether to enable depth sensing."""


class RGBDDeviceConfig(ColorDeviceConfig, DepthDeviceConfig):
    """Base configuration for an RGB-D camera device."""

    align_depth: bool = False
    """Whether to align depth to color."""

    @field_validator("align_depth", mode="after")
    def check_align_depth(cls, align_depth, info: ValidationInfo):
        if not info.data.get("enable_depth", False):
            return False
        return align_depth

    @model_validator(mode="after")
    def validate_enable_streams(self) -> Self:
        if not (self.enable_color or self.enable_depth):
            raise ValueError(
                "At least one of `enable_color` or `enable_depth` must be True."
            )
        return self


class CameraStreamConfig(StreamProfile):
    """Base configuration for a camera stream."""

    model_config = ConfigDict(extra="forbid")

    pixel_format: Optional[Union[str, Enum]] = None
    """The pixel format of the camera image."""
    intrinsics: Optional[Intrinsics] = None
    """The intrinsic parameters of the camera."""
    calibration: Optional[Calibration] = None
    """The calibration parameters of the camera."""


class ColorStreamConfig(CameraStreamConfig):
    """Configuration for an color camera device."""


class DepthStreamConfig(CameraStreamConfig):
    """Configuration for a depth camera device."""


def replace_none(obj, ref, keys: List[str]):
    for key in keys:
        if getattr(obj, key) is None:
            value = ref.get(key) if isinstance(ref, dict) else getattr(ref, key)
            object.__setattr__(obj, key, value)


def validate_stream_config(
    value: CameraStreamConfig, info: ValidationInfo
) -> CameraStreamConfig:
    replace_none(value, info.data, ("fps", "width", "height"))
    return value


class ColorCameraConfig(ColorDeviceConfig):
    """Configuration for an color camera device."""

    rgb_camera: Annotated[ColorStreamConfig, AfterValidator(validate_stream_config)] = (
        Field(default_factory=ColorStreamConfig, validate_default=True)
    )
    """The stream configuration of the color camera."""

    @property
    def color(self):
        return self.rgb_camera


class DepthCameraConfig(DepthDeviceConfig):
    """Configuration for a depth camera device."""

    depth_module: Annotated[
        DepthStreamConfig, AfterValidator(validate_stream_config)
    ] = Field(default_factory=DepthStreamConfig, validate_default=True)
    """The stream configuration of the depth camera."""

    @property
    def depth(self):
        return self.depth_module


class RGBDCameraConfig(ColorCameraConfig, DepthCameraConfig, RGBDDeviceConfig):
    """Configuration for an RGB-D camera device."""


class CameraControl(BaseModel, frozen=True):
    """Camera control settings."""

    model_config = ConfigDict(extra="forbid")

    brightness: NonNegativeInt
    contrast: NonNegativeInt
    saturation: NonNegativeInt
    hue: NonNegativeInt
    white_balance_automatic: bool
    gamma: PositiveInt
    power_line_frequency: int = 1
    white_balance_temperature: PositiveInt
    sharpness: NonNegativeInt
    backlight_compensation: NonNegativeInt
    auto_exposure: int = 3
    exposure_time_absolute: NonNegativeInt
    exposure_dynamic_framerate: bool


def find_video_capture_devices(
    card_key: bool = False, cards: Optional[List[str]] = None
) -> Dict[Tuple[str, str], List[str]]:
    """
    Finds all video capture devices on the system and returns a dictionary
    with device information.
    """
    from linuxpy.video.device import iter_video_capture_devices

    cards = cards or []
    if isinstance(cards, str):
        cards = [cards]
    slices = slice(None, 2) if card_key else 1
    devices = defaultdict(list)
    for dev in iter_video_capture_devices():
        with dev:
            key = (dev.info.card, dev.info.bus_info)
            use = True
            if cards:
                for card in cards:
                    if card in key[0]:
                        break
                else:
                    use = False
            if use:
                devices[key[slices]].append(str(dev.filename))
    return dict(devices)


def find_camera_indices(
    raise_when_empty: bool = False,
    max_index_search_range: int = 10,
    filter_mode: str = "none",
    sorting: bool = True,
) -> list[int]:
    """Finds the available camera (video capture devices) indices on the system.
    # The maximum opencv device index depends on your operating system. For instance,
    # if you have 3 cameras, they should be associated to index 0, 1, and 2. This is the case
    # on MacOS. However, on Ubuntu, the indices are different like 6, 16, 23.
    # When you change the USB port or reboot the computer, the operating system might
    # treat the same cameras as new devices. Thus we select a higher bound to search indices.
    """
    if platform.system() == "Linux":
        possible_camera_ids = [
            int(device[0].removeprefix("/dev/video"))
            for device in find_video_capture_devices().values()
        ]
    else:
        print(
            "Mac or Windows detected. Finding available camera indices through "
            f"scanning all indices from 0 to {max_index_search_range}"
        )
        possible_camera_ids = range(max_index_search_range)

    camera_ids = possible_camera_ids

    if filter_mode in {"even", "odd"}:
        remainder = 4 - len(filter_mode)
        camera_ids = [
            camera_id for camera_id in camera_ids if camera_id % 2 == remainder
        ]
    if sorting:
        camera_ids = sorted(camera_ids)

    if raise_when_empty and len(camera_ids) == 0:
        raise OSError(
            "Not a single camera was detected. Try re-plugging, or re-installing `opencv2`, "
            "or your camera driver, or make sure your camera is compatible with opencv2."
        )

    return camera_ids


def get_video_device_bus_info():
    """Returns a dictionary mapping video device paths to their bus info."""
    from linuxpy.video.device import iter_video_capture_devices

    device_bus_info = {}
    for dev in iter_video_capture_devices():
        with dev:
            device_bus_info[str(dev.filename)] = dev.info.bus_info
    return device_bus_info


def get_camera_index_by_bus_info(
    bus_info: str, sorting: bool = False, allow_empty: bool = False
) -> List[str]:
    """
    Get the camera index by its bus info.
    Args:
        bus_info: The bus info of the camera.
        sorting: Whether to sort the camera indices.
    Return: The camera indices that match the bus info.
    """
    available = find_video_capture_devices(False)
    devices = available.get(bus_info, [])
    if not (allow_empty or devices):
        raise ValueError(
            f"No camera indexes found with bus info: {bus_info}. Available: {available}"
        )
    if sorting:
        devices = sorted(devices)
    return devices


if __name__ == "__main__":
    from pprint import pprint

    pprint(RGBDCameraConfig(fps=30, width=640, height=480).model_dump())
    # print("RealSense cameras:", find_video_capture_devices(False, "RealSense"))
    # print("LRCP cameras:", find_video_capture_devices(False, "LRCP"))
    # print("Webcam cameras:", find_video_capture_devices(False, "Webcam"))
    # print("cam:", find_video_capture_devices(False, "cam"))
    # print("All cameras:", find_video_capture_devices(False, ""))
    # print(get_video_device_bus_info())
