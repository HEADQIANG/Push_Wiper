import argparse
import asyncio
import logging
import time
import cv2
from linuxpy.video.device import Capability, Device, PixelFormat, VideoCapture
from airdc.common.visualizers.opencv import decode_image
from airdc.utils import zip


MODES = {
    "auto": None,
    "mmap": Capability.STREAMING,
    "read": Capability.READWRITE,
}

frame_dicts = {}
frame_info_dicts = {}


async def run_one(device: Device, frame_format: PixelFormat, args):
    device.open()
    capture = VideoCapture(device)
    capture.set_format(*args.frame_size, frame_format)
    with capture:
        start_time = time.monotonic()
        async for frame in capture:
            freq = 1 / (time.monotonic() - start_time)
            start_time = time.monotonic()
            frame_dicts[device] = frame
            frame_info_dicts[device.index] = (
                f"frame_nb: {frame.frame_nb}, fps: {freq:.2f}"
            )
            cv2.imshow(
                f"{device.index}",
                decode_image(bytes(frame), frame_format.name, *args.frame_size),
            )
            cv2.waitKey(1)


async def run(args):
    _ = [
        asyncio.create_task(run_one(device, frame_format, args))
        for device, frame_format in zip(
            args.devices, args.frame_formats or [PixelFormat.MJPEG] * len(args.devices)
        )
    ]
    while True:
        print(frame_info_dicts, flush=True, end="\r")
        await asyncio.sleep(0.02)


def device_text(text):
    try:
        return Device.from_id(int(text))
    except ValueError:
        return Device(text)


def frame_size(text):
    w, h = text.split("x", 1)
    return int(w), int(h)


def frame_format(text):
    return PixelFormat[text]


def cli():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--log-level", choices=["debug", "info", "warning", "error"], default="info"
    )
    parser.add_argument("--mode", choices=MODES, default="auto")
    parser.add_argument("--nb-buffers", type=int, default=2)
    parser.add_argument("--frame-rate", type=float, default=0.0)
    parser.add_argument("--frame-size", type=frame_size, default="640x480")
    parser.add_argument("-ff", "--frame-formats", type=frame_format, nargs="+")
    parser.add_argument("devices", type=device_text, nargs="+")
    return parser


def main(args=None):
    parser = cli()
    args = parser.parse_args(args=args)
    fmt = "%(threadName)-10s %(asctime)-15s %(levelname)-5s %(name)s: %(message)s"
    logging.basicConfig(level=args.log_level.upper(), format=fmt)

    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        logging.info("Ctrl-C pressed. Bailing out")


if __name__ == "__main__":
    main()
