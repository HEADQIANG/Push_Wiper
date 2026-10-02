"""Check collection dependencies and a synthetic MCAP round trip, without hardware."""

import importlib
from collections import Counter, defaultdict
from io import BytesIO
from pathlib import Path
from tempfile import TemporaryDirectory
from time import time_ns

import av
import numpy as np
from mcap.reader import make_reader
from mcap_data_loader.serialization.flb import McapFlatBuffersReader
from turbojpeg import TurboJPEG

from airdc.common.samplers.mcap_sampler import McapDataSampler


def main():
    for module in (
        "airdc.main",
        "airbot_ie.robots.airbot_play",
        "airdc.common.devices.cameras.intelrealsense",
        "airdc.common.devices.cameras.v4l2",
    ):
        importlib.import_module(module)
        print(f"OK: {module}")

    rgb = np.full((64, 64, 3), 96, dtype=np.uint8)
    depth = np.full((64, 64), 1234, dtype=np.uint16)
    jpeg = TurboJPEG()
    assert jpeg.decode(jpeg.encode(rgb)).shape == rgb.shape
    print("OK: TurboJPEG encode/decode")

    camera_keys = ("/realsense/color/image_raw", "/rgb_camera/color/image_raw")
    depth_key = "/realsense/aligned_depth_to_color/image_raw"
    numeric = {}
    for side in ("left", "right"):
        numeric[f"/{side}/follow/arm/joint_state/position"] = [0.1] * 6
        numeric[f"/{side}/follow/eef/joint_state/position"] = [0.02]
    frames = 5
    start = time_ns()

    with TemporaryDirectory(prefix="airdc-env-check-") as temp_dir:
        sampler = McapDataSampler()
        sampler.set_info({"environment_check": {"synthetic": True}})
        assert sampler.configure()
        path = sampler.compose_path(Path(temp_dir), 0)
        remaining = defaultdict(list)
        for index in range(frames):
            stamp = start + index * 50_000_000
            payload = {key: {"t": stamp, "data": value} for key, value in numeric.items()}
            payload.update({key: {"t": stamp, "data": rgb} for key in camera_keys})
            payload[depth_key] = {"t": stamp, "data": depth}
            payload["log_stamps"] = stamp
            for key, value in sampler.update(payload).items():
                remaining[key].append(value)
        assert sampler.save(path, dict(remaining))
        sampler.shutdown()

        with path.open("rb") as stream:
            reader = make_reader(stream)
            counts = Counter(channel.topic for _, channel, _ in reader.iter_messages())
            assert counts == Counter({key: frames for key in (*numeric, depth_key)}), counts
            video_counts = {}
            for attachment in reader.iter_attachments():
                if attachment.media_type == "video/mp4":
                    with av.open(BytesIO(attachment.data)) as video:
                        video_counts[attachment.name] = sum(1 for _ in video.decode(video=0))
            assert video_counts == {key: frames for key in camera_keys}, video_counts

        with path.open("rb") as stream:
            reader = McapFlatBuffersReader(stream)
            decoded_depth = 0
            for sample in reader.iter_message_samples(topics=[depth_key]):
                np.testing.assert_array_equal(sample[depth_key]["data"], depth)
                assert sample[depth_key]["data"].dtype == np.uint16
                decoded_depth += 1
            assert decoded_depth == frames
        print("OK: MCAP round trip (two arms, two H.264 videos, uint16 depth; 5 frames)")


if __name__ == "__main__":
    main()
