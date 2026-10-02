from mcap_data_loader.datasets.mcap_dataset import (
    McapFlatBuffersEpisodeDataset,
    McapFlatBuffersEpisodeDatasetConfig,
)
from pydantic import BaseModel
from pydantic_settings import CliApp
from pathlib import Path
from collections import defaultdict
import time
import json


class Config(BaseModel):
    """Configuration for the MCAP to DISCOVERSE conversion.
    Args:
        root (str): Root directory containing the task data.
        task_name (str): Name of the task to process.
        output_dir (str): Directory to save the output files. If not provided,
            it defaults to `<root>/raw/<task_name>`.
    """

    root: Path
    task_name: str
    output_dir: str = ""


config = CliApp.run(Config)

start = time.perf_counter()
directory = config.root / config.task_name
output_dir = (
    Path(config.output_dir)
    if config.output_dir
    else (config.root / "raw" / config.task_name)
)
output_dir.mkdir(parents=True, exist_ok=True)
dataset = McapFlatBuffersEpisodeDataset(
    McapFlatBuffersEpisodeDatasetConfig(data_root=directory, topics=None)
)

for index, episode in enumerate(dataset):
    folder_dir = output_dir / episode.config.data_root.stem.zfill(3)
    folder_dir.mkdir(parents=True, exist_ok=True)
    print(f"{folder_dir=}")
    # extract video attachments
    for attach in episode.reader.reader.iter_attachments():
        if attach.media_type == "video/mp4":
            name = attach.name.removeprefix("/").replace("/", ".") + ".mp4"
            output_path = folder_dir / name
            print(f"Extracting video attachment: {output_path}")
            with open(output_path, "wb") as f:
                f.write(attach.data)
        elif attach.name == "log_stamps":
            stamps = json.loads(attach.data)
    # extract obs_action.json
    obs_action = defaultdict(list)
    # {"time": stamps}
    for sample in episode:
        for key, value in sample.items():
            obs_action[key].append(value["data"].tolist())
    obs_action["time"] = stamps
    output_path = folder_dir / "obs_action.json"
    print(f"Writing obs_action.json to: {output_path}")
    with open(output_path, "w") as f:
        json.dump(obs_action, f, indent=4)

print(
    f"Time taken: {time.perf_counter() - start:.3f} seconds of {len(dataset)} folders"
)
