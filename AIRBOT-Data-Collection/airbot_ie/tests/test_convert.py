from typing import Dict
from pprint import pprint

act_obs = {
    "time": [0.0],
    "obs": {
        "jq": [[0.0] * 7],
        "jv": [[0.0] * 7],
        "tau": [[0.0] * 7],
        "eef_pos": [[0.0] * 3],
        "eef_quat": [[0.0] * 4],
        "eef_vel": [[0.0] * 6],
        "eef_gyro": [[0.0] * 3],
    },
    "act": [[0.0] * 3],
}

topic_mapping: Dict[str, dict] = {
    "jq": {
        "/follow/arm/joint_states/position": slice(0, 6),
        "/follow/eef/joint_states/position": slice(6, 7),
    },
    "eef_pos": "/follow/arm/pose/position",
    "end_force": {
        "/follow/arm/wrench/force": slice(0, 3),
        "/follow/arm/wrench/torque": slice(3, 6),
    },
    "act": "/lead/arm/pose/position",
}

topic_names = set()
for key, value in topic_mapping.items():
    if isinstance(value, str):
        topic_mapping[key] = {value: None}
        topic_names.add(value)
    else:
        assert isinstance(value, dict), f"Value for key {key} must be a dict."
        topic_names.update(value.keys())

print(f"All topic names: {topic_names}")

pprint(topic_mapping)


keys = ("time", *act_obs["obs"].keys(), "act")

for action_values in zip(act_obs["time"], *act_obs["obs"].values(), act_obs["act"]):
    for i, value in enumerate(action_values):
        key = keys[i]
        if mapping := topic_mapping.get(key, None):
            for topic, slc in mapping.items():
                if slc:
                    print(f"{topic}: {value[slc]}")
                else:
                    print(f"{topic}: {value}")
        else:
            print(f"{key}: {value}")
