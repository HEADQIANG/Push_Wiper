from airdc.common.systems.mcap_player import (
    McapPlayer,
    McapPlayerConfig,
    McapDatasetConfig,
)
from airbot_ie.robots.airbot_mmk import AIRBOTMMK, AIRBOTMMKConfig
from airdc.common.systems.basis import SystemMode
from typing import Optional, List
from mmk2_types.types import RobotComponents
import numpy as np


class MMKMcapDataReplay:
    def __init__(self, file_path: str, topics: Optional[List[str]], ip: str):
        components = [
            RobotComponents.LEFT_ARM,
            RobotComponents.LEFT_ARM_EEF,
            RobotComponents.RIGHT_ARM,
            RobotComponents.RIGHT_ARM_EEF,
            RobotComponents.HEAD,
            RobotComponents.SPINE,
        ]
        self.topics = topics or [
            f"/mmk/action/{component.value}/joint_state/position"
            for component in components
        ]
        self.config = McapPlayerConfig(
            source=McapDatasetConfig(
                data_root=file_path,
                topics=self.topics,
            )
        )
        self._mcap_player = McapPlayer(self.config)
        self._robot = AIRBOTMMK(
            AIRBOTMMKConfig(ip=ip, components=components, demonstrate=False)
        )

    def configure(self) -> bool:
        return self._mcap_player.configure() and self._robot.configure()

    def reset(self) -> bool:
        self._mcap_player.send_action(0)
        self._robot.switch_mode(SystemMode.RESETTING)
        if self.update():
            return self._robot.switch_mode(SystemMode.SAMPLING)
        return False

    def update(self) -> bool:
        if obs := self._mcap_player.capture_observation():
            act_pos = obs["/mmk/action/left_arm/joint_state/position"]
            print(f"{act_pos=}")
            self._robot.send_action(obs)
            input("Press Enter to continue...")
            cur_pos = self._robot.capture_observation()[
                "observation/left_arm/joint_state/position"
            ]["data"]
            print(f"{cur_pos=}")
            print(f"delta_pos={(act_pos - np.array(cur_pos)).tolist()}")
            return True
        else:
            return False

    def shutdown(self) -> bool:
        return self._mcap_player.shutdown() and self._robot.shutdown()


if __name__ == "__main__":
    from airdc.utils import init_logging
    import argparse
    import time
    from logging import getLogger
    from itertools import count

    parser = argparse.ArgumentParser()
    parser.add_argument("file_path", type=str)
    parser.add_argument("-f", "--fps", type=int, default=15)
    parser.add_argument("-ip", "--ip", type=str, default="192.168.11.200")
    args = parser.parse_args()

    period = 1.0 / args.fps

    init_logging()
    logger = getLogger("MMKMcapDataReplay")

    mmk_replay = MMKMcapDataReplay(args.file_path, None, args.ip)
    assert mmk_replay.configure()
    assert mmk_replay.reset()
    logger.info("Press Enter to start replay...")
    input()
    logger.info("Starting replay...")
    try:
        for step in count():
            logger.info(f"step: {step}")
            start = time.perf_counter()
            if mmk_replay.update():
                sleep_time = period - (time.perf_counter() - start)
                # if sleep_time > 0:
                #     time.sleep(sleep_time)
            else:
                logger.info("Replay finished.")
                break
    except KeyboardInterrupt:
        pass
    logger.info("Shutting down...")
    assert mmk_replay.shutdown()
