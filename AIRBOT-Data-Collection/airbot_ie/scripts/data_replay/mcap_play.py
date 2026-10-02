"""AIRBOT Play MCAP data replay."""

from airdc.common.systems.mcap_player import (
    McapPlayer,
    McapPlayerConfig,
    McapDatasetConfig,
)
# from airbot_ie.robots.airbot_play import AIRBOTPlay, AIRBOTPlayConfig

from airbot_ie.robots.airbot_play_mock import AIRBOTPlay, AIRBOTPlayConfig
from airdc.common.systems.basis import SystemMode, ActionConfigs
from typing import List
from pprint import pformat
import logging
import numpy as np


class AIRBOTPlayMcapDataReplay:
    def __init__(
        self,
        file_path: str,
        topics: List[str],
        url: str,
        action_cfg: ActionConfigs,
        eef_threshold: float = 0.0,
    ):
        components = ["arm", "eef"]
        self.topics = topics
        self.config = McapPlayerConfig(
            source=McapDatasetConfig(data_root=file_path, topics=self.topics)
        )
        self._mcap_player = McapPlayer(self.config)
        if "log_level" in AIRBOTPlayConfig.model_fields:
            kwargs = {"log_level": logging.DEBUG}
        else:
            kwargs = {}

        self._robot = AIRBOTPlay(
            AIRBOTPlayConfig(
                url=url, components=components, action=action_cfg, **kwargs
            )
        )
        self._eef_threshold = eef_threshold

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
            self.get_logger().info(f"\n{pformat(obs)}")
            if self._eef_threshold > 0.0:
                for key in obs:
                    if key.endswith("eef/joint_state/position"):
                        position = obs[key]["data"]
                        position = np.where(
                            position < self._eef_threshold, 0.0, position
                        )
                        obs[key]["data"] = position
            self._robot.send_action(obs)
            # input("Press Enter to continue...")
            return True
        else:
            return False

    def shutdown(self) -> bool:
        return self._mcap_player.shutdown() and self._robot.shutdown()

    def get_logger(self) -> logging.Logger:
        return logging.getLogger(self.__class__.__name__)


if __name__ == "__main__":
    import argparse
    import time
    from logging import getLogger
    from itertools import count
    from airdc.utils import init_logging
    from airdc.common.configs.control import (
        PosePlan,
        PoseServo,
        JointPositionPlan,
        JointPositionServo,
    )

    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("file_path", type=str)
    parser.add_argument("-f", "--fps", type=int, default=20)
    parser.add_argument("-ip", "--ip", type=str, default="localhost")
    parser.add_argument("-eth", "--eef-threshold", type=float, default=0.04)
    args = parser.parse_args()

    period = 1.0 / args.fps

    init_logging()
    logger = getLogger("AIRBOTPlayMcapDataReplay")
    # 1 - 1 pose
    # topics = [
    #     "/lead/eef/pose/position",
    #     "/lead/eef/pose/orientation",
    #     "/lead/eef/joint_state/position",
    # ]
    topics = [
        "/follow/arm/joint_state/position",
        "/follow/eef/joint_state/position",
    ]
    pose_control = False
    # 2 - 2
    # topics = [f"/left{topic}" for topic in topics]
    airbot_replay = AIRBOTPlayMcapDataReplay(
        args.file_path,
        topics,
        args.ip,
        # [
        #     {SystemMode.RESETTING: PosePlan(), SystemMode.SAMPLING: PoseServo()},
        #     {
        #         SystemMode.RESETTING: JointPositionPlan(),
        #         SystemMode.SAMPLING: JointPositionServo(),
        #     },
        # ],
        [
            {
                SystemMode.RESETTING: JointPositionPlan(),
                SystemMode.SAMPLING: JointPositionServo(),
            },
        ],
        args.eef_threshold,
    )
    assert airbot_replay.configure()
    assert airbot_replay.reset()
    logger.info("Press Enter to start replay...")
    input()
    logger.info("Starting replay...")
    try:
        for step in count():
            logger.info(f"step: {step}")
            start = time.perf_counter()
            if airbot_replay.update():
                sleep_time = period - (time.perf_counter() - start)
                if sleep_time > 0:
                    time.sleep(sleep_time)
            else:
                logger.info("Replay finished.")
                break
            input("Press Enter to continue to next step...")
    except KeyboardInterrupt:
        pass
    logger.info("Shutting down...")
    assert airbot_replay.shutdown()
