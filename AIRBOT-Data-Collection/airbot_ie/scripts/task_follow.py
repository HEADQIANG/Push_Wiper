#!/usr/bin/env python3
"""
Make one arm follow the movement of another arm.
"""

import logging

LOG_FORMAT = (
    "[%(asctime)s] [%(levelname)-8s] [%(name)s.%(funcName)s:%(lineno)d] - %(message)s"
)


def follow():
    """
    Follow the lead robot arm
    """
    import argparse

    logging.basicConfig(
        level=logging.DEBUG,
        format=LOG_FORMAT,
        datefmt="%Y-%m-%d %H:%M:%S",
        handlers=[logging.StreamHandler()],
    )
    logger = logging.getLogger(__name__)
    parser = argparse.ArgumentParser(
        description="Multi-pair lead/follow control example"
    )
    parser.add_argument(
        "--lead-url",
        type=str,
        nargs="+",
        help="URL(s) to connect to the lead server(s), space separated for multiple.",
    )
    parser.add_argument(
        "--follow-url",
        type=str,
        nargs="+",
        help="URL(s) to connect to the follow server(s), space separated for multiple.",
    )
    parser.add_argument(
        "-lp",
        "--lead-port",
        type=int,
        nargs="+",
        required=True,
        help="Server port(s) for arm(s) to lead, space separated for multiple.",
    )
    parser.add_argument(
        "-fp",
        "--follow-port",
        type=int,
        nargs="+",
        required=True,
        help="Server port(s) for arm(s) to follow, space separated for multiple.",
    )
    parser.add_argument(
        "--no-eef",
        action="store_true",
        help="Only follow arm joints; never read or command a gripper.",
    )

    args = parser.parse_args()
    n = len(args.lead_port)
    if not args.lead_url:
        args.lead_url = ["localhost"] * n
    if not args.follow_url:
        args.follow_url = ["localhost"] * n
    if not (
        len(args.lead_url)
        == len(args.lead_port)
        == len(args.follow_url)
        == len(args.follow_port)
    ):
        raise ValueError("The number of lead/follow urls and ports must be the same.")
    for i in range(n):
        if (
            args.lead_port[i] == args.follow_port[i]
            and args.lead_url[i] == args.follow_url[i]
        ):
            raise ValueError(
                f"Pair {i}: Lead and follow port, lead and follow url cannot be the same at the same time."
            )

    from airbot_ie.push_wiper.devices import lock_robot_endpoints

    endpoints = [
        f"{url}:{port}"
        for url, port in zip(
            args.lead_url + args.follow_url, args.lead_port + args.follow_port
        )
    ]
    with lock_robot_endpoints(endpoints):
        _run_follow(args, logger)


def _run_follow(args, logger):
    from contextlib import ExitStack

    from airbot_py.arm import RobotMode, SpeedProfile

    with ExitStack() as resources:
        robots = _connect_pairs(args, resources)
        _follow_connected(args, logger, robots, RobotMode, SpeedProfile)


def _connect_pairs(args, resources):
    from airbot_py.arm import AIRBOTPlay

    robots = []
    n = len(args.lead_port)
    for i in range(n):
        lead = AIRBOTPlay(url=args.lead_url[i], port=args.lead_port[i])
        follow = AIRBOTPlay(url=args.follow_url[i], port=args.follow_port[i])
        resources.callback(lead.disconnect)
        resources.callback(follow.disconnect)
        print(
            f"Connecting to lead {args.lead_url[i]}:{args.lead_port[i]} and follow {args.follow_url[i]}:{args.follow_port[i]}"
        )
        assert lead.connect()
        assert follow.connect()
        robots.append((lead, follow))
    return robots


def _follow_connected(args, logger, robots, RobotMode, SpeedProfile):
    import time

    try:
        for lead, follow in robots:
            lead.switch_mode(RobotMode.PLANNING_POS)
            if (
                sum(
                    abs(a - b)
                    for a, b in zip(lead.get_joint_pos(), follow.get_joint_pos())
                )
                > 0.1
            ):
                follow.switch_mode(RobotMode.PLANNING_POS)
                follow.move_to_joint_pos(lead.get_joint_pos())
                time.sleep(1)
            logger.info(f"Lead robot arm {lead} is ready to follow.")
        for lead, follow in robots:
            lead.switch_mode(RobotMode.GRAVITY_COMP)
            follow.switch_mode(RobotMode.SERVO_JOINT_POS)
            # follow.set_speed_profile(SpeedProfile.FAST)
            follow.set_params(
                {
                    "servo_node.moveit_servo.scale.linear": 10.0,
                    "servo_node.moveit_servo.scale.rotational": 10.0,
                    "servo_node.moveit_servo.scale.joint": 1.0,
                    "sdk_server.max_velocity_scaling_factor": 1.0,
                    "sdk_server.max_acceleration_scaling_factor": 0.5,
                }
            )
        factor = 0.072 / 0.0471
        while True:
            for lead, follow in robots:
                follow.servo_joint_pos(lead.get_joint_pos())
                if not args.no_eef:
                    eef_pos = lead.get_eef_pos()
                    if eef_pos:
                        eef_pos[0] *= factor
                    else:
                        logger.info("Failed to get lead eef pos")
                    follow.servo_eef_pos(eef_pos or [])
            time.sleep(0.01)
    finally:
        for lead, follow in robots:
            lead.switch_mode(RobotMode.PLANNING_POS)
            follow.switch_mode(RobotMode.PLANNING_POS)
            follow.set_speed_profile(SpeedProfile.DEFAULT)


if __name__ == "__main__":
    follow()
