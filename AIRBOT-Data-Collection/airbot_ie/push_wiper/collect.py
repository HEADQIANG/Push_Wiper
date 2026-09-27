"""Keyboard-driven segmented collection; --demo never connects to hardware."""

import argparse
import json
import logging
import os
import time
from contextlib import ExitStack
from pathlib import Path

import cv2
import numpy as np

from .config import CollectionConfig, WorkHeightReference, atomic_json, load_config
from .clock import acquisition_ns
from .controller import (
    CommandError,
    ControllerRunner,
    DualArmController,
    SingleArmDragController,
)
from .devices import CameraWorker, MockArm, SDKArm, lock_robot_endpoints
from .geometry import quaternion, vector
from .recording import identifier
from .session import CollectionSession

HELP = """Push-Wiper（请聚焦视频窗口按键）：
O 登记观察姿态并精确到位（仅未开始任务时）。
Z 登记贴桌时执行臂的工作高度，跨任务及重启复用（仅未开始任务时）。
G 恢复主从；R 人工粗定位后的双臂精确复位；N 新任务。
C 起始/结束图像；空格 开始/结束有效推动；S 接受本段；Q 拒绝本段。
T 结束任务；Esc 退出，未完成段标为 incomplete；H 显示帮助。
启动不自动移动。按 G 平滑接入主从，不限制两臂之间的关节差。
O/Z 登记时请人工停稳；Z 只保存高度，不移动机械臂、不自动判定接触。
R 前人工抬离桌面并返回观察位附近。
不检查静止速度或复位近位范围；复位仍需两臂各自到位后才能拍照。
首次 G 到观察位后 O；再 G 贴桌后 Z；抬起返回附近 R，等待 observing 后 N、C。
模拟模式可用方向键、[ ]、逗号/句号移动虚拟示教臂。
"""

SINGLE_DRAG_HELP = """Push-Wiper 单臂拖动（请聚焦视频窗口按键）：
O 登记观察姿态并精确到位（仅未开始任务时）。
Z 登记贴桌时执行臂的工作高度，跨任务及重启复用（仅未开始任务时）。
G 进入重力补偿，直接拖动执行臂；R 单臂精确复位；N 新任务。
C 起始/结束图像；空格 开始/结束有效推动；S 接受本段；Q 拒绝本段。
T 结束任务；Esc 退出，未完成段标为 incomplete；H 显示帮助。
启动不自动移动。拖动期间只读反馈；第二次空格后仍可拖动。
O/Z 登记时请人工停稳；Z 只保存高度，不移动机械臂、不自动判定接触。
R 前手动抬离桌面、返回观察位附近并松手；等待 observing 后才能拍照。
首次 G 到观察位后 O；再 G 贴桌后 Z；抬起返回附近 R，等待 observing 后 N、C。
模拟模式可用方向键、[ ]、逗号/句号移动虚拟执行臂。
"""


def create_controller(config, arms, observation=None):
    if config.robot.mode == "single_drag":
        return SingleArmDragController(arms["follow"], config.reset, observation)
    return DualArmController(arms["lead"], arms["follow"], config.reset, observation)


def check_control_status(snapshot, robot_config, now_ns=None):
    """Bound command waits separately; return whether feedback can be sampled."""
    now_ns = acquisition_ns() if now_ns is None else now_ns
    age_s = (now_ns - snapshot["published_ns"]) / 1e9
    active = snapshot.get("active_command")
    if active is not None:
        elapsed_s = (now_ns - active["started_ns"]) / 1e9
        # Only explicit mode-switch/planning commands get the longer deadline.
        # Queued commands and ordinary control ticks retain the feedback limit.
        limit_s = (
            robot_config.command_timeout_s
            if active["name"] in {
                "resume_follow", "reset_to_observation", "register_observation_pose"
            }
            else robot_config.feedback_timeout_s
        )
        if elapsed_s > limit_s:
            raise RuntimeError(
                f"Control command timed out: {active['name']}, "
                f"elapsed={elapsed_s:.3f}s, limit={limit_s:.3f}s, "
                f"last_state={snapshot['state']}, feedback_age={age_s:.3f}s"
            )
        return False
    if age_s > robot_config.feedback_timeout_s:
        raise RuntimeError(
            "Control loop is unresponsive: "
            f"state={snapshot['state']}, status_age={age_s:.3f}s, "
            f"limit={robot_config.feedback_timeout_s:.3f}s, no command running"
        )
    return True


def bindings(config, simulated):
    return {
        "mode": config.robot.mode,
        "simulated": simulated,
        "camera": config.camera.model_dump(mode="json"),
        **{
            name: [
                getattr(config.robot, name + "_url"),
                getattr(config.robot, name + "_port"),
            ]
            for name in config.robot.arm_names
        },
        "quaternion_order": "xyzw",
        "action_reference": "sdk_end_reference",
    }


def load_references(path, config, simulated):
    expected = bindings(config, simulated)
    if not path.exists():
        return {
            "schema_version": 1,
            "version": identifier("reference"),
            "bindings": expected,
        }
    references = json.loads(path.read_text())
    saved_bindings = dict(references.get("bindings") or {})
    # Reference files created before mode selection always described dual arms.
    saved_bindings.setdefault("mode", "teleop")
    if references.get("schema_version") != 1 or saved_bindings != expected:
        raise ValueError(
            "Reference configuration differs (mode/camera/ROI/endpoints/mock); use a new --references file"
        )
    references["bindings"] = expected
    observation = references.get("observation")
    if observation:
        if set(observation) != set(config.robot.arm_names):
            raise ValueError("Reference arms differ; use a new --references file")
        for name in config.robot.arm_names:
            vector(observation[name]["joints"], 6)
            vector(observation[name]["position"], 3)
            quaternion(observation[name]["orientation"])
    # Ignore the retired reference in existing files; keep the saved O pose.
    references.pop("contact", None)
    if references.get("work_height") is not None:
        WorkHeightReference.model_validate(references["work_height"])
    return references


class MockCamera:
    def __init__(self, config):
        self.info = {"name": "SIMULATED camera", "simulated": True}
        self.image = np.full((config.height, config.width, 3), 245, np.uint8)
        cv2.circle(
            self.image,
            (config.width // 2, config.height // 2),
            max(8, config.height // 10),
            (30, 70, 210),
            -1,
        )

    def start(self):
        pass

    def latest(self):
        return {"data": self.image.copy(), "t": acquisition_ns()}

    def close(self):
        pass


def draw(frame, snapshot, session, message, mock):
    image = frame["data"].copy()
    roi = session.config.camera.roi
    if roi:
        x, y, width, height = roi
        cv2.rectangle(image, (x, y), (x + width - 1, y + height - 1), (0, 200, 0), 1)
    height = session.references.get("work_height")
    height_text = f"{height['z_m']:.5f} m" if height is not None else "MISSING"
    single_drag = session.config.robot.mode == "single_drag"
    state = snapshot["state"]
    if single_drag and state == "following":
        state = "dragging"
    motion = "drag" if single_drag else "follow"
    lines = [
        f"{'MOCK | ' if mock else ''}{session.config.robot.mode} | {state} | segment: {session.phase}",
        f"Task: {session.task['task_id'] if session.task else 'none'}",
        "References: O observation="
        + ("ready" if session.references.get("observation") else "MISSING"),
        f"Z work height: {height_text}",
        f"O observe ref | Z work height | G {motion} | R precise reset",
        "N task | C photo | SPACE stroke | S accept | Q reject | T end",
        "Esc quit | H help",
    ]
    if state == "settling":
        lines.append("Capture paused: wait for observing, or press G/R")
    if snapshot.get("active_command"):
        lines.append("Waiting for robot command; recording paused")
    follow = snapshot.get("samples", {}).get("follow")
    if follow:
        lines.append(f"Follower SDK Z: {follow['position'][2]:.5f} m")
    for name, errors in snapshot.get("errors", {}).items():
        lines.append(
            f"{name}: {errors['position_m'] * 1000:.1f}mm {errors['angle_deg']:.2f}deg q={errors['joint_rad']:.4f}"
        )
    if session.writer:
        lines.append(f"Recorded: {session.writer.metadata['sample_count']} samples")
    lines.append(message[:100])
    panel = np.full((len(lines) * 23 + 10, max(image.shape[1], 800), 3), 30, np.uint8)
    for index, line in enumerate(lines):
        cv2.putText(
            panel,
            line,
            (8, 22 + index * 23),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (240, 240, 240),
            1,
        )
    canvas = np.full((image.shape[0] + panel.shape[0], panel.shape[1], 3), 30, np.uint8)
    canvas[: image.shape[0], : image.shape[1]] = image
    canvas[image.shape[0] :] = panel
    return canvas


def run_collection(config, mock=False):
    if not mock and not config.camera.serial:
        raise ValueError("Real collection requires camera.serial or --serial")
    if os.name != "nt" and not (
        os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    ):
        raise RuntimeError(
            "Interactive collection needs a desktop; use --demo for headless testing"
        )
    references_path = Path(config.references)
    references = load_references(references_path, config, mock)
    session = CollectionSession(config, references, simulated=mock)
    window = "Push-Wiper collection"
    runner = None
    help_text = SINGLE_DRAG_HELP if config.robot.mode == "single_drag" else HELP
    print(help_text, flush=True)
    with ExitStack() as resources:
        if not mock:
            endpoints = [
                f"{getattr(config.robot, name + '_url')}:{getattr(config.robot, name + '_port')}"
                for name in config.robot.arm_names
            ]
            resources.enter_context(lock_robot_endpoints(endpoints))
        camera = MockCamera(config.camera) if mock else CameraWorker(config.camera)
        resources.callback(camera.close)
        camera.start()  # Camera must work before connecting the robots.
        arms = {}
        for name in config.robot.arm_names:
            arm = (
                MockArm()
                if mock
                else SDKArm(
                    getattr(config.robot, name + "_url"),
                    getattr(config.robot, name + "_port"),
                    config.robot.feedback_timeout_s,
                )
            )
            resources.callback(arm.close)
            arm.connect()
            arms[name] = arm
            previous = references.get("devices", {}).get(name, {})
            if previous.get("sn") and previous["sn"] != arm.info.get("sn"):
                raise ValueError(
                    f"{name} robot identity changed; register new references"
                )
        references["devices"] = {name: arm.info for name, arm in arms.items()} | {
            "camera": camera.info
        }
        controller = create_controller(config, arms, references.get("observation"))
        runner = ControllerRunner(controller, config.robot.control_hz)
        runner.start()
        resources.callback(runner.close)
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        resources.callback(cv2.destroyAllWindows)
        pending = None
        message = "Idle: no motion until an explicit command"
        next_sample = time.monotonic()
        exit_reason = "Operator exited before segment submission"
        try:
            while True:
                frame = camera.latest()
                snapshot = runner.snapshot()
                if snapshot["state"] == "fault":
                    raise RuntimeError(
                        f"Controller fault: {snapshot['error']}; hold failures: {snapshot['hold_errors']}"
                    )
                feedback_ready = check_control_status(snapshot, config.robot)
                if pending is not None and pending[1].done():
                    action, future = pending
                    pending = None
                    try:
                        value = future.result()
                        if action in {"observation", "work_height"}:
                            references[action] = value
                            references["version"] = identifier("reference")
                            atomic_json(references_path, references)
                            message = f"Registered {action} reference"
                            if action == "work_height":
                                message = f"Registered work height: {value['z_m']:.5f} m (follower SDK Z)"
                        else:
                            message = f"Command completed: {action}"
                    except CommandError as exc:
                        message = str(exc)
                    print(message, flush=True)
                    # A command may finish after the snapshot at the top of
                    # this iteration. Use its newly published feedback now.
                    snapshot = runner.snapshot()
                    feedback_ready = check_control_status(snapshot, config.robot)
                if (
                    snapshot["samples"]
                    and feedback_ready
                    and pending is None
                    and session.writer
                    and time.monotonic() >= next_sample
                ):
                    try:
                        session.sample(snapshot, frame)
                    except CommandError as exc:
                        message = str(exc)
                    next_sample += 1 / config.sample_hz
                    if next_sample < time.monotonic():
                        next_sample = time.monotonic() + 1 / config.sample_hz
                cv2.imshow(window, draw(frame, snapshot, session, message, mock))
                key = cv2.waitKeyEx(10)
                if key == 27 or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                    break
                if key < 0:
                    continue
                if key in range(ord("A"), ord("Z") + 1):
                    key += 32
                if pending is not None or not feedback_ready:
                    message = "Waiting for the previous controller command"
                    continue
                try:
                    if key == ord("h"):
                        print(help_text, flush=True)
                    elif key == ord("n"):
                        session.new_task()
                        message = "New task created"
                    elif key == ord("r"):
                        session.allow_reset()
                        pending = ("reset", runner.submit("reset_to_observation"))
                    elif key == ord("g"):
                        session.allow_follow()
                        motion = (
                            "drag" if config.robot.mode == "single_drag" else "follow"
                        )
                        pending = (motion, runner.submit("resume_follow"))
                    elif key == ord("c"):
                        session.capture(snapshot, frame)
                        message = "Captured image: " + session.phase
                    elif key == 32:
                        session.toggle_push(snapshot, frame)
                        message = "Stroke state: " + session.phase
                    elif key in (ord("s"), ord("q")):
                        result = session.finish_segment(accepted=key == ord("s"))
                        message = "Saved: " + result.name
                    elif key == ord("t"):
                        session.end_task()
                        message = "Task completed"
                    elif key in (ord("o"), ord("z")):
                        session.allow_reference_update()
                        action, command = (
                            ("observation", "register_observation_pose")
                            if key == ord("o")
                            else ("work_height", "register_work_height")
                        )
                        pending = (
                            action,
                            runner.submit(command),
                        )
                    elif mock:
                        mapping = {
                            65361: (0, -0.005),
                            65363: (0, 0.005),
                            65362: (1, 0.005),
                            65364: (1, -0.005),
                            ord("["): (2, -0.005),
                            ord("]"): (2, 0.005),
                            ord(","): (5, -0.02),
                            ord("."): (5, 0.02),
                        }
                        if key in mapping:
                            index, value = mapping[key]
                            delta = [0.0] * 6
                            delta[index] = value
                            pending = (
                                "mock_move",
                                runner.submit("simulate_lead", delta=delta),
                            )
                except CommandError as exc:
                    message = str(exc)
                print(message, flush=True)
        except BaseException as exc:
            exit_reason = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            # Stop motion before potentially slow video flushing.
            runner.request_stop(exit_reason)
            try:
                runner.close()
            except Exception as exc:
                logging.getLogger(__name__).debug(
                    "Robot hold request failed", exc_info=True
                )
                print(f"Could not confirm robot hold: {exc}", flush=True)
            session.abort(exit_reason)
            if session.task is not None:
                session.end_task(interrupted=True)


def run_demo(config):
    """Two tasks / two strokes each, real MCAP serialization, synthetic devices."""
    arms = {name: MockArm() for name in config.robot.arm_names}
    controller = create_controller(config, arms)
    manual_arm = arms.get("lead", arms["follow"])
    now = 0.0
    clock_ns = acquisition_ns()
    camera = MockCamera(config.camera)

    def step(count=1):
        nonlocal now, clock_ns
        for _ in range(count):
            now += 0.05
            clock_ns += 50_000_000
            controller.tick(now)
        snapshot = controller.get_status()
        for sample in snapshot["samples"].values():
            sample["t_ns"] = clock_ns
        frame = {"data": camera.image.copy(), "t": clock_ns}
        return snapshot, frame

    step(12)
    observation = controller.register_observation_pose(now)
    step(15)
    references = {
        "schema_version": 1,
        "version": identifier("reference"),
        "bindings": bindings(config, True),
        "observation": observation,
        "work_height": controller.register_work_height(now),
    }
    session = CollectionSession(config, references, simulated=True)
    try:
        for _ in range(2):
            session.new_task()
            for _ in range(2):
                snapshot, frame = step()
                session.capture(snapshot, frame, clock_ns)
                controller.resume_follow(now)
                snapshot, frame = step(2)
                session.toggle_push(snapshot, frame, clock_ns)
                for index in range(1, 21):
                    manual_arm.joints[0] = index * 0.001
                    manual_arm.joints[1] = np.sin(index / 20 * np.pi) * 0.005
                    manual_arm.joints[5] = index * 0.002
                    snapshot, frame = step(2)
                    session.sample(snapshot, frame, clock_ns)
                snapshot, frame = step()
                session.toggle_push(snapshot, frame, clock_ns)
                manual_arm.joints[:] = 0
                snapshot, frame = step(12)
                session.sample(snapshot, frame, clock_ns)
                controller.reset_to_observation(now)
                snapshot, frame = step(15)
                session.sample(snapshot, frame, clock_ns)
                snapshot, frame = step()
                session.capture(snapshot, frame, clock_ns)
                print(session.finish_segment())
            session.end_task()
    finally:
        session.abort("Demo interrupted")
    print("SIMULATED: 2 tasks, 4 segments; no hardware was accessed.")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", help="Collection JSON configuration")
    parser.add_argument(
        "--mode",
        choices=("teleop", "single_drag"),
        help="Dual-arm teleoperation or direct dragging of the follow arm",
    )
    parser.add_argument(
        "--serial", help="RealSense serial, required for real collection"
    )
    parser.add_argument("--output", help="Raw dataset root")
    parser.add_argument(
        "--references", help="Observation pose and work height reference JSON"
    )
    parser.add_argument(
        "--stain", help="Task stain label, e.g. ketchup or peanut_butter"
    )
    parser.add_argument(
        "--mock", action="store_true", help="Interactive simulated robot/camera"
    )
    parser.add_argument(
        "--demo", action="store_true", help="Headless simulated end-to-end collection"
    )
    parser.add_argument(
        "--check-config", action="store_true", help="Validate config, without devices"
    )
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.WARNING)
    try:
        config = load_config(args.config)
        data = config.model_dump()
        if args.mode is not None:
            data["robot"]["mode"] = args.mode
        for field in ("output", "references", "stain"):
            if getattr(args, field) is not None:
                data[field] = getattr(args, field)
        if args.serial is not None:
            data["camera"]["serial"] = args.serial
        config = CollectionConfig.model_validate(data)
        if args.check_config:
            print(config.model_dump_json(indent=2))
        elif args.demo:
            run_demo(config)
        else:
            run_collection(config, mock=args.mock)
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception:
        logging.getLogger(__name__).exception("Push-Wiper stopped")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
