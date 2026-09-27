"""Run the standalone AIRBOT Play + LFS-6D65 force-position controller."""

from __future__ import annotations

import argparse
import csv
import json
import logging
import time
from pathlib import Path

import numpy as np

from airbot_ie.force_control.core import DragYSweepTrajectory, NpzPlanarTrajectory
from airbot_ie.force_control.hardware import (
    AirbotPlayAdapter,
    LFS6D65Reader,
    collect_bias,
)
from airbot_ie.force_control.runner import (
    HybridRunner,
    build_fixed_xy_trajectory,
    parse_runner_config,
)


def load_mapping(path: str | Path) -> dict:
    config_path = Path(path)
    with config_path.open(encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError("Configuration root must be a JSON object")
    return value


def validate_config(mapping: dict, config_path: Path) -> None:
    parse_runner_config(mapping)
    robot = mapping.get("robot", {})
    sensor = mapping.get("sensor", {})
    trajectory = mapping.get("trajectory", {})
    mode = trajectory.get("mode", "fixed_xy")
    if mode == "fixed_xy":
        standalone = mapping.get("standalone", {})
        fixed_xy = standalone.get("fixed_xy")
        if fixed_xy is None or len(fixed_xy) != 2:
            raise ValueError("standalone.fixed_xy must contain two values")
        nominal_z = standalone.get("surface_z_m", "auto from current pose")
        print(
            f"配置有效: {config_path}\n"
            f"  robot={robot.get('url', 'localhost')}:{robot.get('port', 50051)}\n"
            f"  sensor={sensor.get('port', '/dev/ttyUSB0')}\n"
            f"  mode=fixed_xy, fixed_xy={fixed_xy}, surface_z_m={nominal_z}"
        )
        return
    if mode == "npz":
        npz_path = trajectory.get("npz")
        if not npz_path or not Path(npz_path).exists():
            raise ValueError(f"trajectory.npz does not exist: {npz_path}")
        surface_z = trajectory.get("surface_z_m")
        NpzPlanarTrajectory.from_npz(
            npz_path,
            float(trajectory.get("duration_s", 6.0)),
            None if surface_z is None else float(surface_z),
            replay_yaw=bool(trajectory.get("replay_yaw", True)),
            align_tool_z=bool(trajectory.get("align_tool_z", True)),
        )
        height_source = (
            f"surface_z_m={surface_z}"
            if surface_z is not None
            else "surface_z_m=NPZ work_height_m or bounded current-pose approach"
        )
        print(f"配置和 NPZ 轨迹有效: {config_path}\n  npz={npz_path}\n  {height_source}")
        return
    if mode == "drag_y_sweep":
        trajectory = mapping.get("trajectory", {})
        distance = float(trajectory.get("y_distance_m", 0.10))
        speed = trajectory.get("y_speed_m_s")
        if speed is not None:
            speed = float(speed)
            duration = 3.0 * distance / speed if speed > 0 else 0.0
        else:
            duration = float(trajectory.get("duration_s", 6.0))
        if distance <= 0 or duration <= 0:
            raise ValueError("trajectory.y_distance_m and duration_s must be positive")
        if not bool(mapping.get("drag", {}).get("enabled", True)):
            raise ValueError("drag.enabled must be true for drag_y_sweep mode")
        speed_display = speed if speed is not None else 3.0 * distance / duration
        print(
            f"拖拽往返轨迹配置有效: {config_path}\n"
            f"  y_distance_m={distance}, y_speed_m_s={speed_display:.4f}, "
            f"duration_s={duration}, "
            f"align_tool_z={bool(trajectory.get('align_tool_z', True))}"
        )
        return
    raise ValueError(f"Unsupported trajectory.mode: {mode}")


def build_trajectory(mapping: dict, current_orientation, current_position=None):
    trajectory_cfg = mapping.get("trajectory", {})
    mode = trajectory_cfg.get("mode", "fixed_xy")
    if mode == "fixed_xy":
        return build_fixed_xy_trajectory(mapping, current_orientation)
    if mode == "npz":
        surface_z = trajectory_cfg.get("surface_z_m")
        return NpzPlanarTrajectory.from_npz(
            trajectory_cfg["npz"],
            float(trajectory_cfg.get("duration_s", 6.0)),
            None if surface_z is None else float(surface_z),
            replay_yaw=bool(trajectory_cfg.get("replay_yaw", True)),
            align_tool_z=bool(trajectory_cfg.get("align_tool_z", True)),
        )
    if mode == "drag_y_sweep":
        if current_position is None:
            raise ValueError("drag_y_sweep requires the confirmed dragged position")
        return DragYSweepTrajectory(
            current_position,
            current_orientation,
            distance_m=float(trajectory_cfg.get("y_distance_m", 0.10)),
            duration_s=(
                None
                if trajectory_cfg.get("y_speed_m_s") is not None
                else float(trajectory_cfg.get("duration_s", 6.0))
            ),
            align_tool_z=bool(trajectory_cfg.get("align_tool_z", True)),
            surface_z_m=(
                None
                if trajectory_cfg.get("surface_z_m") is None
                else float(trajectory_cfg["surface_z_m"])
            ),
            speed_m_s=(
                None
                if trajectory_cfg.get("y_speed_m_s") is None
                else float(trajectory_cfg["y_speed_m_s"])
            ),
        )
    raise ValueError(f"Unsupported trajectory.mode: {mode}")


def run_drag_preview(robot: AirbotPlayAdapter, sample_hz: float, csv_path: str | None) -> None:
    """Display live end-pose XYZ while the operator manually drags the arm."""

    if sample_hz <= 0:
        raise ValueError("preview sample_hz must be positive")
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError(
            "Drag preview requires matplotlib; install it in the active environment"
        ) from exc

    robot.start_gravity_comp()
    try:
        started = time.monotonic()
        times: list[float] = []
        positions: list[np.ndarray] = []
        figure, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 8))
        figure.suptitle("AIRBOT 末端位姿拖拽预览（关闭窗口或 Ctrl+C 结束）")
        lines = []
        labels = ("X", "Y", "Z")
        colors = ("tab:blue", "tab:orange", "tab:green")
        for axis, label, color in zip(axes, labels, colors):
            line, = axis.plot([], [], color=color, label=label)
            axis.set_ylabel(f"{label} (mm)")
            axis.grid(True, alpha=0.3)
            axis.legend(loc="upper left")
            lines.append(line)
        axes[-1].set_xlabel("time (s)")
        figure.tight_layout()
        plt.show(block=False)
        output = None
        if csv_path:
            output = Path(csv_path)
            output.parent.mkdir(parents=True, exist_ok=True)
    except BaseException:
        robot.stop_gravity_comp()
        raise

    try:
        while plt.fignum_exists(figure.number):
            position, _ = robot.read_pose()
            times.append(time.monotonic() - started)
            positions.append(position.copy())
            values_mm = np.asarray(positions) * 1000.0
            for index, line in enumerate(lines):
                line.set_data(times, values_mm[:, index])
                axes[index].relim()
                axes[index].autoscale_view()
            delta_mm = values_mm[-1] - values_mm[0]
            figure.suptitle(
                "AIRBOT 末端位姿拖拽预览（关闭窗口或 Ctrl+C 结束）\n"
                f"当前 ΔX={delta_mm[0]:+.1f} mm, "
                f"ΔY={delta_mm[1]:+.1f} mm, ΔZ={delta_mm[2]:+.1f} mm"
            )
            figure.canvas.draw_idle()
            plt.pause(1.0 / sample_hz)
    finally:
        robot.stop_gravity_comp()
        if output and positions:
            with output.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(("t_s", "x_m", "y_m", "z_m"))
                writer.writerows(
                    [time_s, position[0], position[1], position[2]]
                    for time_s, position in zip(times, positions)
                )
        plt.close(figure)
        if positions:
            delta_mm = (positions[-1] - positions[0]) * 1000.0
            print(
                "拖拽预览结束，末端位移 "
                f"ΔX={delta_mm[0]:+.1f} mm, "
                f"ΔY={delta_mm[1]:+.1f} mm, "
                f"ΔZ={delta_mm[2]:+.1f} mm"
            )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="JSON controller configuration")
    check_group = parser.add_mutually_exclusive_group()
    check_group.add_argument(
        "--validate",
        action="store_true",
        help="Validate configuration and optional NPZ without connecting hardware",
    )
    check_group.add_argument(
        "--sensor-check",
        action="store_true",
        help="Check LFS-6D65 communication and print a short unloaded bias; do not move AIRBOT",
    )
    check_group.add_argument(
        "--drag-preview",
        action="store_true",
        help="Show live XYZ end-pose curves while manually dragging AIRBOT; no force sensor required",
    )
    parser.add_argument(
        "--preview-hz",
        type=float,
        default=20.0,
        help="Sampling rate for --drag-preview (default: 20 Hz)",
    )
    parser.add_argument(
        "--preview-csv",
        help="Optional CSV output path for --drag-preview",
    )
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    config_path = Path(args.config).resolve()
    mapping = load_mapping(config_path)
    if args.validate:
        validate_config(mapping, config_path)
        return 0

    sensor_cfg = mapping.get("sensor", {})
    if args.sensor_check:
        reader = LFS6D65Reader(
            sensor_cfg.get("port", "/dev/ttyUSB0"),
            timeout_s=float(sensor_cfg.get("timeout_s", 1.0)),
        )
        try:
            reader.zero()
            time.sleep(float(sensor_cfg.get("zero_settle_s", 0.1)))
            bias = collect_bias(
                reader,
                duration_s=float(mapping.get("force", {}).get("bias_duration_s", 2.0)),
                poll_interval_s=float(sensor_cfg.get("poll_interval_s", 0.02)),
            )
            print(
                "LFS-6D65 communication OK; unloaded mean "
                f"[FX,FY,FZ,MX,MY,MZ]={bias.tolist()}"
            )
            return 0
        finally:
            reader.close()

    if args.drag_preview:
        robot_cfg = mapping.get("robot", {})
        robot = AirbotPlayAdapter(
            url=robot_cfg.get("url", "localhost"),
            port=int(robot_cfg.get("port", 50051)),
        )
        try:
            robot.connect()
            run_drag_preview(robot, args.preview_hz, args.preview_csv)
            return 0
        except KeyboardInterrupt:
            logging.getLogger(__name__).warning("Drag preview cancelled by operator")
            return 130
        finally:
            robot.stop()
            robot.close()

    robot_cfg = mapping.get("robot", {})
    robot = AirbotPlayAdapter(
        url=robot_cfg.get("url", "localhost"),
        port=int(robot_cfg.get("port", 50051)),
    )
    reader = LFS6D65Reader(
        sensor_cfg.get("port", "/dev/ttyUSB0"),
        timeout_s=float(sensor_cfg.get("timeout_s", 1.0)),
    )
    try:
        robot.connect()
        trajectory_cfg = mapping.get("trajectory", {})
        drag_cfg = mapping.get("drag", {})
        trajectory_mode = trajectory_cfg.get("mode", "fixed_xy")
        drag_enabled = trajectory_mode == "drag_y_sweep"
        if drag_enabled and not bool(drag_cfg.get("enabled", True)):
            raise ValueError("drag.enabled must be true for drag_y_sweep mode")
        if drag_enabled:
            prompt = drag_cfg.get(
                "prompt",
                "请拖拽机械臂到初始位置，确认工具悬空且姿态正确后按 Enter：",
            )
            current_position, orientation = robot.drag_to_pose(str(prompt))
        else:
            current_position, orientation = robot.read_pose()
        trajectory = build_trajectory(mapping, orientation, current_position)
        runner = HybridRunner(
            robot=robot,
            reader=reader,
            trajectory=trajectory,
            config=parse_runner_config(mapping),
        )
        final_state = runner.run()
        if final_state != "DONE":
            logging.getLogger(__name__).error("Controller ended in %s: %s", final_state, runner.error)
            return 2
        return 0
    except KeyboardInterrupt:
        logging.getLogger(__name__).warning("Force controller cancelled by operator")
        robot.close()
        reader.close()
        return 130
    except Exception:
        logging.getLogger(__name__).exception("Could not start force controller")
        robot.close()
        reader.close()
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
