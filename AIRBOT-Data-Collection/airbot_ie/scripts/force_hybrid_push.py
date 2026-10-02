"""Run the standalone AIRBOT Play + force-position controller."""

from __future__ import annotations

import argparse
import csv
import json
import logging
import time
from pathlib import Path

import numpy as np

from airbot_ie.force_control.core import (
    DragYSweepTrajectory,
    NpzPlanarTrajectory,
    quaternion_to_matrix,
)
from airbot_ie.force_control.hardware import (
    AirbotPlayAdapter,
    collect_bias,
    make_force_reader,
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
    slave_id = int(sensor.get("slave_id", 1))
    baudrate = int(sensor.get("baudrate", 115200))
    if not 1 <= slave_id <= 247:
        raise ValueError("sensor.slave_id must be in [1, 247]")
    if baudrate <= 0:
        raise ValueError("sensor.baudrate must be positive")
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
        replay_raw = bool(
            trajectory.get(
                "replay_raw", trajectory.get("replay_full_orientation", False)
            )
        )
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
            tool_normal_axis=str(trajectory.get("tool_normal_axis", "z")),
            replay_full_orientation=replay_raw,
        )
        height_source = (
            f"surface_z_m={surface_z}"
            if surface_z is not None
            else "surface_z_m=NPZ work_height_m or bounded current-pose approach"
        )
        print(
            f"配置和 NPZ 轨迹有效: {config_path}\n"
            f"  npz={npz_path}\n"
            f"  move_to_capture_pose={bool(trajectory.get('move_to_capture_pose', False))}\n"
            f"  staged_move_to_trajectory={bool(trajectory.get('staged_move_to_trajectory', False))}\n"
            f"  high_z_m={trajectory.get('high_z_m', 'capture_reference_pose.z')}\n"
            f"  replay_raw={replay_raw}\n"
            f"  tool_normal_axis={str(trajectory.get('tool_normal_axis', 'z'))}\n"
            f"  rezero_after_preposition={bool(sensor.get('rezero_after_preposition', False))}\n"
            f"  {height_source}"
        )
        return
    if mode in {"xy_json", "json"}:
        json_path = trajectory.get("json") or trajectory.get("path")
        if not json_path or not Path(json_path).exists():
            raise ValueError(f"trajectory JSON does not exist: {json_path}")
        duration = trajectory.get("duration_s")
        loaded = NpzPlanarTrajectory.from_xy_json(
            json_path,
            trajectory.get("capture_reference_pose"),
            None if duration is None else float(duration),
            None
            if trajectory.get("surface_z_m") is None
            else float(trajectory["surface_z_m"]),
            align_tool_z=bool(trajectory.get("align_tool_z", True)),
            tool_normal_axis=str(trajectory.get("tool_normal_axis", "z")),
        )
        print(
            f"配置和 XY JSON 轨迹有效: {config_path}\n"
            f"  json={json_path}\n"
            f"  duration_s={loaded.duration_s:.3f}\n"
            f"  replay_yaw=false\n"
            f"  surface_z_m={trajectory.get('surface_z_m', 'auto from current pose')}"
        )
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
            f"align_tool_z={bool(trajectory.get('align_tool_z', True))}, "
            f"tool_normal_axis={str(trajectory.get('tool_normal_axis', 'z'))}"
        )
        return
    raise ValueError(f"Unsupported trajectory.mode: {mode}")


def build_trajectory(mapping: dict, current_orientation, current_position=None):
    trajectory_cfg = mapping.get("trajectory", {})
    mode = trajectory_cfg.get("mode", "fixed_xy")
    if mode == "fixed_xy":
        return build_fixed_xy_trajectory(mapping, current_orientation)
    if mode == "npz":
        replay_raw = bool(
            trajectory_cfg.get(
                "replay_raw", trajectory_cfg.get("replay_full_orientation", False)
            )
        )
        surface_z = trajectory_cfg.get("surface_z_m")
        return NpzPlanarTrajectory.from_npz(
            trajectory_cfg["npz"],
            float(trajectory_cfg.get("duration_s", 6.0)),
            None if surface_z is None else float(surface_z),
            replay_yaw=bool(trajectory_cfg.get("replay_yaw", True)),
            align_tool_z=bool(trajectory_cfg.get("align_tool_z", True)),
            tool_normal_axis=str(trajectory_cfg.get("tool_normal_axis", "z")),
            replay_full_orientation=replay_raw,
        )
    if mode in {"xy_json", "json"}:
        json_path = trajectory_cfg.get("json") or trajectory_cfg.get("path")
        if not json_path:
            raise ValueError("trajectory.json is required for xy_json mode")
        capture_reference_pose = trajectory_cfg.get("capture_reference_pose")
        surface_z = trajectory_cfg.get("surface_z_m")
        return NpzPlanarTrajectory.from_xy_json(
            json_path,
            capture_reference_pose,
            None
            if trajectory_cfg.get("duration_s") is None
            else float(trajectory_cfg["duration_s"]),
            None if surface_z is None else float(surface_z),
            align_tool_z=bool(trajectory_cfg.get("align_tool_z", True)),
            tool_normal_axis=str(trajectory_cfg.get("tool_normal_axis", "z")),
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
            tool_normal_axis=str(trajectory_cfg.get("tool_normal_axis", "z")),
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
    """Display live end-pose XYZ and TCP/base axes while dragging the arm."""

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
        figure = plt.figure(figsize=(14, 8))
        grid = figure.add_gridspec(3, 2, width_ratios=(1.4, 1.0))
        axes = [figure.add_subplot(grid[index, 0]) for index in range(3)]
        orientation_axis = figure.add_subplot(grid[:, 1], projection="3d")
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
        orientations: list[np.ndarray] = []
        if csv_path:
            output = Path(csv_path)
            output.parent.mkdir(parents=True, exist_ok=True)
    except BaseException:
        robot.stop_gravity_comp()
        raise

    try:
        while plt.fignum_exists(figure.number):
            position, orientation = robot.read_pose()
            rotation = quaternion_to_matrix(orientation)
            times.append(time.monotonic() - started)
            positions.append(position.copy())
            orientations.append(rotation.copy())
            values_mm = np.asarray(positions) * 1000.0
            for index, line in enumerate(lines):
                line.set_data(times, values_mm[:, index])
                axes[index].relim()
                axes[index].autoscale_view()
            orientation_axis.clear()
            orientation_axis.set_title("TCP 与基座坐标轴（当前末端位置）")
            orientation_axis.set_xlabel("Base X")
            orientation_axis.set_ylabel("Base Y")
            orientation_axis.set_zlabel("Base Z")
            orientation_axis.set_xlim(position[0] - 0.12, position[0] + 0.12)
            orientation_axis.set_ylim(position[1] - 0.12, position[1] + 0.12)
            orientation_axis.set_zlim(position[2] - 0.12, position[2] + 0.12)
            orientation_axis.set_box_aspect((1, 1, 1))
            axis_colors = ("tab:red", "tab:green", "tab:blue")
            axis_labels = ("X", "Y", "Z")
            for index, (color, label) in enumerate(zip(axis_colors, axis_labels)):
                direction = rotation[:, index]
                orientation_axis.quiver(
                    *position,
                    *direction,
                    length=0.08,
                    normalize=True,
                    color=color,
                    linewidth=2.5,
                )
                orientation_axis.text(
                    *(position + direction * 0.09),
                    f"TCP {label}",
                    color=color,
                )
                base_direction = np.eye(3)[:, index]
                orientation_axis.quiver(
                    *position,
                    *base_direction,
                    length=0.05,
                    normalize=True,
                    color=color,
                    alpha=0.25,
                    arrow_length_ratio=0.2,
                )
            orientation_axis.text(
                *position,
                "基座 XYZ 原点（平移到 TCP 位置）",
                color="black",
                fontsize=8,
            )
            delta_mm = values_mm[-1] - values_mm[0]
            axis_text = " ".join(
                f"TCP {label}=({direction[0]:+.2f},{direction[1]:+.2f},{direction[2]:+.2f})"
                for label, direction in zip(axis_labels, rotation.T)
            )
            figure.suptitle(
                "AIRBOT 末端位姿拖拽预览（关闭窗口或 Ctrl+C 结束）\n"
                f"当前 ΔX={delta_mm[0]:+.1f} mm, "
                f"ΔY={delta_mm[1]:+.1f} mm, ΔZ={delta_mm[2]:+.1f} mm"
                f"\n{axis_text}"
            )
            figure.canvas.draw_idle()
            plt.pause(1.0 / sample_hz)
    finally:
        robot.stop_gravity_comp()
        if output and positions:
            with output.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(
                    (
                        "t_s", "x_m", "y_m", "z_m",
                        "tcp_x_bx", "tcp_x_by", "tcp_x_bz",
                        "tcp_y_bx", "tcp_y_by", "tcp_y_bz",
                        "tcp_z_bx", "tcp_z_by", "tcp_z_bz",
                    )
                )
                writer.writerows(
                    [
                        time_s,
                        position[0], position[1], position[2],
                        *rotation[:, 0], *rotation[:, 1], *rotation[:, 2],
                    ]
                    for time_s, position, rotation in zip(times, positions, orientations)
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
        help="Validate configuration and optional NPZ/XY JSON without connecting hardware",
    )
    check_group.add_argument(
        "--sensor-check",
        action="store_true",
        help="Check force sensor communication and print a short unloaded bias; do not move AIRBOT",
    )
    check_group.add_argument(
        "--sensor-read",
        action="store_true",
        help="Read force sensor without sending the hardware-zero command",
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
    if args.sensor_check or args.sensor_read:
        reader = make_force_reader(sensor_cfg)
        try:
            if args.sensor_check:
                reader.zero()
                time.sleep(float(sensor_cfg.get("zero_settle_s", 0.1)))
            bias = collect_bias(
                reader,
                duration_s=float(mapping.get("force", {}).get("bias_duration_s", 2.0)),
                poll_interval_s=float(sensor_cfg.get("poll_interval_s", 0.02)),
            )
            print(
                f"{sensor_cfg.get('driver', 'kunwei')} communication OK; unloaded mean "
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
    reader = make_force_reader(sensor_cfg)
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
