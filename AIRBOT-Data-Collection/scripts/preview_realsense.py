#!/usr/bin/env python3
"""连续预览 RealSense 彩色视频，可选同时显示深度图。"""

import argparse
import os
import sys
import time


def positive_int(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("必须是大于 0 的整数")
    return number


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", help="相机序列号；默认使用第一台 RealSense")
    parser.add_argument(
        "--width", type=positive_int, default=640, help="宽度，默认 640"
    )
    parser.add_argument(
        "--height", type=positive_int, default=480, help="高度，默认 480"
    )
    parser.add_argument("--fps", type=positive_int, default=30, help="帧率，默认 30")
    parser.add_argument(
        "--depth", action="store_true", help="并排显示彩色图和深度伪彩色图"
    )
    parser.add_argument(
        "--headless", action="store_true", help="只读取视频并打印帧率，不开窗口"
    )
    parser.add_argument(
        "--frames", type=positive_int, help="读取指定帧数后退出；默认持续运行"
    )
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        import cv2
        import numpy as np
        import pyrealsense2 as rs
    except ImportError as exc:
        print(
            f"缺少相机依赖：{exc}\n"
            "请先在仓库根目录执行 source install/activate_airdc.sh。\n"
            "安装依赖的方法见 docs/setup/realsense_preview.md。",
            file=sys.stderr,
        )
        return 1

    if (
        not args.headless
        and sys.platform.startswith("linux")
        and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    ):
        print("没有图形桌面，请在本机桌面终端运行，或加 --headless。", file=sys.stderr)
        return 1

    pipeline = None
    started = False
    window_created = False
    window_name = "RealSense Preview - Q / Esc to quit"
    try:
        context = rs.context()
        devices = list(context.query_devices())
        if args.serial:
            devices = [
                device
                for device in devices
                if device.get_info(rs.camera_info.serial_number) == args.serial
            ]
        if not devices:
            target = f"（序列号 {args.serial}）" if args.serial else ""
            print(
                f"未找到 RealSense 相机{target}。请检查 USB 连接和设备权限。",
                file=sys.stderr,
            )
            return 1

        device = devices[0]
        serial = device.get_info(rs.camera_info.serial_number)
        print(
            f"相机：{device.get_info(rs.camera_info.name)}，序列号：{serial}",
            flush=True,
        )
        if device.supports(rs.camera_info.usb_type_descriptor):
            usb_type = device.get_info(rs.camera_info.usb_type_descriptor)
            print(f"USB 连接：{usb_type}", flush=True)
            if usb_type.startswith("2"):
                print(
                    "USB 2.0 带宽有限；双流或高分辨率卡顿时可用 --fps 15，或改接 USB 3.0。"
                )

        pipeline = rs.pipeline(context)
        config = rs.config()
        config.enable_device(serial)
        config.enable_stream(
            rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps
        )
        if args.depth:
            config.enable_stream(
                rs.stream.depth, args.width, args.height, rs.format.z16, args.fps
            )
        profile = pipeline.start(config)
        started = True
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        print(
            f"彩色视频：{color_profile.width()}×{color_profile.height()} "
            f"@ {color_profile.fps()} FPS",
            flush=True,
        )
        colorizer = rs.colorizer() if args.depth and not args.headless else None
        if not args.headless:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            window_created = True
            print(
                "在视频窗口按 q / Esc 或关闭窗口退出；终端也可按 Ctrl+C。", flush=True
            )
        else:
            print("无窗口模式，按 Ctrl+C 退出。", flush=True)

        total_frames = 0
        interval_frames = 0
        last_report = time.perf_counter()
        measured_fps = 0.0
        while args.frames is None or total_frames < args.frames:
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame() if args.depth else None
            if not color_frame or (args.depth and not depth_frame):
                continue

            total_frames += 1
            interval_frames += 1
            now = time.perf_counter()
            elapsed = now - last_report
            if elapsed >= 2.0:
                measured_fps = interval_frames / elapsed
                print(
                    f"已接收 {total_frames} 帧，实际帧率 {measured_fps:.1f} FPS",
                    flush=True,
                )
                interval_frames = 0
                last_report = now

            if args.headless:
                continue

            # 拷贝帧数据，避免在 SDK 管理的缓冲区上绘制文字。
            color_image = np.asanyarray(color_frame.get_data()).copy()
            cv2.putText(
                color_image,
                f"Color | {measured_fps:.1f} FPS",
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            if colorizer is not None:
                # rs.colorizer 输出 RGB，OpenCV 窗口需要 BGR。
                depth_rgb = np.asanyarray(colorizer.colorize(depth_frame).get_data())
                depth_image = cv2.cvtColor(depth_rgb, cv2.COLOR_RGB2BGR)
                cv2.putText(
                    depth_image,
                    "Depth",
                    (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2,
                )
                color_image = np.hstack((color_image, depth_image))

            cv2.imshow(window_name, color_image)
            if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q"), 27):
                break
            if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                break

        print(f"预览结束，共接收 {total_frames} 帧。", flush=True)
        return 0
    except KeyboardInterrupt:
        print("\n已停止预览。", flush=True)
        return 0
    except (RuntimeError, cv2.error) as exc:
        print(
            f"预览失败：{exc}\n"
            "请检查相机是否被其他程序占用、USB 连接和权限，以及分辨率/帧率是否受支持。\n"
            "USB 2.0 双流可尝试 --depth --fps 15；窗口问题见 docs/setup/realsense_preview.md。",
            file=sys.stderr,
        )
        return 1
    finally:
        try:
            if started:
                pipeline.stop()
        finally:
            if window_created:
                cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
