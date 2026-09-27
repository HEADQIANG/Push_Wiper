import cv2
import numpy as np
import os
import yaml
from pydantic import BaseModel
from typing import List, Tuple, Union
from pathlib import Path


class CalibrationConfig(BaseModel):
    """Configuration for camera calibration from video files."""

    video_dir: Union[str, Path] = Path(__file__).parent / "calibration/intrinsic"
    """Directory containing calibration video files."""
    video_paths: List[str] = []
    """The path to the calibration video file."""
    board_size: Tuple[int, int]
    """Number of inner corners per chessboard row and column (cols, rows)."""
    square_size: float
    """Size of a square in your defined unit (point, millimeter,etc.)."""
    max_frames: int = 20
    """Maximum number of frames to use for calibration."""
    frame_interval: int = 1
    """Interval between frames to sample."""
    use_fisheye: bool = False
    """Whether to use fisheye model for calibration."""
    output_dir: Path = Path(__file__).parent / "outputs/intrinsic"
    """Directory to save calibration results."""

    def model_post_init(self, context):
        if self.video_dir and not self.video_paths:
            p = Path(self.video_dir)
            self.video_paths = [str(f) for f in p.glob("*.mp4")]
        elif not self.video_paths:
            raise ValueError("Either video_dir or video_paths must be provided.")


def compute_fov(intrinsics, image_size, in_degrees=True):
    """
    根据相机内参和图像尺寸计算水平和垂直视场角（FOV）。

    参数:
        intrinsics (np.ndarray or list): 3x3 相机内参矩阵 K
        image_size (tuple): (width, height)，图像尺寸（像素）
        in_degrees (bool): 若为 True，返回角度；否则返回弧度

    返回:
        dict: {'h': 水平FOV, 'v': 垂直FOV, 'd': 对角线FOV} （单位由 in_degrees 决定）
    """
    K = np.array(intrinsics)
    if K.shape != (3, 3):
        raise ValueError("intrinsics 必须是 3x3 矩阵")

    fx = K[0, 0]
    fy = K[1, 1]
    width, height = image_size

    # 避免除零
    if fx <= 0 or fy <= 0:
        raise ValueError("焦距 fx 和 fy 必须为正数")

    fov_x = 2 * np.arctan(width / (2 * fx))
    fov_y = 2 * np.arctan(height / (2 * fy))
    fov_d = 2 * np.arctan(np.sqrt(width**2 + height**2) / (2 * np.sqrt(fx * fy)))

    if in_degrees:
        fov_x = np.degrees(fov_x)
        fov_y = np.degrees(fov_y)
        fov_d = np.degrees(fov_d)
    return {"h": float(fov_x), "v": float(fov_y), "d": float(fov_d)}


def calibrate_from_video(
    video_path,
    board_size,
    square_size,
    max_frames=20,
    frame_interval=10,
    use_fisheye=False,
    output_dir="calibration_output",
):
    """从视频中自动提取棋盘格帧并进行相机标定。
    参数:
        video_path: 标定视频路径。
        board_size: 棋盘格内角点数量 (cols, rows)。
        square_size: 棋盘格方块边长（单位任意，但需与实际测量一致）。
        max_frames: 用于标定的最大帧数。
        frame_interval: 采样帧间隔。
        use_fisheye: 是否使用鱼眼模型进行标定。
        output_dir: 标定结果保存目录。
    """
    os.makedirs(output_dir, exist_ok=True)
    undist_dir = os.path.join(output_dir, "undistorted")
    os.makedirs(undist_dir, exist_ok=True)

    # 棋盘格世界坐标
    objp = np.zeros((1, board_size[0] * board_size[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[0 : board_size[0], 0 : board_size[1]].T.reshape(-1, 2)
    objp *= square_size

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise IOError(f"Can not open video file: {video_path}")

    candidate_frames = []
    frame_idx = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if frame_idx % frame_interval != 0:
            frame_idx += 1
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        found, corners = cv2.findChessboardCorners(
            gray,
            board_size,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )

        if found:
            corners_refined = cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
            )

            # 质量指标：亚像素修正幅度
            quality = np.mean(np.linalg.norm(corners_refined - corners, axis=2))

            candidate_frames.append(
                {
                    "frame": frame.copy(),
                    "gray": gray,
                    "corners": corners_refined,
                    "quality": quality,
                    "index": frame_idx,
                }
            )

        frame_idx += 1

    cap.release()

    if len(candidate_frames) < max_frames:
        # raise RuntimeError(
        print(
            f"No enough valid frames found for calibration. Found: {len(candidate_frames)}, Required: {max_frames}"
        )
        return

    candidate_frames.sort(key=lambda x: x["quality"])
    selected = candidate_frames[:max_frames]

    objpoints = []
    imgpoints = []

    for _ in selected:
        objpoints.append(objp)

    for item in selected:
        imgpoints.append(item["corners"].reshape(1, -1, 2))

    image_size = selected[0]["gray"].shape[::-1]

    if use_fisheye:
        K = np.zeros((3, 3))
        D = np.zeros((4, 1))
        rvecs = []
        tvecs = []

        flags = (
            cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
            + cv2.fisheye.CALIB_CHECK_COND
            + cv2.fisheye.CALIB_FIX_SKEW
        )

        rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
            objpoints,
            imgpoints,
            image_size,
            K,
            D,
            rvecs,
            tvecs,
            flags=flags,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6),
        )
    else:
        ret, K, D, rvecs, tvecs = cv2.calibrateCamera(
            [op.reshape(-1, 3) for op in objpoints],
            [ip.reshape(-1, 2) for ip in imgpoints],
            image_size,
            None,
            None,
        )
        rms = ret

    calib_data = {
        "model": "fisheye" if use_fisheye else "pinhole",
        "camera_matrix": K.tolist(),
        "distortion_coefficients": D.tolist(),
        "rvecs": [rvec.tolist() for rvec in rvecs],
        "tvecs": [tvec.tolist() for tvec in tvecs],
        "image_width": image_size[0],
        "image_height": image_size[1],
        "board_size": list(board_size),
        "square_size": square_size,
        "rms_reprojection_error": float(rms),
        "fov": compute_fov(K, image_size),
    }

    yaml_path = os.path.join(output_dir, "calibration.yaml")
    with open(yaml_path, "w") as f:
        yaml.dump(calib_data, f)

    for i, item in enumerate(selected):
        vis = item["frame"]
        cv2.drawChessboardCorners(vis, board_size, item["corners"], True)

        if use_fisheye:
            undistorted = cv2.fisheye.undistortImage(item["frame"], K, D, Knew=K)
        else:
            undistorted = cv2.undistort(item["frame"], K, D)

        cv2.imwrite(
            os.path.join(undist_dir, f"frame_{i:02d}_undistorted.jpg"), undistorted
        )

    print("标定完成：")
    print(f" - 模型类型: {'Fisheye' if use_fisheye else 'Pinhole'}")
    print(f" - 使用帧数: {len(selected)}")
    print(f" - RMS 重投影误差: {rms:.6f}")
    print(f" - 结果保存至: {yaml_path}")


if __name__ == "__main__":
    from pydantic_settings import CliApp

    app = CliApp()
    config = app.run(CalibrationConfig)

    cfg_dict = config.model_dump(exclude={"video_paths", "video_dir", "output_dir"})

    for video_path in config.video_paths:
        print(f"Processing video: {video_path}")
        calibrate_from_video(
            video_path=video_path,
            **cfg_dict,
            output_dir=os.path.join(
                config.output_dir,
                Path(video_path).stem,
            ),
        )
