"""Deterministic offline trajectory evaluation; these are not cleaning scores."""

from __future__ import annotations

from contextlib import contextmanager
import json
from pathlib import Path
import random
import re
from typing import Any, Iterator

import numpy as np
import torch


@contextmanager
def isolated_seed(seed: int, device: torch.device) -> Iterator[None]:
    """Seed inference without advancing the caller's Python, NumPy or torch RNG."""
    python_state = random.getstate()
    numpy_state = np.random.get_state()
    cuda_devices = []
    if device.type == "cuda":
        cuda_devices = [device.index if device.index is not None else torch.cuda.current_device()]
    try:
        with torch.random.fork_rng(devices=cuda_devices):
            random.seed(seed)
            np.random.seed(seed % (2**32))
            # torch.manual_seed also seeds other GPUs, whose states are not forked.
            torch.random.default_generator.manual_seed(seed)
            for index in cuda_devices:
                torch.cuda.default_generators[index].manual_seed(seed)
            yield
    finally:
        random.setstate(python_state)
        np.random.set_state(numpy_state)


def policy_device(policy: torch.nn.Module) -> torch.device:
    return next(policy.parameters(), torch.empty(0)).device


def predict_observation(
    policy: torch.nn.Module, observation: dict[str, torch.Tensor], seed: int
) -> np.ndarray:
    """Predict one segment from observation tensors without using target actions."""
    device = policy_device(policy)
    batched = {
        key: value.unsqueeze(0).to(device=device, dtype=torch.float32)
        for key, value in observation.items()
    }
    was_training = policy.training
    try:
        policy.eval()
        with isolated_seed(seed, device), torch.inference_mode():
            action = policy.predict_action(batched)["action"]
            prediction = action.detach().cpu().numpy().astype(np.float32, copy=True)
    finally:
        policy.train(was_training)
    if prediction.shape != (1, 16, 3):
        raise ValueError(f"Expected full-segment action shape (1, 16, 3), got {prediction.shape}")
    if not np.isfinite(prediction).all():
        raise ValueError("Policy produced non-finite action values")
    return prediction[0]


def trajectory_metrics(prediction: np.ndarray, target: np.ndarray) -> dict[str, float]:
    """Compare metre/radian actions; continuity uses the unwrapped predicted yaw."""
    prediction = np.asarray(prediction, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    if (
        prediction.ndim != 2
        or prediction.shape[-1] != 3
        or prediction.shape[0] < 2
        or prediction.shape != target.shape
    ):
        raise ValueError("prediction and target must have the same shape [T >= 2, 3]")
    if not np.isfinite(prediction).all() or not np.isfinite(target).all():
        raise ValueError("prediction and target must contain only finite values")
    xy_error = np.linalg.norm(prediction[:, :2] - target[:, :2], axis=-1)
    yaw_delta = prediction[:, 2] - target[:, 2]
    circular_yaw_error = np.abs(np.arctan2(np.sin(yaw_delta), np.cos(yaw_delta)))
    return {
        "xy_ade_mm": float(xy_error.mean() * 1000.0),
        "xy_fde_mm": float(xy_error[-1] * 1000.0),
        "yaw_mae_deg": float(np.rad2deg(circular_yaw_error).mean()),
        "xy_max_step_mm": float(np.linalg.norm(np.diff(prediction[:, :2], axis=0), axis=-1).max() * 1000.0),
        "yaw_max_step_deg": float(np.rad2deg(np.abs(np.diff(prediction[:, 2]))).max()),
    }


def _plot_segment(
    observation: dict[str, torch.Tensor], prediction: np.ndarray,
    target: np.ndarray, path: Path, sample_id: str,
) -> None:
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    figure = Figure(figsize=(12, 3.7), layout="constrained")
    FigureCanvasAgg(figure)
    mask_axis, xy_axis, yaw_axis = figure.subplots(1, 3)
    mask = observation["mask"][0, 0].detach().cpu().numpy()
    mask_axis.imshow(mask, cmap="gray", vmin=0, vmax=1)
    mask_axis.set_title("Initial mask (dirty = 0)")
    mask_axis.set_axis_off()
    xy_axis.plot(target[:, 0], target[:, 1], "o-", label="Demonstration", markersize=3)
    xy_axis.plot(prediction[:, 0], prediction[:, 1], "o-", label="Prediction", markersize=3)
    xy_axis.set(xlabel="Base x (m)", ylabel="Base y (m)", title="Segment XY trajectory")
    xy_axis.set_aspect("equal", adjustable="datalim")
    xy_axis.legend(fontsize=8)
    xy_axis.grid(alpha=0.2)
    time = np.linspace(0, 1, len(target))
    yaw_axis.plot(time, np.rad2deg(target[:, 2]), "o-", label="Demonstration", markersize=3)
    yaw_axis.plot(time, np.rad2deg(prediction[:, 2]), "o-", label="Prediction", markersize=3)
    yaw_axis.set(xlabel="Normalized segment time", ylabel="Unwrapped delta yaw (deg)", title="Segment yaw")
    yaw_axis.legend(fontsize=8)
    yaw_axis.grid(alpha=0.2)
    figure.suptitle(sample_id, fontsize=8)
    figure.savefig(path, dpi=130)
    figure.clear()


def evaluate_policy(
    policy: torch.nn.Module, dataset: Any, output_dir: str | Path,
    seed: int = 1042, limit: int | None = None,
) -> dict[str, Any]:
    """Evaluate fixed dataset order, save full trajectories, per-sample metrics and plots.

    Error metrics are sample means. Aggregate continuity metrics are maxima across
    all samples. A separate deterministic seed (seed + index) is used per sample.
    The policy's training flag and caller RNG states are restored after prediction.
    """
    if limit is not None and limit <= 0:
        raise ValueError("limit must be positive")
    count = len(dataset) if limit is None else min(len(dataset), limit)
    if count == 0:
        raise ValueError("Cannot evaluate an empty dataset")
    output_dir = Path(output_dir)
    plot_dir = output_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    samples = []
    predictions, targets, sample_ids = [], [], []
    records = getattr(dataset, "records", None)
    for index in range(count):
        sample = dataset[index]
        prediction = predict_observation(policy, sample["obs"], seed + index)
        target = sample["action"].detach().cpu().numpy().astype(np.float32, copy=True)
        record = records[index] if records is not None else {}
        sample_id = str(record.get("sample", f"sample_{index:04d}"))
        filename = re.sub(r"[^a-zA-Z0-9_-]", "_", str(record.get("segment_id", f"sample_{index:04d}")))[:128]
        plot_path = plot_dir / f"{index:04d}_{filename}.png"
        metrics = trajectory_metrics(prediction, target)
        _plot_segment(sample["obs"], prediction, target, plot_path, sample_id)
        samples.append({
            "sample_id": sample_id,
            "task_id": record.get("task_id"),
            "segment_id": record.get("segment_id"),
            "seed": seed + index,
            "plot": str(plot_path.relative_to(output_dir)),
            **metrics,
        })
        predictions.append(prediction)
        targets.append(target)
        sample_ids.append(sample_id)
    aggregate = {}
    for key in metrics:
        values = [sample[key] for sample in samples]
        aggregate[key] = float(max(values) if "max_step" in key else np.mean(values))
    report = {
        "count": count,
        "seed": seed,
        "aggregate": aggregate,
        "aggregation": "sample means for errors; global maxima for continuity",
        "interpretation": "Offline trajectory metrics; not cleaning success rates.",
        "samples": samples,
    }
    np.savez_compressed(
        output_dir / "predictions.npz",
        predictions=np.stack(predictions),
        targets=np.stack(targets),
        sample_ids=np.asarray(sample_ids),
    )
    (output_dir / "metrics.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return report
