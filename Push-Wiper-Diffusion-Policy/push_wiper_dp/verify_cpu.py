"""Full-size CPU acceptance: updates, EMA inference and exact epoch-boundary resume."""

import argparse
import gc
import hashlib
import json
from pathlib import Path
import resource

import dill
import numpy as np
import torch

from .config import load_config
from .evaluation import predict_observation
from .predict import PushWiperPredictor
from .workspace import PushWiperWorkspace


def _digest(value):
    digest = hashlib.sha256()

    def update(item):
        if isinstance(item, torch.Tensor):
            update(item.detach().cpu().numpy())
        elif isinstance(item, np.ndarray):
            digest.update(str((item.dtype.str, item.shape)).encode())
            digest.update(item.tobytes())
        elif isinstance(item, dict):
            for key in sorted(item, key=str):
                update(key)
                update(item[key])
        elif isinstance(item, (tuple, list)):
            digest.update(type(item).__name__.encode())
            for element in item:
                update(element)
        else:
            digest.update(repr(item).encode())
        digest.update(b"\0")

    update(value)
    return digest.hexdigest()


def checkpoint_summary(path):
    with Path(path).open("rb") as stream:
        payload = torch.load(stream, map_location="cpu", pickle_module=dill, weights_only=False)
    states = payload["state_dicts"]
    result = {key: _digest(states[key]) for key in ("model", "ema_model", "optimizer", "lr_scheduler")}
    for key in ("epoch", "global_step", "best_val_loss", "ema_state"):
        result[key] = dill.loads(payload["pickles"][key])
    result["rng_state"] = _digest(dill.loads(payload["pickles"]["rng_state"]))
    return result


def _witness(model):
    # The first nonempty parameter is the ResNet input convolution.
    return next(p for p in model.parameters() if p.numel()).detach().cpu().flatten()[:1024].clone()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/cpu_acceptance"))
    args = parser.parse_args()
    root = args.output_dir.resolve()
    if root.exists() and any(root.iterdir()):
        parser.error("output-dir must be new or empty; verification never overwrites existing runs")
    root.mkdir(parents=True, exist_ok=True)
    common = ["runtime=cpu_smoke"]
    if args.data_root:
        common.append(f"task.dataset.root={args.data_root.resolve()}")
    full_dir, resumed_dir = root / "uninterrupted", root / "resumed"

    print("CPU acceptance 1/3: full network, four optimizer/EMA updates", flush=True)
    workspace = PushWiperWorkspace(load_config(common + [f"run.output_dir={full_dir}"]))
    workspace.setup()
    initial = _witness(workspace.model)
    workspace.run()
    optimizer_updated = not torch.equal(initial, _witness(workspace.model))
    ema_updated = not torch.equal(initial, _witness(workspace.ema_model))
    if not optimizer_updated or not ema_updated:
        raise AssertionError("Model and EMA weights must both change during the four-update smoke run")
    observation = workspace.val_dataset[0]["obs"]
    expected = predict_observation(workspace.ema_model, observation, seed=31415)
    with np.load(workspace.val_dataset.root / workspace.val_dataset.records[0]["sample"], allow_pickle=False) as sample:
        mask = sample["mask"].copy()
    pose = observation["capture_reference_pose"][0].numpy().copy()
    parameter_count = sum(p.numel() for p in workspace.model.parameters())
    del workspace
    gc.collect()
    predictor = PushWiperPredictor(full_dir / "checkpoints/latest.ckpt", device="cpu")
    actual = predictor.predict(mask, pose, seed=31415)
    np.testing.assert_array_equal(actual, expected)
    if actual.shape != (16, 3) or not np.isfinite(actual).all():
        raise AssertionError("Inference must return all 16 finite actions")
    del predictor
    gc.collect()

    print("CPU acceptance 2/3: save after epoch one", flush=True)
    workspace = PushWiperWorkspace(load_config(common + [
        f"run.output_dir={resumed_dir}", "training.stop_after_epochs=1",
    ]))
    workspace.run()
    saved_step, saved_lr = workspace.global_step, workspace.lr_scheduler.get_last_lr()[0]
    del workspace
    gc.collect()

    print("CPU acceptance 3/3: restore and match uninterrupted training", flush=True)
    workspace = PushWiperWorkspace(load_config(common + [
        f"run.output_dir={resumed_dir}",
        f"training.resume={resumed_dir / 'checkpoints/latest.ckpt'}",
    ]))
    workspace.setup()
    assert workspace.epoch == 1 and workspace.global_step == saved_step == 2
    assert workspace.ema_helper.optimization_step == saved_step
    assert workspace.lr_scheduler.get_last_lr()[0] == saved_lr
    workspace.run()
    del workspace
    gc.collect()
    uninterrupted = checkpoint_summary(full_dir / "checkpoints/latest.ckpt")
    resumed = checkpoint_summary(resumed_dir / "checkpoints/latest.ckpt")
    if uninterrupted != resumed:
        mismatch = [key for key in uninterrupted if uninterrupted[key] != resumed[key]]
        raise AssertionError(f"Epoch-boundary resume differs from uninterrupted training: {mismatch}")
    report = {
        "passed": True, "device": "cpu", "parameters": parameter_count,
        "optimizer_updated": optimizer_updated, "ema_updated": ema_updated,
        "inference_shape": list(actual.shape), "reload_prediction_exact": True,
        "resume_states_exact": True, "global_step": resumed["global_step"],
        "ema_updates": resumed["ema_state"]["optimization_step"],
        "state_digests": {key: resumed[key] for key in ("model", "ema_model", "optimizer", "lr_scheduler")},
        "gpu_verified": False,
        "peak_process_rss_mb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024,
    }
    (root / "verification_report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
