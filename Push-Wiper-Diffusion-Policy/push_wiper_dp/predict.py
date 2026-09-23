"""Load an EMA checkpoint and generate a complete 16-point pushing segment."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import dill
import hydra
import numpy as np
from omegaconf import OmegaConf
import torch

from push_wiper_dp.dataset import PushWiperDataset, preprocess_observation
from push_wiper_dp.evaluation import evaluate_policy, isolated_seed, predict_observation


class PushWiperPredictor:
    """Inference API: raw binary mask + reference xyzw pose -> float32 [16, 3]."""

    def __init__(self, checkpoint: str | Path, device: str = "cpu") -> None:
        self.checkpoint = Path(checkpoint).expanduser().resolve()
        self.device = torch.device(device)
        if self.device.type not in {"cpu", "cuda"}:
            raise ValueError("Only cpu and cuda devices are supported")
        if self.device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA is unavailable; install the CUDA environment or use --device cpu")
        # Workspace checkpoints contain dill metadata; use checkpoints from this project.
        payload = torch.load(
            self.checkpoint, map_location="cpu", pickle_module=dill, weights_only=False,
        )
        if not isinstance(payload, dict) or "cfg" not in payload or "state_dicts" not in payload:
            raise ValueError("Expected a Diffusion Policy BaseWorkspace checkpoint")
        self.cfg = payload["cfg"]
        if not OmegaConf.is_config(self.cfg):
            self.cfg = OmegaConf.create(self.cfg)
        state_dicts = payload["state_dicts"]
        if "ema_model" in state_dicts:
            self.weights_source = "ema_model"
        elif "model" in state_dicts:
            self.weights_source = "model"
        else:
            raise ValueError("Checkpoint contains neither EMA nor model weights")
        weights = state_dicts[self.weights_source]
        # Training checkpoints also contain Adam moments and a second network.
        # Release those before constructing the inference network on a CPU host.
        del state_dicts, payload
        if (
            list(self.cfg.policy.shape_meta.action.shape) != [3]
            or (self.cfg.policy.horizon, self.cfg.policy.n_obs_steps, self.cfg.policy.n_action_steps)
            != (16, 1, 16)
        ):
            raise ValueError("Checkpoint must describe one observation and a complete [16, 3] segment")
        # Construction initializes random weights before replacing them; isolate it too.
        with isolated_seed(0, torch.device("cpu")):
            self.policy = hydra.utils.instantiate(self.cfg.policy)
        self.policy.load_state_dict(weights, strict=True)
        del weights
        self.policy.to(device=self.device, dtype=torch.float32).eval()
        shape = list(self.cfg.policy.shape_meta.obs.mask.shape)
        if len(shape) != 3 or shape[0] != 3:
            raise ValueError(f"Expected a three-channel mask shape in checkpoint, got {shape}")
        self.image_size = (int(shape[1]), int(shape[2]))

    def predict(
        self, mask: np.ndarray, capture_reference_pose: np.ndarray, seed: int = 42,
    ) -> np.ndarray:
        observation = preprocess_observation(mask, capture_reference_pose, image_size=self.image_size)
        return predict_observation(self.policy, observation, seed)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--split", choices=("train", "validation"), default="validation")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--seed", type=int, default=1042)
    parser.add_argument("--threads", type=int, default=4)
    args = parser.parse_args()
    if args.threads <= 0:
        parser.error("--threads must be positive")
    torch.set_num_threads(args.threads)
    predictor = PushWiperPredictor(args.checkpoint, device=args.device)
    dataset = PushWiperDataset(args.data_root, split=args.split, image_size=predictor.image_size)
    output_dir = args.output_dir or args.checkpoint.resolve().parent.parent / "predictions" / args.split
    report = evaluate_policy(predictor.policy, dataset, output_dir, seed=args.seed)
    print(json.dumps({
        "output_dir": str(output_dir.resolve()),
        "weights_source": predictor.weights_source,
        "count": report["count"],
        **report["aggregate"],
    }, indent=2))


if __name__ == "__main__":
    main()
