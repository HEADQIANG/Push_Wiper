#!/usr/bin/env python3
"""Check pinned dependencies, upstream imports, and the requested compute device."""

import argparse
import importlib.metadata
import json
import platform
import sys
from pathlib import Path

from verify_upstream import verify


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cpu")
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    if sys.version_info[:2] != (3, 10):
        raise RuntimeError("This environment is locked for Python 3.10")
    import torch
    import torchvision
    from diffusers import DDIMScheduler
    from diffusion_policy.model.common.lr_scheduler import get_scheduler
    from diffusion_policy.model.vision.model_getter import get_resnet
    from diffusion_policy.workspace.train_diffusion_unet_image_workspace import (
        TrainDiffusionUnetImageWorkspace,
    )

    expected = {
        "torch": "2.5.1", "torchvision": "0.20.1", "diffusers": "0.11.1",
        "huggingface-hub": "0.25.2", "numpy": "1.26.4", "zarr": "2.18.3",
        "numcodecs": "0.12.1",
    }
    versions = {name: importlib.metadata.version(name) for name in expected}
    for name, version in expected.items():
        if versions[name].split("+")[0] != version:
            raise RuntimeError(f"{name}: expected {version}, installed {versions[name]}")
    scheduler = DDIMScheduler(num_train_timesteps=100, prediction_type="sample")
    scheduler.set_timesteps(10)
    if len(scheduler.timesteps) != 10:
        raise RuntimeError("DDIM inference schedule did not produce 10 steps")
    model = get_resnet("resnet18", weights=None)
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-4)
    lr_scheduler = get_scheduler("cosine", optimizer, num_warmup_steps=500, num_training_steps=9000)
    if lr_scheduler is None or TrainDiffusionUnetImageWorkspace is None:
        raise RuntimeError("Official training workspace import failed")

    gpu = None
    if args.device == "cuda":
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA was requested but is unavailable; check the NVIDIA driver and CUDA wheel")
        if torch.version.cuda != "12.1":
            raise RuntimeError(f"Expected CUDA 12.1 wheel, installed CUDA {torch.version.cuda}")
        device = torch.device("cuda:0")
        value = torch.ones((32, 32), device=device) @ torch.ones((32, 32), device=device)
        torch.cuda.synchronize(device)
        if not torch.isfinite(value).all().item():
            raise RuntimeError("CUDA matrix multiplication returned non-finite values")
        props = torch.cuda.get_device_properties(device)
        gpu = {"name": props.name, "total_memory_bytes": props.total_memory,
               "compute_capability": list(torch.cuda.get_device_capability(device))}
    report = {
        "python": platform.python_version(), "executable": sys.executable,
        "platform": platform.platform(), "packages": versions,
        "requested_device": args.device, "cuda_available": torch.cuda.is_available(),
        "torch_cuda_version": torch.version.cuda, "gpu": gpu,
        "upstream": verify(Path(__file__).resolve().parents[1]),
        "checks": ["official_workspace_import", "resnet18_weights_none",
                   "ddim_sample_100_train_10_inference", "official_cosine_lr_scheduler"],
    }
    rendered = json.dumps(report, indent=2)
    print(rendered)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(rendered + "\n")


if __name__ == "__main__":
    main()
