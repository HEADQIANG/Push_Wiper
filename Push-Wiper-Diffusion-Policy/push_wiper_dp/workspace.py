"""Offline adaptation of the official image workspace, with epoch-boundary resume."""

from contextlib import contextmanager
from pathlib import Path
import json
import math
import os
import random
import time

import dill
import hydra
import numpy as np
from omegaconf import OmegaConf
import torch
from torch.utils.data import DataLoader, Subset

from diffusion_policy.common.json_logger import JsonLogger
from diffusion_policy.common.pytorch_util import dict_apply, optimizer_to
from diffusion_policy.model.common.lr_scheduler import get_scheduler
from diffusion_policy.workspace.train_diffusion_unet_image_workspace import (
    TrainDiffusionUnetImageWorkspace,
)

from . import UPSTREAM_REVISION
from .evaluation import evaluate_policy


def _plain(value):
    return OmegaConf.to_container(value, resolve=True)


@contextmanager
def fixed_torch_seed(seed, device):
    """Validation must not consume the training diffusion RNG stream."""
    devices = [device.index or 0] if device.type == "cuda" else []
    with torch.random.fork_rng(devices=devices):
        torch.random.default_generator.manual_seed(seed)
        if devices:
            with torch.cuda.device(device):
                torch.cuda.manual_seed(seed)
        yield


def _limited(dataset, limit):
    if limit is None:
        return dataset
    if int(limit) < 1:
        raise ValueError("Dataset limits must be positive or null")
    return Subset(dataset, range(min(len(dataset), int(limit))))


class PushWiperWorkspace(TrainDiffusionUnetImageWorkspace):
    """Reuse official model/optimizer initialization; adapt the offline lifecycle."""

    include_keys = (
        "global_step", "epoch", "best_val_loss", "metadata", "training_contract",
        "ema_state", "rng_state",
    )

    def __init__(self, cfg, output_dir=None):
        cfg = OmegaConf.create(_plain(cfg))
        self._validate_config(cfg)
        torch.set_num_threads(int(cfg.training.threads))
        device = torch.device(cfg.training.device)
        if device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA is unavailable. Install the CUDA environment on the GPU host, "
                               "or use runtime=cpu_smoke for CPU verification.")
        super().__init__(cfg, output_dir=str(Path(output_dir or cfg.run.output_dir).resolve()))
        self.device = device
        self.best_val_loss = math.inf
        self.metadata = {}
        self.training_contract = {}
        self.ema_state = {}
        self.rng_state = {}
        self._prepared = False

    @staticmethod
    def _validate_config(cfg):
        if not cfg.training.use_ema:
            raise ValueError("Push-Wiper validation and inference require use_ema=true")
        if cfg.training.gradient_accumulate_every != 1:
            raise ValueError("This workspace supports gradient_accumulate_every=1")
        if (cfg.policy.horizon, cfg.policy.n_obs_steps, cfg.policy.n_action_steps) != (16, 1, 16):
            raise ValueError("A Push-Wiper sample requires horizon=16, n_obs_steps=1, n_action_steps=16")
        if cfg.policy.noise_scheduler.prediction_type != "sample":
            raise ValueError("Push-Wiper uses sample prediction, not epsilon prediction")
        if cfg.policy.noise_scheduler._target_ != "diffusers.schedulers.scheduling_ddim.DDIMScheduler":
            raise ValueError("Push-Wiper requires a DDIM scheduler")
        for key in ("num_epochs", "val_every", "sample_every", "checkpoint_every", "threads"):
            if int(cfg.training[key]) < 1:
                raise ValueError(f"training.{key} must be positive")
        if cfg.training.stop_after_epochs is not None and cfg.training.stop_after_epochs < 1:
            raise ValueError("stop_after_epochs must be positive or null")
        if cfg.dataloader.drop_last or cfg.val_dataloader.drop_last:
            raise ValueError("Keep incomplete batches: drop_last must be false")

    def _contract(self):
        cfg = self.cfg
        return {
            "policy": _plain(cfg.policy), "optimizer": _plain(cfg.optimizer),
            "ema": _plain(cfg.ema), "num_epochs": int(cfg.training.num_epochs),
            "batch_size": int(cfg.dataloader.batch_size),
            "train_limit": cfg.runtime.train_limit, "val_limit": cfg.runtime.val_limit,
            "seed": int(cfg.training.seed), "val_seed": int(cfg.training.val_seed),
            "lr_scheduler": cfg.training.lr_scheduler,
            "lr_warmup_steps": int(cfg.training.lr_warmup_steps),
            "total_optimizer_steps": len(self.train_loader) * int(cfg.training.num_epochs),
        }

    def setup(self):
        if self._prepared:
            return
        cfg = self.cfg
        output = Path(self.output_dir)
        output.mkdir(parents=True, exist_ok=True)
        (output / "checkpoints").mkdir(exist_ok=True)
        if not cfg.training.resume and any((output / "checkpoints").glob("*.ckpt")):
            raise FileExistsError("Output already contains checkpoints. Set training.resume=<latest.ckpt> "
                                  "or choose a new run.output_dir.")

        self.dataset = hydra.utils.instantiate(cfg.task.dataset)
        self.val_dataset = self.dataset.get_validation_dataset()
        if not len(self.dataset) or not len(self.val_dataset):
            raise ValueError("Both training and validation splits must contain samples")
        self.train_generator = torch.Generator().manual_seed(int(cfg.training.seed))
        self.val_generator = torch.Generator().manual_seed(int(cfg.training.val_seed))
        self.train_loader = DataLoader(
            _limited(self.dataset, cfg.runtime.train_limit), shuffle=True,
            generator=self.train_generator, **_plain(cfg.dataloader),
        )
        self.val_loader = DataLoader(
            _limited(self.val_dataset, cfg.runtime.val_limit), shuffle=False,
            generator=self.val_generator, **_plain(cfg.val_dataloader),
        )

        # Fit on all training strokes even when the CPU/overfit loader is limited.
        normalizer = self.dataset.get_normalizer()
        self.model.set_normalizer(normalizer)
        self.ema_model.set_normalizer(normalizer)
        self.lr_scheduler = get_scheduler(
            cfg.training.lr_scheduler, optimizer=self.optimizer,
            num_warmup_steps=int(cfg.training.lr_warmup_steps),
            num_training_steps=len(self.train_loader) * int(cfg.training.num_epochs),
        )
        self.ema_helper = hydra.utils.instantiate(cfg.ema, model=self.ema_model)
        self.metadata = {
            "schema_version": 1, "upstream_revision": UPSTREAM_REVISION,
            "data_fingerprint": self.dataset.fingerprint,
            "action_definition": self.dataset.manifest["action_definition"],
            "preprocessing": {
                "source_mask_shape": [480, 640], "image_size": [240, 320],
                "resize": "nearest", "channels": "repeat_binary_mask_3_times",
                "raw_mask_range": [0, 1], "normalized_mask_range": [-1, 1],
                "observation_steps": 1, "action_shape": [16, 3],
            },
            "samples": {"train": len(self.dataset), "validation": len(self.val_dataset)},
            "normalizer_source": "all_training_samples",
            "runtime": cfg.runtime.name,
        }
        self.training_contract = self._contract()
        if cfg.training.resume:
            self.restore_checkpoint(cfg.training.resume)

        self.model.to(self.device)
        self.ema_model.to(self.device).eval()
        optimizer_to(self.optimizer, self.device)
        self._prepared = True
        OmegaConf.save(cfg, output / "config.yaml", resolve=True)
        (output / "data_audit.json").write_text(
            json.dumps(self.dataset.describe(), indent=2, ensure_ascii=False) + "\n"
        )
        print(json.dumps({
            "device": str(self.device), "parameters": sum(p.numel() for p in self.model.parameters()),
            "train_samples": len(self.train_loader.dataset),
            "validation_samples": len(self.val_loader.dataset),
            "normalizer_train_samples": len(self.dataset),
            "next_epoch": self.epoch, "total_epochs": int(cfg.training.num_epochs),
            "output_dir": self.output_dir,
        }), flush=True)

    def _capture_rng(self):
        return {
            "python": random.getstate(), "numpy": np.random.get_state(),
            "torch": torch.get_rng_state(),
            "cuda": torch.cuda.get_rng_state_all() if self.device.type == "cuda" else None,
            "train_loader": self.train_generator.get_state(),
            "val_loader": self.val_generator.get_state(),
        }

    def _restore_rng(self):
        state = self.rng_state
        random.setstate(state["python"])
        np.random.set_state(state["numpy"])
        torch.set_rng_state(state["torch"])
        if self.device.type == "cuda" and state["cuda"] is not None:
            # Restore available devices; CPU/GPU migrations are not bitwise identical.
            for index, value in enumerate(state["cuda"][:torch.cuda.device_count()]):
                torch.cuda.set_rng_state(value, device=index)
        self.train_generator.set_state(state["train_loader"])
        self.val_generator.set_state(state["val_loader"])

    def restore_checkpoint(self, path):
        """Load states after building scheduler, without restoring old output paths."""
        with Path(path).open("rb") as stream:
            payload = torch.load(stream, map_location="cpu", pickle_module=dill, weights_only=False)
        required = set(self.include_keys)
        if not required.issubset(payload.get("pickles", {})):
            raise ValueError("Not a complete Push-Wiper training checkpoint")
        required_states = {"model", "ema_model", "optimizer", "lr_scheduler"}
        if not required_states.issubset(payload.get("state_dicts", {})):
            raise ValueError("Checkpoint is missing model, EMA, optimizer or scheduler state")
        metadata = dill.loads(payload["pickles"]["metadata"])
        contract = dill.loads(payload["pickles"]["training_contract"])
        if metadata != self.metadata:
            # Runtime names may differ when moving between devices; data and semantics may not.
            old = {k: v for k, v in metadata.items() if k != "runtime"}
            new = {k: v for k, v in self.metadata.items() if k != "runtime"}
            if old != new:
                raise ValueError("Checkpoint data fingerprint, upstream version, or preprocessing mismatch")
        if contract != self.training_contract:
            raise ValueError("Resume must preserve the model, optimizer, data subset and LR schedule. "
                             "Use stop_after_epochs to interrupt without changing total num_epochs.")
        self.load_payload(payload, include_keys=self.include_keys)
        self.ema_helper.optimization_step = int(self.ema_state["optimization_step"])
        self.ema_helper.decay = float(self.ema_state["decay"])
        if self.global_step != self.ema_helper.optimization_step:
            raise ValueError("Checkpoint optimizer and EMA update counts disagree")
        if (self.global_step != self.epoch * len(self.train_loader)
                or self.lr_scheduler.last_epoch != self.global_step):
            raise ValueError("Checkpoint epoch, optimizer and scheduler counts disagree")
        self._restore_rng()
        print(f"Restored epoch {self.epoch}, optimizer step {self.global_step} from {path}", flush=True)

    def save_checkpoint(self, path=None, tag="latest", **kwargs):
        """Synchronous upstream serialization plus atomic replacement."""
        if kwargs.get("use_thread"):
            raise ValueError("Asynchronous checkpoints are disabled")
        self.ema_state = {
            "optimization_step": self.ema_helper.optimization_step,
            "decay": self.ema_helper.decay,
        }
        self.rng_state = self._capture_rng()
        target = Path(path or self.get_checkpoint_path(tag=tag))
        target.parent.mkdir(parents=True, exist_ok=True)
        temporary = target.with_name(target.name + ".tmp")
        try:
            super().save_checkpoint(
                path=temporary, include_keys=self.include_keys, use_thread=False,
            )
            os.replace(temporary, target)
        finally:
            temporary.unlink(missing_ok=True)
        return str(target)

    @torch.no_grad()
    def validate(self):
        self.ema_model.eval()
        loss_sum, count = 0.0, 0
        with fixed_torch_seed(int(self.cfg.training.val_seed), self.device):
            for batch in self.val_loader:
                batch = dict_apply(batch, lambda x: x.to(self.device, non_blocking=True))
                batch_size = batch["action"].shape[0]
                loss = self.ema_model.compute_loss(batch)
                if not torch.isfinite(loss):
                    raise FloatingPointError("Non-finite EMA validation loss")
                loss_sum += loss.item() * batch_size
                count += batch_size
        return loss_sum / count

    def run(self):
        self.setup()
        cfg = self.cfg
        end_epoch = int(cfg.training.num_epochs)
        if cfg.training.stop_after_epochs is not None:
            end_epoch = min(end_epoch, self.epoch + int(cfg.training.stop_after_epochs))
        start_time = time.monotonic()
        if self.device.type == "cuda":
            torch.cuda.reset_peak_memory_stats(self.device)
        with JsonLogger(str(Path(self.output_dir) / "logs.jsonl")) as logger:
            while self.epoch < end_epoch:
                self.model.train()
                self.ema_model.eval()
                loss_sum, count = 0.0, 0
                epoch_start = time.monotonic()
                for batch in self.train_loader:
                    batch = dict_apply(batch, lambda x: x.to(self.device, non_blocking=True))
                    self.optimizer.zero_grad(set_to_none=True)
                    loss = self.model.compute_loss(batch)
                    if not torch.isfinite(loss):
                        raise FloatingPointError("Non-finite training loss")
                    loss.backward()
                    grad_norm = torch.nn.utils.clip_grad_norm_(
                        self.model.parameters(), max_norm=float("inf"), error_if_nonfinite=True,
                    )
                    self.optimizer.step()
                    self.lr_scheduler.step()
                    self.ema_helper.step(self.model)
                    self.global_step += 1
                    self.optimizer.zero_grad(set_to_none=True)
                    batch_size = batch["action"].shape[0]
                    loss_sum += loss.item() * batch_size
                    count += batch_size
                    logger.log({
                        "global_step": self.global_step, "epoch": self.epoch,
                        "train_batch_loss": loss.item(), "grad_norm": grad_norm.item(),
                        "lr": self.lr_scheduler.get_last_lr()[0],
                    })

                # Saved epoch always identifies the NEXT epoch to execute.
                self.epoch += 1
                final = self.epoch == end_epoch
                metrics = {
                    "epoch": self.epoch, "global_step": self.global_step,
                    "train_loss": loss_sum / count,
                    "lr": self.lr_scheduler.get_last_lr()[0],
                    "ema_updates": self.ema_helper.optimization_step,
                    "epoch_seconds": time.monotonic() - epoch_start,
                }
                if self.epoch % int(cfg.training.val_every) == 0 or final:
                    metrics["val_ema_loss"] = self.validate()
                    if metrics["val_ema_loss"] < self.best_val_loss:
                        self.best_val_loss = metrics["val_ema_loss"]
                        self.save_checkpoint(tag="best")
                if self.epoch % int(cfg.training.sample_every) == 0 or final:
                    evaluation = evaluate_policy(
                        self.ema_model, self.val_dataset,
                        Path(self.output_dir) / "validation" / f"epoch_{self.epoch:04d}",
                        seed=int(cfg.training.sample_seed), limit=cfg.runtime.val_limit,
                    )
                    metrics.update({f"val_{key}": value for key, value in evaluation["aggregate"].items()})
                    # The overfit profile also reports fit to the exact four demonstrations.
                    if cfg.runtime.name == "overfit4090":
                        fitted = evaluate_policy(
                            self.ema_model, self.dataset,
                            Path(self.output_dir) / "overfit" / f"epoch_{self.epoch:04d}",
                            seed=int(cfg.training.sample_seed), limit=cfg.runtime.train_limit,
                        )
                        metrics.update({f"overfit_{key}": value for key, value in fitted["aggregate"].items()})
                if self.device.type == "cuda":
                    metrics["cuda_peak_allocated_gb"] = torch.cuda.max_memory_allocated(self.device) / 2**30
                    metrics["cuda_peak_reserved_gb"] = torch.cuda.max_memory_reserved(self.device) / 2**30
                if self.epoch % int(cfg.training.checkpoint_every) == 0 or final:
                    self.save_checkpoint(tag="latest")
                logger.log(metrics)
                print(json.dumps(metrics), flush=True)
        report = {
            "next_epoch": self.epoch, "global_step": self.global_step,
            "ema_updates": self.ema_helper.optimization_step,
            "lr": self.lr_scheduler.get_last_lr()[0], "best_val_loss": self.best_val_loss,
            "device": str(self.device), "elapsed_seconds": time.monotonic() - start_time,
            "data_fingerprint": self.dataset.fingerprint,
            "completed": self.epoch == int(cfg.training.num_epochs),
        }
        (Path(self.output_dir) / "run_summary.json").write_text(json.dumps(report, indent=2) + "\n")
        return report
