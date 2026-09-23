"""Shared Hydra composition for the CLI and integration checks."""

from pathlib import Path

from hydra import compose, initialize_config_dir
from omegaconf import OmegaConf

PROJECT_ROOT = Path(__file__).resolve().parent.parent
CONFIG_DIR = Path(__file__).resolve().parent / "configs"


def register_resolvers():
    OmegaConf.register_new_resolver("project_root", lambda: str(PROJECT_ROOT), replace=True)


def load_config(overrides=()):
    register_resolvers()
    with initialize_config_dir(version_base="1.3", config_dir=str(CONFIG_DIR)):
        cfg = compose(config_name="push_wiper", overrides=list(overrides))
    OmegaConf.resolve(cfg)
    return cfg
