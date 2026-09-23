"""Train an official image Diffusion Policy on exported Push-Wiper strokes."""

import hydra
from omegaconf import OmegaConf

from .config import register_resolvers

register_resolvers()


@hydra.main(version_base="1.3", config_path="configs", config_name="push_wiper")
def main(cfg):
    from .workspace import PushWiperWorkspace

    OmegaConf.resolve(cfg)
    PushWiperWorkspace(cfg).run()


if __name__ == "__main__":
    main()
