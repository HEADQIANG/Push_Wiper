from airdc.common.demonstrators.basis import Demonstrator
from pydantic import BaseModel


class SingleComponentDemonstratorConfig(BaseModel):
    """Configuration for the single component demonstrator."""

    pass


class SingleComponentDemonstrator(Demonstrator):
    """Demonstrator for a single component."""

    def __init__(self, config: SingleComponentDemonstratorConfig):
        self.config = config

    def start(self):
        """Starts the demonstration."""
        pass
