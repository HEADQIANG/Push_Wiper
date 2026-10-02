from airdc.common.systems.basis import System
from airdc.common.utils.progress import ProgressHandler
from airdc.demonstrate.basis import DemonstrateAction
from pydantic import BaseModel
from abc import abstractmethod


class Demonstrator(System):
    """Abstract base class for all demonstrators."""

    @abstractmethod
    def react(self, action: DemonstrateAction) -> bool:
        """React to a demonstration action."""

    @property
    @abstractmethod
    def handler(self) -> ProgressHandler:
        """Gets the handler for the demonstrator."""


class MockDemonstratorConfig(BaseModel):
    pass


class MockDemonstrator(Demonstrator):
    """Mock implementation of the Demonstrator for testing purposes."""

    config: MockDemonstratorConfig

    def on_configure(self):
        self._handler = ProgressHandler()
