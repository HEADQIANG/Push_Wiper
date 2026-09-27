from pydantic import BaseModel, NonNegativeFloat
from airdc.demonstrate.configs import DemonstrateConfig
from airdc.common.configs.component import ComponentsConfig
from airdc.state_machine.fsm import (
    DemonstrateFSMConfig,
    StateMachineConfig,
)


class DataCollectionConfig(BaseModel, frozen=True):
    """Configuration for the data collection."""

    update_rate: NonNegativeFloat = 0
    """the maximum rate for the managers
    # 0 means as fast as possible"""
    fsm: DemonstrateFSMConfig
    """the finite state machine config"""
    managers: ComponentsConfig
    """managers to control the demonstrate actions"""
    log_metrics: int = -1
    """log metrics every N seconds, -1 to disable"""
    log_jitter: bool = True
    """whether to log jitter statistics"""


class DataCollectionArgs(DemonstrateConfig, DataCollectionConfig):
    """Top level arguments for the data collection.
    The structure is similar but not identical to
    `DataCollectionConfig` which is more suitable
    for the CLI configuration.
    """

    # the finite state machine config
    fsm: StateMachineConfig
