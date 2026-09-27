from typing import Dict, List, TypeVar, Generic
from pydantic import BaseModel, NonNegativeFloat, ConfigDict, model_validator
from airdc.basis import ConcurrentMode, force_set_attr


T = TypeVar("T")


class ComponentConfig(BaseModel, Generic[T], frozen=True):
    """The config of one component to be used in the demonstration."""

    model_config = ConfigDict(arbitrary_types_allowed=True, extra="forbid")

    name: str = ""
    """name of the component"""
    instance: T = None
    """the component instance"""
    concurrent: ConcurrentMode = ConcurrentMode.none
    """the concurrent mode of the component"""
    update_rate: NonNegativeFloat = 0
    """the update rate of the component (Hz), 0 means no limit"""


class ComponentsConfig(BaseModel, Generic[T], frozen=True):
    """The config of multiple components to be used in the demonstration."""

    # TODO: set extra="forbid" after pydantic fix relevant bugs
    model_config = ConfigDict(arbitrary_types_allowed=True)

    names: List[str] = []
    """names of the components, e.g. ("left_arm", "right_arm", "left_camera");
    if empty, no component will be used"""
    instances: List[T] = []
    """the component instances"""
    concurrents: List[ConcurrentMode] = []
    """the concurrent modes of the components"""
    update_rates: List[NonNegativeFloat] = []
    """the update rates of the components (Hz), 0 means no limit"""

    @model_validator(mode="after")
    @force_set_attr
    def validate_lengths(self):
        name_length = len(self.names)
        if name_length == 0:
            self.instances.clear()
            self.concurrents.clear()
            self.update_rates.clear()
        else:
            if name_length != len(self.instances):
                raise ValueError("names and instances must have the same length")
            if len(self.concurrents) == 1:
                self.concurrents *= name_length
            elif not self.concurrents:
                self.concurrents = [ConcurrentMode.none] * name_length
            if len(self.update_rates) == 1:
                self.update_rates *= name_length
            elif not self.update_rates:
                self.update_rates = [0.0] * name_length
            if name_length != len(self.update_rates):
                raise ValueError("names and update_rates must have the same length")
        return self

    @model_validator(mode="after")
    def check_unique_names(self):
        # NOTE: This validation logic will be overridden as `model_validator` in the subclass.
        # Using `field_validator` here will cause validation exceptions in the subclass.
        names = self.names
        if len(names) != len(set(names)):
            raise ValueError(f"names must be unique, got {names}")
        return self

    @property
    def instance_dict(self) -> Dict[str, T]:
        """Returns a dictionary of component instances."""
        return dict(zip(self.names, self.instances))
