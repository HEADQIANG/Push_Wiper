from pydantic import BaseModel, ConfigDict
from functools import cached_property
from typing import Generic, TypeVar

T = TypeVar("T")


class Component(BaseModel, Generic[T]):
    # model_config = ConfigDict(extra="forbid")
    model_config = ConfigDict(extra="forbid")

    name: T

    def model_post_init(self, context):
        print(self.unique_key)
        pass

    @cached_property
    def unique_key(self) -> str:
        return self.name.upper()


class Config(BaseModel, Generic[T]):
    item: Component[T]


comp = Component(name="test")
print(f"{comp.__dict__=}")
print(f"{comp.model_dump()=}")
cfg = Config[str](item=comp)
