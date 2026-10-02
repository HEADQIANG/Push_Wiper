import numpy as np
from typing import Any, Generic, TypeVar, final
from typing_extensions import Self
from collections.abc import Callable
from abc import abstractmethod
from pydantic import BaseModel, PositiveInt
from logging import getLogger
from airdc.basis import ConfigurableBasis
from airdc.common.environments.basis import (
    EnvironmentBasis,
    EnvironmentOutput,
)


T = TypeVar("T", bound=Callable[..., Any])


class WrapperBasis(ConfigurableBasis, Generic[T]):
    caller: Callable
    # the chained outputs of the wrappers, cleared on reset
    output_chain: list = []
    should_take_over_env: bool = False

    def on_configure(self) -> bool:
        return True

    @final
    def wrap(self, caller: T) -> Self:
        """Wrap the callable with additional functionality."""
        caller_type = self.__annotations__.get("caller", Callable)
        if not isinstance(caller, caller_type):
            raise TypeError(
                f"Expected callable of type {self.caller}, got {type(caller)}"
            )
        self._warmed_up = False
        self.caller = caller
        self.env = None
        return self

    @final
    def take_over_env(self, env: EnvironmentBasis):
        """Set the environment for the topmost wrapper.
        If the subclass needs to take over env, please
        set `self.should_take_over_env = True` in any of __init__ or warm_up.
        """
        if not self._warmed_up:
            raise RuntimeError("Please warm up first")
        if not self.should_take_over_env:
            raise RuntimeError("This wrapper has no need to take over the environment")
        if not isinstance(env, EnvironmentBasis):
            raise TypeError("The `env` must be an instance of EnvironmentBasis")
        self.env = env

    @final
    def warm_up(self, *args, **kwds) -> Any:
        """Warm up the wrapper by calling the caller once and process its output.
        The inputs are the same as the caller inputs."""
        if not hasattr(self, "caller"):
            raise RuntimeError("No caller, please wrap first")
        output = self.caller(*args, **kwds)
        if hasattr(output, "shape"):
            if len(output.shape) in (3, 5):
                self.output_type = type(output)
                self.output_type_name = type(output).__name__
                self.output_dtype = output.dtype
                if self.output_type_name == "ndarray":
                    backend = np
                    self.output_device = "cpu"
                elif self.output_type_name == "Tensor":
                    import torch as backend

                    self.output_device = output.device
                else:
                    raise TypeError(f"Unsupported output type: {self.output_type_name}")
                self.output_backend = backend
                self._warmed_up = True
                return self.on_warm_up(output)
            raise ValueError(
                "The shape of the caller output must be (B, T, C, H, W) or (B, T, D), "
                "i.e. (batch, time, channel, height, width) or (batch, time, dimension)."
                f"But got shape: {output.shape}"
            )
        raise TypeError("The caller output must have a shape")

    @final
    def reset(self):
        """Reset the internal state of the wrapper, if any."""
        if not self._warmed_up:
            raise RuntimeError("Please warm up first")
        if self.taken_over_env:
            self.env.reset()
            self.last_env_output = self.env.output()
        return self.on_reset()

    @abstractmethod
    def on_reset(self):
        """Hook when `reset` is called."""

    @abstractmethod
    def on_warm_up(self, output: Any):
        """Hook when `warm_up` is called. The `output` is the return value of the caller."""

    @abstractmethod
    def call(self, *args, **kwds) -> Any:
        """Call the wrapped callable and return the output."""

    @final
    def __call__(self, *args, **kwds):
        """Call the wrapped callable and update the output chain.
        If the env is not taken over, this method will return
        the output of the wrapped caller, and return the last output of env
        otherwise.
        """
        output = self.call(*args, **kwds)
        self.output_chain.append(output)
        if self.taken_over_env:
            self.env.input(output)
            return self.last_env_output
        return output

    @final
    def shutdown(self) -> bool:
        """Shutdown the wrapper and the environment."""
        if self.on_shutdown():
            if self.taken_over_env:
                return self.env.shutdown()
            return True
        return False

    def on_shutdown(self) -> bool:
        """Shutdown the wrapper, if any."""
        # NOTE: since most wrappers do not need to shutdown,
        # we provide a default implementation that does nothing
        return True

    @classmethod
    def get_logger(cls):
        return getLogger(cls.__name__)

    @classmethod
    def clear_output_chain(cls):
        cls.output_chain = []

    @staticmethod
    def calculate_size(data) -> float:
        type_name = type(data).__name__
        if type_name == "Tensor":
            size_mb = data.numel() * data.element_size() / (1024**2)
        elif type_name == "ndarray":
            size_mb = data.nbytes / (1024**2)
        else:
            raise TypeError(f"Unsupported type name: {type_name}")
        return size_mb

    @property
    def taken_over_env(self) -> bool:
        """Whether the wrapper takes over the environment"""
        return self.env is not None


class ForwardingWrapper(WrapperBasis):
    """A wrapper that just forwards the call to the caller"""

    def on_warm_up(self, output: Any):
        return output

    def on_reset(self):
        """Do nothing"""

    def call(self, *args, **kwds) -> Any:
        return self.caller(*args, **kwds)


class TakeOverEnvWrapper(WrapperBasis):
    """A wrapper that just takes over the environment"""

    should_take_over_env = True

    def on_warm_up(self, output: Any):
        return output

    def on_reset(self):
        """Do nothing"""

    def call(self) -> EnvironmentOutput:
        """Call the environment, using the output
        observation as the caller input and returning
        the environment output
        """
        env_output = self.env.output()
        call_output = self.caller(env_output.observation)
        self.env.input(call_output)
        self.last_env_output = env_output
        return env_output

    def __call__(self, *args, **kwds) -> EnvironmentOutput:
        return super().__call__(*args, **kwds)


class NormalizationConfig(BaseModel):
    mean: float = 0.0
    std: float = 0.0
    min_std: float = 1e-8

    def model_post_init(self, context):
        self.std = self.std or self.min_std


class NormalizerConfig(BaseModel):
    """Configuration to normalize the input and denormalize the output"""

    input: NormalizationConfig = NormalizationConfig()
    output: NormalizationConfig = NormalizationConfig()
    # if input data is a dict and input_key is not empty, normalize the data[input_key]
    input_key: str = ""


class Normalizer(WrapperBasis):
    config: NormalizerConfig

    def normalize_input(self, data: Any) -> Any:
        if isinstance(data, dict):
            key = self.config.input_key
            if not key:
                raise ValueError("data is a dict, but key is empty")
            data[key] = self._normalize(data[key])
            return data
        return self._normalize(data)

    def _normalize(self, data: Any) -> Any:
        return (data - self.config.input.mean) / self.config.input.std

    def denormalize_output(self, data: Any) -> Any:
        return data * self.config.output.std + self.config.output.mean

    def on_warm_up(self, output: Any):
        """Do nothing"""

    def on_reset(self):
        """Do nothing"""

    def call(self, *args, **kwds):
        return self.denormalize_output(self.caller(self.normalize_input(*args, **kwds)))


class FrequencyReductionCallConfig(BaseModel):
    # 1 means call at each step
    period: PositiveInt = 1


class FrequencyReductionCall(WrapperBasis):
    config: FrequencyReductionCallConfig

    def on_warm_up(self, output: Any):
        horizon = output.shape[1]
        period = self.config.period
        if horizon < period:
            raise ValueError(
                f"output horizon: {horizon} can not be shorter than period: {period}"
            )
        return output

    def on_reset(self):
        self._t = 0

    def call(self, *args, **kwds):
        target_t = self._t % self.config.period
        if target_t == 0:
            self._outputs = self.caller(*args, **kwds)
        self._t += 1
        return self._outputs[:, target_t]
