import numpy as np
from typing import Any, Generic, TypeVar
from abc import abstractmethod
from pydantic import BaseModel, ConfigDict
from logging import getLogger
from airdc.basis import ConfigurableBasis


T = TypeVar("T")


class EnvironmentOutput(BaseModel, Generic[T]):
    model_config = ConfigDict(arbitrary_types_allowed=True)

    # The current observation from the environment
    observation: T
    # The reward obtained from the last action
    reward: float = 0.0
    # Whether the agent reaches the terminal state (as defined under the MDP of the task) which can be positive or negative. If true, the user needs to call reset().
    terminated: bool = False
    # Whether the truncation condition outside the scope of the MDP is satisfied. Typically, this is a timelimit, but could also be used to indicate an agent physically going out of bounds. Can be used to end the episode prematurely before a terminal state is reached. If true, the user needs to call reset().
    truncated: bool = False
    # Contains auxiliary diagnostic information (helpful for debugging, learning, and logging). This might, for instance, contain: metrics that describe the agent’s performance state, variables that are hidden from observations, or individual reward terms that are combined to produce the total reward.
    info: dict = {}


class EnvironmentBasis(ConfigurableBasis, Generic[T]):
    @abstractmethod
    def reset(self) -> None:
        """Reset the environment"""

    @abstractmethod
    def input(self, input: Any) -> None:
        """Take an action in the environment"""

    @abstractmethod
    def output(self) -> EnvironmentOutput[T]:
        """Get the current output from the environment.
        This method should not change the state of the environment.
        """

    @abstractmethod
    def shutdown(self) -> bool:
        """Shutdown the environment"""

    @classmethod
    def get_logger(cls):
        return getLogger(cls.__name__)


class MockEnvironmentConfig(BaseModel):
    max_steps: int = 0
    output: Any = None


class MockEnvironment(EnvironmentBasis):
    config: MockEnvironmentConfig

    def on_configure(self) -> bool:
        self.reset()
        return True

    def reset(self) -> None:
        self.get_logger().info("Environment resetting...")
        self._t = 0
        self._output_t = 0

    def input(self, input: Any) -> None:
        # self.get_logger().info(f"Environment input: {input}")
        self._t += 1

    def output(self) -> EnvironmentOutput[np.ndarray]:
        # self._output_t += 1
        # if self._t != self._output_t:
        #     raise RuntimeError(
        #         "The environment output is out of sync with the input. Please make sure to call input() before output()"
        #     )
        terminated = (
            (self._t >= self.config.max_steps) if self.config.max_steps > 0 else False
        )
        # self.get_logger().info(f"Environment step: {self._t}, terminated: {terminated}")
        return EnvironmentOutput(
            observation=np.array([self._t])
            if self.config.output is None
            else self.config.output,
            terminated=terminated,
        )

    def shutdown(self) -> bool:
        return True
