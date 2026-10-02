import numpy as np
from typing import List, Any, Union
from pydantic import BaseModel, NonNegativeInt
from airdc.common.utils.event_rpc import (
    EventRpcManager,
    ConcurrentMode,
    EventRpcServer,
)
from airdc.common.wrappers.basis import WrapperBasis


class TemporalEnsemblingWithDroppingConfig(BaseModel):
    drop_num: NonNegativeInt = 0
    # empty list means average, float means exp decay
    weights: Union[List[float], float] = []
    max_steps: NonNegativeInt = 0


class TemporalEnsemblingWithDropping(WrapperBasis):
    """Temporal Ensembling for output time serial data with dropping method"""

    config: TemporalEnsemblingWithDroppingConfig

    def on_configure(self) -> bool:
        # TODO: use a dynamic method to adjust the
        # buffer size instead of allocating a very
        # large memory
        if self.config.max_steps == 0:
            self.config.max_steps = 2048
        if self.config.drop_num > 0:
            self.should_take_over_env = True
            mode = ConcurrentMode.thread
            self._rpc_manager = EventRpcManager(EventRpcManager.get_args(mode))
            self._call_thread = self._rpc_manager.get_concurrent_cls(mode)(
                target=self._async_call, args=(self._rpc_manager.server,), daemon=True
            )
            self._call_thread.start()
        return True

    def on_warm_up(self, output: Any):
        self._horizon = output.shape[1]
        if self.config.drop_num >= self._horizon:
            raise ValueError(
                f"The `drop_num`: {self.config.drop_num} must be less than the output horizon {self._horizon}"
            )
        self._ele_shape = output.shape[2:]
        weights = self.config.weights
        if isinstance(weights, float):
            weights = np.exp(-weights * np.arange(self._horizon))
        else:
            if not weights:
                weights = np.ones(self._horizon)
            elif len(weights) != self._horizon:
                raise ValueError(
                    f"The length of the weights {weights} must be equal to {self._horizon}"
                )
            weights = np.array(weights)
        if self.output_type_name == "Tensor":
            weights = (
                self.output_backend.from_numpy(weights)
                .to(device=self.output_device)
                .unsqueeze(dim=1)  # add a batch dim
            )
        self._weights = weights / weights.sum()
        # print(self._weights.shape)
        return output

    def on_reset(self):
        self._t = 0
        self._async_t = 0
        self._call_t = 0
        kwds = {}
        if self.output_type == "Tensor":
            kwds = {"device": self.output_device}
        self._all_time_outputs = self.output_backend.zeros(
            (self.config.max_steps, self.config.max_steps + self._horizon)
            + self._ele_shape,
            **kwds,
            dtype=self.output_dtype,
        )
        self.get_logger().info(
            f"Output buffer shape: {self._all_time_outputs.shape}, size: {self.calculate_size(self._all_time_outputs):.2f} MB"
        )

    def _async_call(self, rpc_server: EventRpcServer):
        self.get_logger().info("Call thread started")
        while rpc_server.wait():
            output = self.env.output()
            self.last_env_output = output
            self._sync_call(output.observation)
            rpc_server.respond()
        self.get_logger().info("Call thread exited")

    def _sync_call(self, *args, **kwds):
        t = self._async_t
        outputs = self.caller(*args, **kwds)
        # x axis uses t, y axis uses call_t
        self._all_time_outputs[[self._call_t], t : t + self._horizon] = outputs
        self._call_t += 1

    def call(self, *args, **kwds):
        t = self._t
        # need call
        if t == 0 or self.config.drop_num == 0 or ((t - 1) % self.config.drop_num == 0):
            self._async_t = t
            if self.taken_over_env:
                client = self._rpc_manager.client
                if not client.is_responded():
                    self.get_logger().info("Waiting for call thread to respond...")
                if client.wait(5.0):
                    if not client.request(0):
                        raise RuntimeError("Failed to request")
                    if t == 0:
                        # wait for the first call to be done
                        # to get the initial output
                        client.wait(5.0)
                else:
                    raise TimeoutError("Timeout waiting for call thread to be ready")
            else:
                self._sync_call(*args, **kwds)

        drop_num = self.config.drop_num
        hori = self._horizon
        eq_drop_num = 1 if drop_num == 0 else drop_num
        start = max(0, (t - (hori - eq_drop_num) - 1)) // eq_drop_num + (t > hori - 1)
        end = max(0, (t - 1) // drop_num if drop_num > 0 else t) + 1
        used_outputs = self._all_time_outputs[start:end, t]
        used_weights = self._weights[: end - start]
        # print(used_outputs)
        self._t += 1
        return (used_outputs * used_weights).sum(dim=0, keepdim=True)

    def on_shutdown(self):
        if self.taken_over_env:
            if self._rpc_manager.shutdown():
                self._call_thread.join(5.0)
                return not self._call_thread.is_alive()
        return True
