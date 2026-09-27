from airdc.basis import ConcurrentMode
from airdc.common.utils.progress import (
    Waitable,
    ConcurrentProgressHandler,
)
from airdc.utils import StrEnum
from multiprocessing import get_context
from enum import auto
from pydantic import NonNegativeInt, BaseModel
import time


class DemonstrateAction(StrEnum):
    configure = auto()
    activate = auto()
    capture = auto()
    sample = auto()
    update = auto()
    save = auto()
    remove = auto()
    abandon = auto()
    deactivate = auto()
    finish = auto()


class DemonstrateState(StrEnum):
    error = auto()
    unconfigured = auto()
    inactive = auto()
    active = auto()
    sampling = auto()
    finalized = auto()


SpawnEvent = get_context("spawn").Event


class SampleInfo(BaseModel):
    """Information of the current sampling episode."""

    # NOTE: no frozen, no validation for performance consideration
    index: NonNegativeInt = 0
    """the current index (number) of the data in a single sample episode"""
    episode: NonNegativeInt = 0
    """the current episode of the sampling"""


if __name__ == "__main__":
    from airdc.utils import init_logging

    init_logging()

    handler = ConcurrentProgressHandler(ConcurrentMode.thread)

    def auto_control(waitable: Waitable):
        time.sleep(2)
        with waitable:
            while waitable.wait():
                print("Auto control is running...")
                time.sleep(1)
            print("Auto control has stopped.")

    handler.launch(target=auto_control, args=(handler.get_waitable(),), daemon=False)
    input("Press Enter to start...")
    handler.start()
    input("Press Enter to stop...")
    handler.stop()
    input("Press Enter to exit...")
    handler.exit()
