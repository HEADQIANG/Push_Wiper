"""One process-wide acquisition timeline, unaffected by wall-clock corrections."""

import time
from threading import Lock


TIMESTAMP_BASIS = (
    "host monotonic_ns anchored to Unix time at process start; "
    "not hardware synchronized"
)


class AcquisitionClock:
    def __init__(self):
        self._lock = Lock()
        self._monotonic_origin = time.monotonic_ns()
        self._epoch_origin = time.time_ns()
        self._last_ns = self._epoch_origin - 1

    def now_ns(self):
        with self._lock:
            stamp = self._epoch_origin + time.monotonic_ns() - self._monotonic_origin
            # Some clocks can return equal consecutive readings. Only resolve
            # those ties; elapsed time always comes from the monotonic clock.
            self._last_ns = max(stamp, self._last_ns + 1)
            return self._last_ns


_clock = AcquisitionClock()


def acquisition_ns():
    return _clock.now_ns()
