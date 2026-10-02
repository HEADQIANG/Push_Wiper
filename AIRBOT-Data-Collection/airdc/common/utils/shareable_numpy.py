"""Utilities for creating and manipulating shareable NumPy arrays via multiprocessing shared memory.

This module defines a thin wrapper `ShareableNumpy` around `multiprocessing.shared_memory.SharedMemory`
that provides:
    * Convenient creation or attachment to an existing shared memory block.
    * A NumPy ndarray view backed by the shared memory buffer.
    * Optional process/thread safe mutations guarded by a Lock.
    * Pickle support that re-attaches to the shared memory segment in child processes.
    * Helper registration for use with `SharedMemoryManager`.
"""

import numpy as np
from multiprocessing.shared_memory import SharedMemory
from multiprocessing.managers import SharedMemoryManager
from typing import Any, Callable, Optional, Sequence, Union, Dict
from typing_extensions import Self


class ShareableNumpy:
    """Shareable numpy ndarray backed by multiprocessing SharedMemory.

    Provides safe optional locking and pickle-friendly reattachment by name.
    """

    def __init__(
        self,
        shape: Optional[Sequence[int]] = None,
        dtype: Union[np.dtype, str, type] = np.float64,
        shm: Optional[SharedMemory] = None,
        name: Optional[str] = None,
        lock: Optional[Any] = None,
        smm: Optional[SharedMemoryManager] = None,
    ) -> None:
        self.shape = tuple(shape) if shape is not None else None
        self.dtype = np.dtype(dtype)
        self.lock = lock

        if shm is not None:
            self.shm = shm
        elif name is not None:
            self.shm = SharedMemory(name=name)
        else:
            if self.shape is None:
                raise ValueError(
                    "'shape' must be provided when creating new shared memory"
                )
            nbytes = int(np.prod(self.shape)) * self.dtype.itemsize
            if smm is None:
                self.shm = SharedMemory(create=True, size=nbytes)
            else:
                self.shm = smm.SharedMemory(nbytes)

        if self.shape is None:
            raise ValueError("Unable to infer shape for ndarray view")
        self._array = np.ndarray(self.shape, dtype=self.dtype, buffer=self.shm.buf)

    @classmethod
    def from_array(
        cls,
        arr: np.ndarray,
        shm: Optional[SharedMemory] = None,
        name: Optional[str] = None,
        lock: Optional[Any] = None,
        smm: Optional[SharedMemoryManager] = None,
    ) -> Self:
        obj = cls(
            shape=arr.shape, dtype=arr.dtype, shm=shm, name=name, lock=lock, smm=smm
        )
        np.copyto(obj._array, arr)
        return obj

    @classmethod
    def from_array_dict(
        cls,
        arr_dict: dict[str, np.ndarray],
        shm_dict: Optional[dict[str, SharedMemory]] = None,
        name_dict: Optional[dict[str, str]] = None,
        lock: Optional[Any] = None,
        smm: Optional[SharedMemoryManager] = None,
        replace: bool = False,
    ) -> Dict[str, Self]:
        result = {}
        for k, arr in arr_dict.items():
            shm = shm_dict[k] if shm_dict is not None and k in shm_dict else None
            name = name_dict[k] if name_dict is not None and k in name_dict else None
            shm_arr = cls.from_array(arr, shm=shm, name=name, lock=lock, smm=smm)
            if replace:
                arr_dict[k] = shm_arr
            else:
                result[k] = shm_arr
        if replace:
            return arr_dict
        return result

    @property
    def name(self) -> str:  # type: ignore[override]
        return self.shm.name

    def close(self) -> None:
        self.shm.close()

    def unlink(self) -> None:
        self.shm.unlink()

    def to_numpy(self, copy=True, readonly: bool = True) -> np.ndarray:
        arr = np.array(self._array, copy=copy)
        if readonly:
            arr.flags.writeable = False
        return arr

    def safe_get(self, idx: Any) -> Any:
        if self.lock:
            with self.lock:
                return self._array[idx]
        return self._array[idx]

    def safe_set(self, idx: Any, value: Any) -> None:
        if self.lock:
            with self.lock:
                self._array[idx] = value
        else:
            self._array[idx] = value

    def safe_update(self, func: Callable[[np.ndarray], None]) -> None:
        if self.lock:
            with self.lock:
                func(self._array)
        else:
            func(self._array)

    def __array__(self) -> np.ndarray:
        return self._array

    def __getitem__(self, idx: Any) -> Any:
        return self._array[idx]

    def __setitem__(self, idx: Any, value: Any) -> None:
        self._array[idx] = value

    def __repr__(self) -> str:
        return f"ShareableNumpy(shape={self.shape}, dtype={self.dtype}, name={self.shm.name})"

    def __reduce__(self):
        return (
            self.__class__,
            (self.shape, self.dtype, None, self.shm.name, self.lock),
        )

    def __enter__(self) -> Self:
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:  # type: ignore[override]
        try:
            self.close()
        finally:
            self.unlink()

    @property
    def array(self) -> np.ndarray:
        """Get the underlying NumPy array (read-write)."""
        return self._array

    @array.setter
    def array(self, value: np.ndarray) -> None:
        self._array[:] = value


# === extend SharedMemoryManager ===
def _ShareableNumpy_factory(
    shape: Sequence[int], dtype: Union[np.dtype, str, type] = np.float64
) -> ShareableNumpy:
    return ShareableNumpy(shape=shape, dtype=dtype)


def _ShareableNumpy_from_array_factory(arr: np.ndarray) -> ShareableNumpy:
    return ShareableNumpy.from_array(arr)


SharedMemoryManager.register("ShareableNumpy", _ShareableNumpy_factory)
SharedMemoryManager.register(
    "ShareableNumpy_from_array", _ShareableNumpy_from_array_factory
)
