import numpy as np
from multiprocessing.shared_memory import SharedMemory
from multiprocessing.managers import SharedMemoryManager
from typing import Any, Callable, Optional, Union, Dict
from typing_extensions import Self
from airdc.common.utils.shareable_numpy import ShareableNumpy


class ShareableValue:
    """Shareable scalar value, built on top of ShareableNumpy (0-dim array)."""

    def __init__(
        self,
        dtype: Union[np.dtype, str, type] = np.float64,
        shm: Optional[SharedMemory] = None,
        name: Optional[str] = None,
        lock: Optional[Any] = None,
        smm: Optional[SharedMemoryManager] = None,
        initial: Optional[Any] = None,
    ) -> None:
        # 内部持有一个 shape=() 的 ShareableNumpy
        self._sn = ShareableNumpy(
            shape=(),
            dtype=dtype,
            shm=shm,
            name=name,
            lock=lock,
            smm=smm,
        )
        if shm is None and name is None and initial is not None:
            self._sn._array[()] = initial

    # --------- classmethods ---------
    @classmethod
    def from_value(
        cls,
        value: Any,
        dtype: Optional[Union[np.dtype, str, type]] = None,
        shm: Optional[SharedMemory] = None,
        name: Optional[str] = None,
        lock: Optional[Any] = None,
        smm: Optional[SharedMemoryManager] = None,
    ) -> Self:
        if dtype is None:
            dtype = np.asarray(value).dtype
        obj = cls(dtype=dtype, shm=shm, name=name, lock=lock, smm=smm)
        obj._sn._array[()] = value
        return obj

    @classmethod
    def from_value_dict(
        cls,
        val_dict: Dict[str, Any],
        dtypes: Optional[Dict[str, Union[np.dtype, str, type]]] = None,
        shm_dict: Optional[Dict[str, SharedMemory]] = None,
        name_dict: Optional[Dict[str, str]] = None,
        lock: Optional[Any] = None,
        smm: Optional[SharedMemoryManager] = None,
        replace: bool = False,
    ) -> Dict[str, Self]:
        result: Dict[str, ShareableValue] = {}
        for k, v in val_dict.items():
            dt = dtypes[k] if dtypes and k in dtypes else None
            shm = shm_dict[k] if shm_dict and k in shm_dict else None
            name = name_dict[k] if name_dict and k in name_dict else None
            sv = cls.from_value(v, dtype=dt, shm=shm, name=name, lock=lock, smm=smm)
            if replace:
                val_dict[k] = sv  # type: ignore[assignment]
            else:
                result[k] = sv
        return val_dict if replace else result

    # --------- 属性 ---------
    @property
    def dtype(self):
        return self._sn.dtype

    @property
    def name(self):
        return self._sn.name

    @property
    def lock(self):
        return self._sn.lock

    @property
    def value(self) -> Any:
        return self._sn._array[()].item()

    @value.setter
    def value(self, v: Any) -> None:
        self._sn._array[()] = v

    # --------- 基础方法 ---------
    def get(self) -> Any:
        return self._sn._array[()].item()

    def set(self, v: Any) -> None:
        self._sn._array[()] = v

    def safe_get(self) -> Any:
        if self.lock:
            with self.lock:
                return self._sn._array[()].item()
        return self._sn._array[()].item()

    def safe_set(self, v: Any) -> None:
        if self.lock:
            with self.lock:
                self._sn._array[()] = v
        else:
            self._sn._array[()] = v

    def safe_update(self, func: Callable[[Any], Any]) -> None:
        if self.lock:
            with self.lock:
                self._sn._array[()] = func(self._sn._array[()].item())
        else:
            self._sn._array[()] = func(self._sn._array[()].item())

    def close(self):
        self._sn.close()

    def unlink(self):
        self._sn.unlink()

    def __repr__(self) -> str:
        return f"ShareableValue(dtype={self.dtype}, name={self.name}, value={self.value!r})"

    def __reduce__(self):
        return (self.__class__, (self.dtype, None, self.name, self.lock))

    def __int__(self):
        return int(self.value)

    def __float__(self):
        return float(self.value)

    def __bool__(self):
        return bool(self.value)

    def __index__(self):
        return int(self.value)

    def __enter__(self) -> Self:
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        try:
            self.close()
        finally:
            self.unlink()
