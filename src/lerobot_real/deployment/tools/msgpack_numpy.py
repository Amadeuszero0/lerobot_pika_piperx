"""MessagePack serialization with safe NumPy array support.

Object, structured, and complex dtypes are rejected so decoding never falls
back to pickle or Python object construction.
"""

import functools
import math

import msgpack
import numpy as np

_ARRAY_MARKER = b"__ndarray__"
_SCALAR_MARKER = b"__npgeneric__"
_UNSUPPORTED_KINDS = {"V", "O", "c"}


def pack_array(obj: object) -> object:
    """Encode NumPy values without falling back to pickle."""
    if isinstance(obj, (np.ndarray, np.generic)) and obj.dtype.kind in _UNSUPPORTED_KINDS:
        raise ValueError(f"Unsupported dtype: {obj.dtype}")

    if isinstance(obj, np.ndarray):
        array = np.ascontiguousarray(obj)
        return {
            _ARRAY_MARKER: True,
            b"data": array.tobytes(),
            b"dtype": array.dtype.str,
            b"shape": array.shape,
        }

    if isinstance(obj, np.generic):
        return {
            _SCALAR_MARKER: True,
            b"data": obj.tobytes(),
            b"dtype": obj.dtype.str,
        }

    return obj


def _decode_dtype(value: object) -> np.dtype:
    dtype = np.dtype(value)
    if dtype.kind in _UNSUPPORTED_KINDS:
        raise ValueError(f"Unsupported dtype: {dtype}")
    return dtype


def unpack_array(obj: dict) -> object:
    """Decode NumPy values and validate shape/data length first."""
    if _ARRAY_MARKER in obj:
        dtype = _decode_dtype(obj[b"dtype"])
        shape_value = obj[b"shape"]
        if not isinstance(shape_value, (list, tuple)) or len(shape_value) > 32:
            raise ValueError("Invalid ndarray shape")
        shape = tuple(int(size) for size in shape_value)
        if any(size < 0 for size in shape):
            raise ValueError("Invalid ndarray shape")

        data = obj[b"data"]
        if not isinstance(data, bytes):
            raise ValueError("Invalid ndarray payload")
        expected_size = math.prod(shape) * dtype.itemsize
        if len(data) != expected_size:
            raise ValueError(
                f"Invalid ndarray payload size: expected {expected_size}, got {len(data)}"
            )
        return np.frombuffer(data, dtype=dtype).reshape(shape).copy()

    if _SCALAR_MARKER in obj:
        dtype = _decode_dtype(obj[b"dtype"])
        data = obj[b"data"]
        if not isinstance(data, bytes) or len(data) != dtype.itemsize:
            raise ValueError("Invalid NumPy scalar payload")
        return np.frombuffer(data, dtype=dtype, count=1)[0]

    return obj


Packer = functools.partial(msgpack.Packer, default=pack_array, use_bin_type=True)
packb = functools.partial(msgpack.packb, default=pack_array, use_bin_type=True)

Unpacker = functools.partial(
    msgpack.Unpacker,
    object_hook=unpack_array,
    raw=False,
    strict_map_key=False,
)
unpackb = functools.partial(
    msgpack.unpackb,
    object_hook=unpack_array,
    raw=False,
    strict_map_key=False,
)
