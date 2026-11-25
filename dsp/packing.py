"""Compact signed I/Q byte representations, without headers or transport code."""

import numpy as np

from ._validation import integer_components


def pack_iq12(iq):
    """Return three bytes per signed 12-bit I/Q pair, I in low 12 bits.

    b0=I[7:0], b1={Q[3:0], I[11:8]}, b2=Q[11:4]. This removes int16
    container padding without changing the represented component values.
    """
    values = integer_components(iq, 12) & 0xFFF
    i, q = values[:, 0], values[:, 1]
    packed = np.column_stack((i & 255, (i >> 8) | ((q & 15) << 4), q >> 4))
    return packed.astype(np.uint8).tobytes()


def unpack_iq12(data):
    """Decode pack_iq12 bytes into shape (N, 2), signed int16 containers."""
    raw = np.frombuffer(data, dtype=np.uint8)
    if raw.size % 3:
        raise ValueError("packed 12-bit data length must be divisible by 3")
    raw = raw.reshape(-1, 3).astype(np.int32)
    i = raw[:, 0] | ((raw[:, 1] & 15) << 8)
    q = (raw[:, 1] >> 4) | (raw[:, 2] << 4)
    values = np.column_stack((i, q))
    return np.where(values >= 2048, values - 4096, values).astype(np.int16)


def pack_iq8(iq):
    """Return two bytes per signed 8-bit pair, interleaved I then Q."""
    return integer_components(iq, 8).astype(np.int8).tobytes()


def unpack_iq8(data):
    """Decode pack_iq8 bytes into a writable signed int8 array, shape (N, 2)."""
    raw = np.frombuffer(data, dtype=np.int8)
    if raw.size % 2:
        raise ValueError("packed 8-bit data length must be divisible by 2")
    return raw.reshape(-1, 2).copy()
