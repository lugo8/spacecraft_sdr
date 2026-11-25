"""Shared validation for real-valued, interleaved I/Q component arrays."""

from numbers import Integral

import numpy as np


def as_iq_array(iq):
    """Return a finite real numeric array with shape (sample_count, 2)."""
    values = np.asarray(iq)
    if values.ndim != 2 or values.shape[1] != 2:
        raise ValueError("iq must have shape (sample_count, 2), with columns I and Q")
    if not np.issubdtype(values.dtype, np.number) or np.iscomplexobj(values):
        raise ValueError("I and Q must be separate real numeric columns")
    if not np.all(np.isfinite(values)):
        raise ValueError("iq must contain only finite values")
    return values


def signed_dtype(bits):
    """Validate a signed component width and choose its storage container."""
    if isinstance(bits, bool) or not isinstance(bits, Integral) or not 2 <= bits <= 32:
        raise ValueError("bit widths must be integers from 2 through 32")
    return np.int8 if bits <= 8 else np.int16 if bits <= 16 else np.int32


def integer_components(iq, bits):
    """Validate integer-valued components within a signed bit range."""
    signed_dtype(bits)
    values = as_iq_array(iq)
    if not np.all(values == np.rint(values)):
        raise ValueError("components must be integer-valued; round filter output first")
    lower, upper = -(1 << (bits - 1)), (1 << (bits - 1)) - 1
    if np.any(values < lower) or np.any(values > upper):
        raise ValueError(f"components must fit signed {bits}-bit range [{lower}, {upper}]")
    return values.astype(np.int64)
