"""Reduce component precision by discarding low bits."""

import numpy as np

from ._validation import integer_components, signed_dtype


def truncate_bits(iq, input_bits=12, output_bits=8):
    """Discard low bits from each signed I/Q component.

    ``iq`` has shape (N, 2) and integer-valued I,Q columns. The operation is
    floor division by 2**(input_bits-output_bits), matching quantizer.v.
    It does not round to nearest or silently clip out-of-range inputs.

    The returned array keeps the same shape and uses the smallest signed
    int8/int16/int32 container that holds output_bits. Scale is reduced by the
    division factor; multiplying back restores scale, not discarded precision.
    """
    dtype = signed_dtype(output_bits)
    values = integer_components(iq, input_bits)
    if output_bits > input_bits:
        raise ValueError("output_bits must not exceed input_bits")
    shift = int(input_bits - output_bits)
    return np.right_shift(values, shift).astype(dtype)
