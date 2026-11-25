"""NumPy reference low-pass design and finite-record FIR filtering."""

from numbers import Integral

import numpy as np

from ._validation import as_iq_array, signed_dtype


def design_low_pass(sample_rate_hz, cutoff_hz, num_taps=129, beta=8.6):
    """Design an odd-length, unity-DC-gain Kaiser-windowed sinc low-pass FIR.

    Frequencies are in Hz, relative to complex baseband zero. cutoff_hz is
    the ideal cutoff parameter; a finite transition band surrounds it.
    Group delay is (num_taps-1)/2 input samples. These floating-point
    coefficients are not an LMS7002M register configuration or FPGA design.
    """
    if not np.isfinite(sample_rate_hz) or sample_rate_hz <= 0:
        raise ValueError("sample_rate_hz must be positive and finite")
    if not np.isfinite(cutoff_hz) or not 0 < cutoff_hz < sample_rate_hz / 2:
        raise ValueError("cutoff_hz must lie strictly between 0 and sample_rate_hz/2")
    if (isinstance(num_taps, bool) or not isinstance(num_taps, Integral)
            or num_taps < 3 or num_taps % 2 != 1):
        raise ValueError("num_taps must be an odd integer of at least 3")
    if not np.isfinite(beta) or beta < 0:
        raise ValueError("beta must be finite and nonnegative")
    n = np.arange(num_taps) - (num_taps - 1) // 2
    normalized_cutoff = 2 * cutoff_hz / sample_rate_hz
    coefficients = normalized_cutoff * np.sinc(normalized_cutoff * n)
    coefficients *= np.kaiser(num_taps, beta)
    return coefficients / coefficients.sum()


def low_pass_filter(iq, coefficients, mode="causal", output_bits=12):
    """Apply the same real FIR coefficients to I and Q separately.

    Input shape is (N, 2), in real component counts. ``mode='causal'``
    returns the first N samples of full linear convolution, with zero initial
    history and the filter's physical group delay retained. It is a stateless
    finite-record reference: separate calls do not preserve streaming history.

    ``mode='valid'`` returns only complete overlaps (N-taps+1 rows); use real
    guard samples and align timestamps when comparing with unfiltered data.
    ``mode='same'`` centers the filter offline using edge zero-padding, while
    ``mode='full'`` includes the complete convolution tail. For same/valid,
    the input must be at least as long as the coefficient vector.

    By default, outputs are rounded to nearest and saturated to signed 12-bit
    range in int16 containers, matching the original reference study's output
    arithmetic. Use output_bits=None for unrounded float64 output. Rounding
    uses NumPy's ties-to-even rule; later truncate_bits is a separate operation.
    """
    values = as_iq_array(iq).astype(np.float64)
    h = np.asarray(coefficients)
    if (h.ndim != 1 or not h.size or not np.issubdtype(h.dtype, np.number)
            or np.iscomplexobj(h) or not np.all(np.isfinite(h))):
        raise ValueError("coefficients must be a nonempty finite real vector")
    h = h.astype(np.float64)
    if mode not in ("causal", "full", "same", "valid"):
        raise ValueError("mode must be causal, full, same, or valid")
    dtype = None if output_bits is None else signed_dtype(output_bits)
    if mode in ("same", "valid") and len(values) < len(h):
        raise ValueError("same/valid mode requires at least as many input rows as taps")
    if not len(values):
        if mode != "causal":
            raise ValueError("non-causal convolution modes require nonempty input")
        return np.empty((0, 2), dtype=np.float64 if dtype is None else dtype)
    if mode == "causal":
        filtered = np.column_stack([
            np.convolve(values[:, component], h, mode="full")[:len(values)]
            for component in (0, 1)
        ])
    else:
        filtered = np.column_stack([
            np.convolve(values[:, component], h, mode=mode)
            for component in (0, 1)
        ])
    if not np.all(np.isfinite(filtered)):
        raise ValueError("filter output overflowed; check input scale and coefficients")
    if dtype is None:
        return filtered
    lower, upper = -(1 << (output_bits - 1)), (1 << (output_bits - 1)) - 1
    return np.clip(np.rint(filtered), lower, upper).astype(dtype)
