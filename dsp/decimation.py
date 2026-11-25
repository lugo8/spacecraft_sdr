"""Factor-two downsampling of complete I/Q pairs."""

from numbers import Integral

from ._validation import as_iq_array


def decimate_by_two(iq, phase=0):
    """Keep every second I/Q pair, starting at row 0 or row 1.

    This function only selects rows; it does not apply an anti-alias filter.
    Filter first unless the input is already adequately band-limited. The
    returned copy preserves component dtype and I/Q pairing. Output sample
    rate is input_rate/2; output row k corresponds to input row 2*k+phase.
    """
    if isinstance(phase, bool) or not isinstance(phase, Integral) or phase not in (0, 1):
        raise ValueError("phase must be 0 or 1")
    return as_iq_array(iq)[phase::2].copy()
