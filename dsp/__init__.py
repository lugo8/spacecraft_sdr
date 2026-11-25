"""Reusable I/Q preprocessing on real component arrays of shape (N, 2)."""

from .decimation import decimate_by_two
from .filtering import design_low_pass, low_pass_filter
from .packing import pack_iq8, pack_iq12, unpack_iq8, unpack_iq12
from .quantization import truncate_bits

__all__ = [
    "truncate_bits",
    "design_low_pass",
    "low_pass_filter",
    "decimate_by_two",
    "pack_iq8",
    "unpack_iq8",
    "pack_iq12",
    "unpack_iq12",
]
