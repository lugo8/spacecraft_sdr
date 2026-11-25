# I/Q preprocessing

Functions for bit truncation, low-pass filtering, factor-two decimation, and
sample packing.

## Files

| File | What it does |
|---|---|
| `quantization.py` | `truncate_bits()` removes the least significant bits from each signed I/Q component. Defaults to 12 bits → 8 bits. |
| `filtering.py` | `design_low_pass()` creates FIR filter coefficients; `low_pass_filter()` applies them separately to I and Q. |
| `decimation.py` | `decimate_by_two()` keeps every second I/Q pair. `phase=0` starts at the first pair; `phase=1` starts at the second. |
| `packing.py` | Packs and unpacks signed I/Q pairs: three bytes per 12-bit pair or two bytes per 8-bit pair. |
| `rtl/quantizer.v` | Converts one signed 12-bit component to 8 bits by discarding its four least significant bits. Uses `in_valid`/`in_ready` and `out_valid`/`out_ready` handshakes. |
| `_validation.py` | Checks array shape, finite values, component ranges, and bit widths. |
| `__init__.py` | Exposes the processing functions through `from dsp import ...`. |
| `requirements.txt` | Lists the NumPy dependency. |

## Usage

From the repository root:

```sh
python -m pip install -r dsp/requirements.txt
```

Use an array shaped `(N, 2)`, with I in the first column and Q in the second.
For 12-bit input, component values must be integers from -2048 to 2047.

```python
import numpy as np
from dsp import truncate_bits, design_low_pass, low_pass_filter, decimate_by_two

# Example input: interleaved I,Q values stored as little-endian int16.
iq = np.fromfile("capture_iq16le.bin", dtype="<i2").reshape(-1, 2)
sample_rate_hz = 4_096_000

# Truncate 12-bit components to 8 bits.
iq8 = truncate_bits(iq, input_bits=12, output_bits=8)

# Apply a low-pass filter at the original sample rate.
coefficients = design_low_pass(
    sample_rate_hz=sample_rate_hz,
    cutoff_hz=900_000,
    num_taps=129,
)
filtered12 = low_pass_filter(iq, coefficients)

# Keep every second pair: 4.096 → 2.048 million pairs per second.
half_rate12 = decimate_by_two(filtered12)

# Combine filtering, decimation, and bit truncation.
half_rate8 = truncate_bits(half_rate12)
```

- Truncation divides component counts by 16 and rounds down for 12 → 8 bits.
  Multiply the result by 16 in a wider dtype when comparing amplitudes with the input.
- The filter defaults to rounded, clipped 12-bit output. Set `output_bits=None`
  for floating-point output. The 129-tap filter delays the signal by 64 input
  samples; each call starts with zero filter history.
- Decimation does not filter automatically. Filter first to suppress frequencies
  that would alias at the lower sample rate. Choose the cutoff for your signal;
  900 kHz is the example value above.

## Packing and unpacking

```python
from dsp import pack_iq12, unpack_iq12, pack_iq8, unpack_iq8

packed12 = pack_iq12(iq)          # Three bytes per I/Q pair.
restored12 = unpack_iq12(packed12)

packed8 = pack_iq8(half_rate8)    # Two bytes per I/Q pair.
restored8 = unpack_iq8(packed8)
```

Packing preserves the component values supplied to it. Unpacking does not
restore bits discarded by truncation.
