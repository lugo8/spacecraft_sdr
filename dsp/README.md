# I/Q sample preprocessing

Small reusable functions for examining bit truncation, low-pass filtering, and
factor-two decimation. This folder contains processing code and a usage guide;
it does not contain signal generators, simulation runners, testbenches, or
experiment results.

Python functions are finite-record NumPy references. The only Verilog block is
the 12-to-8-bit component quantizer. Filtering and decimation are not implemented
in FPGA logic here, and nothing in this package configures or programs an SDR.

## Files

| File | Function |
|---|---|
| `quantization.py` | Discard low bits from each signed I/Q component |
| `filtering.py` | Design a low-pass FIR and apply it to I and Q |
| `decimation.py` | Keep every second complete I/Q pair |
| `packing.py` | Optional compact 12-bit/8-bit packing and unpacking |
| `rtl/quantizer.v` | Original ready/valid Verilog 12-to-8-bit component converter |
| `_validation.py` | Shared shape/range validation |

## Requirements and input convention

Use Python 3.10+ and NumPy. From the repository root:

```sh
python -m pip install -r dsp/requirements.txt
```

All Python operations accept an array with shape `(N, 2)`:

```python
# Each row is one sampling instant; columns are I and Q.
iq = np.column_stack((i_components, q_components))
```

Values are component counts, not normalized floating-point amplitudes. Signed
12-bit input counts range from -2048 through 2047. For a complex-valued array,
use `np.column_stack((samples.real, samples.imag))`; ensure values are scaled
to component counts before truncation. Truncation requires integer-valued
components and checks their range.

For existing 12-bit-range samples stored in interleaved little-endian int16
containers, an example read is:

```python
import numpy as np

iq = np.fromfile("capture_iq16le.bin", dtype="<i2").reshape(-1, 2)
```

That read applies only to that stated format, not arbitrary SDR captures.

## Use the three operations

Run from the repository root so `dsp` is importable:

```python
from dsp import truncate_bits, design_low_pass, low_pass_filter, decimate_by_two

sample_rate_hz = 4_096_000

# 1. Same sample times, fewer amplitude bits: signed 12 -> signed 8.
iq8 = truncate_bits(iq, input_bits=12, output_bits=8)

# 2. Same sample rate, retained bandwidth narrowed by an FIR.
coefficients = design_low_pass(
    sample_rate_hz=sample_rate_hz,
    cutoff_hz=900_000,
    num_taps=129,
    beta=8.6,
)
filtered12 = low_pass_filter(iq, coefficients, mode="causal", output_bits=12)

# 3. Reduce sample rate after filtering: 4.096 -> 2.048 million I/Q pairs/s.
filtered_half12 = decimate_by_two(filtered12)
output_sample_rate_hz = sample_rate_hz / 2

# Combine rate and precision reduction.
filtered_half8 = truncate_bits(filtered_half12, input_bits=12, output_bits=8)
```

`truncate_bits` floors by `2**(input_bits-output_bits)`, matching the Verilog.
For example, 100 -> 6, -1 -> -1, -17 -> -2. It does not round to nearest.
Multiply 8-bit output by 16 in a wider dtype to compare amplitude on the
original scale; that does not restore discarded precision.

`low_pass_filter` uses float64 convolution internally. By default it rounds to
nearest (ties to even), clips to the signed 12-bit range, and returns int16
containers. Set `output_bits=None` to retain unrounded float64 filter output.
This rounding is separate from the later bit truncation.

`decimate_by_two` only selects rows. It does not filter automatically. Its
`phase=0` keeps rows 0, 2, 4, ...; `phase=1` keeps rows 1, 3, 5, ... . Preserve
the corresponding time origin and update sample-rate metadata in downstream
processing. Use an adequate anti-alias filter first unless the input is already
sufficiently band-limited.

## Recreate the six processing configurations

```python
configurations = {
    "original12": iq.copy(),
    "truncated8": truncate_bits(iq),
    "half12_unfiltered": decimate_by_two(iq),
    "filtered12": filtered12,
    "filtered_half12": filtered_half12,
    "filtered_half8": filtered_half8,
}
```

The unfiltered half-rate configuration is a comparison control, not a generally
safe downsampling method.

| Configuration | Rate (million pairs/s) | Bits/component | Ideal packed payload |
|---|---:|---:|---:|
| Original | 4.096 | 12 | 98.304 Mbit/s |
| Truncated | 4.096 | 8 | 65.536 Mbit/s |
| Half rate, unfiltered | 2.048 | 12 | 49.152 Mbit/s |
| Filter only | 4.096 | 12 | 98.304 Mbit/s |
| Filter + half rate | 2.048 | 12 | 49.152 Mbit/s |
| Filter + half rate + truncation | 2.048 | 8 | 32.768 Mbit/s |

Payload is `sample_rate * 2 * bits_per_component`, excluding transport overhead.
The 12-bit figures assume three bytes per pair, not padded int16 storage.

## Filter bandwidth, edges, and latency

The 900 kHz / 129-tap Kaiser design reproduces the prototype coefficient design
used in our reference study. It is an experimental choice, not a fixed GPS or
mission requirement. At 2.048 million complex pairs/s, the new baseband limits
are +/-1.024 MHz. The filter needs a transition band and must suppress energy
before that boundary. Its finite response is not a brick wall at 900 kHz.

Filtering can remove useful timing information as well as noise. Reducing bits
adds quantization error. Whether the tradeoff is acceptable depends on the
application; these functions do not claim to preserve all signal information.

The default `mode="causal"` returns N output rows, assumes zero prior history,
and retains the FIR delay. A symmetric 129-tap filter has delay 64 input samples,
or 15.625 microseconds at 4.096 million pairs/s. Account for this delay and
startup transients when comparing waveforms or correlation peaks.

For an offline comparison with real guard data, the original reference-study
alignment can be reproduced as:

```python
# iq_with_guard includes 64 actual samples before and after the scored interval.
guard = (len(coefficients) - 1) // 2
original_aligned = iq_with_guard[guard:-guard]
filtered_aligned = low_pass_filter(
    iq_with_guard, coefficients, mode="valid", output_bits=12
)
```

Both arrays then have equal lengths, with the known group delay accounted for
in the comparison's time origin. Physical filtering still requires latency.
`mode="same"` instead performs centered offline convolution with edge zeros;
`mode="full"` includes the complete convolution tail. See the function docstring.

Each function call filters a complete record independently. Do not process live
chunks by repeatedly calling it with no retained state: that resets the filter
history at every chunk boundary. Continuous streaming needs persistent history,
correct clock handling, and an explicit overflow policy.

## Optional compact packing

```python
from dsp import pack_iq12, unpack_iq12, pack_iq8, unpack_iq8

packed12_bytes = pack_iq12(iq)                 # 3 bytes per pair
packed8_bytes = pack_iq8(filtered_half8)       # 2 bytes per pair
iq12_restored = unpack_iq12(packed12_bytes)
iq8_restored = unpack_iq8(packed8_bytes)
```

Twelve-bit packing stores I in the low 12 bits and Q in the high 12 bits of each
three-byte little-endian word. Eight-bit packing is signed I,Q byte interleaving.
Packing/unpacking preserves the selected representation exactly; it does not
undo truncation. These helpers add no headers, timestamps, CRC, or SPI logic.

## Verilog component converter

`rtl/quantizer.v` is the original signed 12-to-8-bit block. Input/output
transactions use ready/valid handshakes, and an unaccepted output stays stable.
It converts one component at a time, not an entire complex pair. Applying it to
I/Q requires maintaining pair order and alignment in the surrounding design.
The real ADC cannot necessarily pause when this block sees backpressure, so a
board implementation needs buffering and overflow handling.

The Python processing functions and the Verilog converter are not integrated
into the LimeSDR gateware here. FIR coefficient quantization, accumulator widths,
FPGA resource use and actual timing remain implementation work.
