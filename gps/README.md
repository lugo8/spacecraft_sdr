# gps/

GPS L1 C/A receiver — software receiver, FPGA correlator RTL, and host controller.

## Files

### Host software

| File | Description |
|------|-------------|
| `gps_rx.c` | Standalone software GPS receiver. Processes raw I/Q samples (file or stream) entirely on the CPU. FFT-based acquisition, software tracking loops, nav data decode, PVT. |
| `gps_fpga.cpp` | Host controller for the FPGA correlator. Drives acquisition, manages tracking channels, demodulates nav data, computes position fixes. Talks to the FPGA via LiteX CSR registers through LimeSuiteNG ICSR. |

### GNURadio capture

| File | Description |
|------|-------------|
| `gps_recv.grc` | GNURadio flowgraph for HackRF GPS L1 capture |
| `gps_recv.py` | Generated Python from the GRC flowgraph |
| `gps_recv_lime.grc` | GNURadio flowgraph for LimeSDR GPS L1 capture |
| `gps_capture.cpp` | C++ capture utility (LimeSDR direct, no GNURadio) |

### FPGA RTL

See [`rtl/README.md`](rtl/README.md).

### Documentation

| File | Description |
|------|-------------|
| `FPGA_GPS_CORRELATOR.md` | FPGA resource analysis and architecture overview |
| `LIMESDR_GPS_SETUP.md` | LimeSDR Mini v2 GPS test setup and results |
| `HACKRF_GPS_SETUP.md` | HackRF GPS capture setup |

## Build

```bash
# Software receiver (runs anywhere)
gcc -O3 -march=native -o gps_rx gps_rx.c -lm

# FPGA host controller (runs on Pi with LimeSDR)
g++ -O2 -o gps_fpga gps_fpga.cpp -llimesuiteng -lm -lpthread
```

## Software vs FPGA pipeline comparison

The software receiver (`gps_rx.c`) processes raw I/Q samples entirely on the CPU.
The FPGA receiver splits the work: sample-rate correlation runs on the FPGA
(`gps_correlator.v`), while epoch-rate control, nav data, and PVT run on the
host CPU (`gps_fpga.cpp`).

This split is the classical GPS receiver architecture and is well-suited to a
low-power flight processor (e.g. SiFive Efficient E1) that cannot sustain
sample-rate processing at 4 MHz but easily handles the 1 kHz epoch-rate control
loop.

### Acquisition

| Parameter | Software (`gps_rx.c`) | FPGA (`gps_fpga.cpp` + RTL) |
|---|---|---|
| Method | FFT-based circular correlation | FFT-based circular correlation (4-bit pipelined, dblclockfft) |
| Integration | 40 ms coherent (FFT across 40 epochs) | 20 × 1 ms non-coherent (sum \|I\|+\|Q\|) |
| Code Doppler compensation | Yes (shifts FFT bins per-ms) | No (peak smears at high Doppler) |
| Doppler range | ±40 kHz, 250 Hz step | ±40 kHz, two-tier (1 kHz coarse + 250 Hz fine) |
| Detection metric | Peak-to-second-peak ratio ≥ 1.5 | Absolute power > threshold |
| Doppler refinement | 400 ms fine search (±150 Hz, 2 Hz step) | Two-tier: coarse 1 kHz + fine 250 Hz |
| Cold start time | ~2 s (all 32 PRNs) | ~2 min (two-tier: coarse + fine) |

### Tracking — carrier loop

| Parameter | Software | FPGA |
|---|---|---|
| Pull-in strategy | FLL first (400 ms), then PLL | FLL first (400 epochs), then PLL |
| FLL | Cross/dot discriminator, gain=0.05 | Cross/dot discriminator, gain configurable (default 800) |
| PLL discriminator | atan2-based Costas | Normalized Costas (QP×sign(IP)/\|IP\|) |
| PLL bandwidth | BN=15 Hz, ζ=0.707, 2nd order | BN≈15 Hz, 2nd order (fixed-point) |
| Lock detection | Σ\|IP\| ≥ 2×Σ\|QP\| over 200 ms, 3 retries | Σ\|IP\| ≥ 2×Σ\|QP\| over 200 ms |

### Tracking — code loop

| Parameter | Software | FPGA |
|---|---|---|
| DLL discriminator | Non-coherent (Em-Lm)/(Em+Lm) | Same |
| E-L spacing | ±0.5 chip | ±1 chip (wider; noisier but more tolerant during pull-in) |
| Carrier-aiding of code | Yes (`CODE_RATE × (1 + dop/F_L1)`) | Yes (carrier delta / 1540 via arithmetic shift) |

### Nav data demodulation

| Parameter | Software | FPGA |
|---|---|---|
| Bit boundary | Histogram of sign changes mod 20 | Fixed 20-epoch accumulation |
| Bit decision | Sum of 20 raw IP values (double) | Majority vote of 20 IP signs (±1) |
| Timing reference | Majority vote across all subframes | First valid subframe |

### Position solution

| Parameter | Software | FPGA |
|---|---|---|
| Algorithm | LSQ with Sagnac rotation | Same |
| Pseudorange resolution | Sub-ms (epoch position interpolation) | 1 ms (FPGA global epoch counter) |
| Fix rate | 1 Hz | 1 Hz |

## Open issues

Six gaps between the FPGA and software implementations, ranked by severity.
All six issues are resolved.

### Issue 1: No frequency-locked loop (FLL) — RESOLVED

**Problem.** The FPGA had no FLL for initial carrier pull-in.

**Fix.** Added FLL to the FPGA tracking loop in `gps_correlator.v` (v6). The
loop filter now runs in FLL mode for the first `FLL_EPOCHS` (default 400) epochs
per channel, using a cross/dot frequency discriminator, then transitions to PLL.
Per-channel `prev_ip`/`prev_qp` registers, `fll_count` counter, and `have_prev`
flag track the FLL state. FLL gain is configurable via channel 12 register 3.
Scan pipeline expanded from 46 to 52 cycles per channel.

### Issue 2: No carrier-aiding of code — RESOLVED

**Problem.** The code NCO ran at a fixed nominal rate.

**Fix.** Added carrier-aiding in the FPGA tracking loop. At writeback, the code
NCO is set to `CODE_FREQ_NOM + (carrier_freq - carrier_freq_at_track_start) / 1540`
using an arithmetic shift approximation (/1536 = >>10, 0.26% error). The base
carrier frequency is captured per channel when tracking is enabled (`car_f_base`).
No host-side changes needed.

### Issue 3: Pseudorange resolution is 20 ms — RESOLVED

**Problem.** Pseudorange timing was based on `bit_count × 0.02 s` (nav bit rate),
giving ~6 km resolution. The software version interpolates to sub-ms using
per-epoch sample positions.

**Fix.** PVT now uses the FPGA's 32-bit global epoch counter (`REG_EPOCH_CNT`,
channel 15 register 1) for 1 ms timing resolution (~300 km, absorbed by clock
bias for differential accuracy). When tracking starts, `epoch_start` is
recorded. When the nav message establishes t0, `epoch_at_t0` is computed as
`epoch_start + bit_ref × 20`. Transit time is then
`t0 + (epoch_now - epoch_at_t0) × 0.001`. The global epoch counter is read
once per PVT cycle so all channels share the same time reference.

### Issue 4: Cold acquisition takes ~57 minutes — RESOLVED

**Problem.** Serial search: 32 PRNs × 161 Doppler bins × ~2.6 s per trial.

**Fix.** Two-tier acquisition in `gps_fpga.cpp`, now accelerated by the FFT
acquisition engine (`gps_fft_acq.v`, v7):

- **Tier 1 (coarse):** 20 epochs (~13 dB gain, within 3 dB of software),
  1000 Hz step, ±40 kHz → 81 bins/PRN, all 32 PRNs. ~26 ms per bin (FFT).
  Low threshold (100) catches weak signals; false positives rejected by tier 2.
- **Tier 2 (fine):** 40 epochs (~16 dB, matches software), 250 Hz step,
  ±500 Hz around each coarse hit → 5 bins per detected PRN only. ~53 ms/bin.

Estimated cold start: 32 × 81 × 0.026 + ~8 × 5 × 0.053 ≈ **~67 s** (tier 1)
+ **~2 s** (tier 2) ≈ **~1.2 min total**. The FFT engine tests all 4096 code
phases simultaneously (vs 20-lane serial), giving ~50× speedup per trial.
This also resolves issue 5 (code Doppler) since the coarse tier uses only
20 epochs.

### Issue 5: No code Doppler compensation during acquisition — RESOLVED

**Problem.** The acquisition engine does not compensate for code-phase drift
across epochs. At ±40 kHz Doppler and 40 epochs (40 ms), code drifts ~1.1
chips, badly smearing the correlation peak.

**Fix.** Resolved by the two-tier search (issue 4). The coarse tier uses only
10 epochs at ±40 kHz, limiting drift to 0.27 chips (< 1 dB loss). The fine
tier uses 40 epochs but only at ±500 Hz around detected Doppler, where drift
is negligible (0.013 chips).

### Issue 6: Doppler search range too narrow — RESOLVED

**Problem.** ±10 kHz did not cover typical LimeSDR Mini v2 clock offsets.

**Fix.** Changed `ACQ_DOPP_MAX` to 40000 in `gps_fpga.cpp`. Combined with
almanac seeding (issue 4), the wider range adds negligible time per PRN.
