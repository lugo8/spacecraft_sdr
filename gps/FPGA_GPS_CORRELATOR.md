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

See [`rtl/README.md`](rtl/README.md) for module hierarchy, register map, and tracking loop details.

### Documentation

| File | Description |
|------|-------------|
| `LIMESDR_GPS_SETUP.md` | LimeSDR Mini v2 GPS test setup and results |
| `HACKRF_GPS_SETUP.md` | HackRF GPS capture setup |

## Build

```bash
# Software receiver (runs anywhere)
gcc -O3 -march=native -o gps_rx gps_rx.c -lm

# FPGA host controller (runs on Pi with LimeSDR)
g++ -O2 -o gps_fpga gps_fpga.cpp -llimesuiteng -lm -lpthread

# FPGA gateware (on Pi, in LimeSDR_GW repo)
source .venv/bin/activate
PYTHONPATH=. python3 boards/targets/limesdr_mini_v2.py --build --with-gps --no-ppsdo
```

## Target FPGA

Lattice ECP5 LFE5U-45F-8MG285C on the LimeSDR Mini v2.

| Resource | Available |
|----------|-----------|
| LUT4 | 44,340 |
| EBR (DP16KD) | 108 |
| DSP (MULT18X18D) | 72 |
| PLLs | 4 |

## Architecture

The software receiver (`gps_rx.c`) processes raw I/Q samples entirely on the CPU.
The FPGA receiver splits the work: sample-rate correlation runs on the FPGA
(`gps_correlator.v`), while epoch-rate control, nav data, and PVT run on the
host CPU (`gps_fpga.cpp`).

```
LMS7002M I/Q (12-bit, 4.096 MS/s)
    |
    +--[passive tap]--→ gps_correlator (lms_rx clock domain)
    |                       ├── 8× gps_channel (carrier/code NCO, E/P/L correlators)
    |                       ├── gps_fft_acq (4096-pt FFT acquisition engine)
    |                       └── tracking loop filter (FLL → PLL + DLL, on-FPGA)
    |                               |
    |                          CSR via USB (LiteX / LimeSuiteNG ICSR)
    |                               |
    |                          gps_fpga.cpp on Raspberry Pi
    |                          (acquisition control, nav decode, PVT)
    |
    +--→ RXPathTop → FT601 USB  (unchanged)
```

The correlator snoops the RX stream without touching `ready` — USB streaming
is unaffected. All RTL runs in the `lms_rx` domain. CDC to `cd_sys` (77.5 MHz)
is handled by `GPSTop.py` using `MultiReg` and `PulseSynchronizer`.

## Software vs FPGA comparison

### Acquisition

| Parameter | Software (`gps_rx.c`) | FPGA (`gps_fpga.cpp` + RTL) |
|---|---|---|
| Method | FFT-based circular correlation | FFT-based circular correlation (4-bit pipelined, dblclockfft) |
| Integration | 40 ms coherent (FFT across 40 epochs) | 20 × 1 ms non-coherent (sum \|I\|+\|Q\|) |
| Code Doppler compensation | Yes (shifts FFT bins per-ms) | No (mitigated by two-tier search) |
| Doppler range | ±40 kHz, 250 Hz step | ±40 kHz, two-tier (1 kHz coarse + 250 Hz fine) |
| Detection metric | Peak-to-second-peak ratio ≥ 1.5 | Absolute power > threshold |
| Cold start time | ~2 s (all 32 PRNs) | ~1.2 min (two-tier: coarse + fine) |

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
