# gps/rtl/

Verilog RTL for the GPS L1 C/A correlator, targeting the Lattice ECP5 on the LimeSDR Mini v2.

## Module hierarchy

```
gps_correlator          Top-level: 12-channel correlator + acquisition + tracking loop
├── gps_channel ×12     Per-channel E/P/L accumulate-and-dump with carrier/code NCOs
│   └── gps_ca_gen      Gold code generator (G1/G2 LFSRs, all 32 PRN tap selections)
└── gps_acq_engine      Matched-filter acquisition (20 parallel correlation lanes)
    └── gps_ca_gen      Code generator instance for acquisition
```

## Files

| File | Lines | Description |
|------|-------|-------------|
| `gps_correlator.v` | ~450 | Top-level. Register map, 12 channel instances, matched-filter acquisition engine, normalized-discriminator tracking loop filter with sequential divider. Version 5 (`0x47505305`). |
| `gps_channel.v` | ~150 | Single correlator channel. Carrier wipe-off (CORDIC-free, sign/negate), code NCO, early/prompt/late code mixing, accumulate-and-dump with output registers. |
| `gps_ca_gen.v` | ~100 | C/A Gold code generator. Two 10-bit LFSRs (G1, G2), 32 PRN tap selections via parameter. Generates ±1 code chips at the code NCO rate. |
| `gps_acq_engine.v` | ~280 | Matched-filter acquisition. Captures one epoch (4096 samples) into block RAM after carrier wipe-off, then correlates all 1023 code phases using a 1042-bit shift register with 20 parallel output taps. Reports best code phase and magnitude. |
| `gps_spi_slave.v` | — | SPI slave interface (phase 2, flight computer access — not yet integrated). |
| `gps_top.v` | — | Standalone top-level wrapper (for simulation/testing outside LiteX). |

## Register map

7-bit address: `{channel[3:0], register[2:0]}`

### Per-channel (ch 0–11)

| Reg | R/W | Name | Description |
|-----|-----|------|-------------|
| 0 | RW | CTRL | `{25'b0, track, enable, prn[4:0]}` |
| 1 | RW | CAR_FREQ | Carrier NCO frequency word (auto-updated when track=1) |
| 2 | RW | COD_FREQ | Code NCO frequency word (auto-updated when track=1) |
| 3 | W | COD_PHASE | `{22'b0, chip_phase[9:0]}` — preset code phase |
| 3 | R | IP | I-Prompt accumulator (32-bit signed) |
| 4 | R | QP | Q-Prompt accumulator (32-bit signed) |
| 5 | R | IE_QE | `{ie[31:16], qe[31:16]}` |
| 6 | R | IL_QL | `{il[31:16], ql[31:16]}` |
| 7 | R | STATUS | `{14'b0, busy, epoch_flag, epoch_count[15:0]}` |

### Channel 12 — loop filter gains

| Reg | R/W | Name | Description |
|-----|-----|------|-------------|
| 0 | RW | CAR_GAIN_P | Carrier PLL proportional gain (signed 32-bit) |
| 1 | RW | CAR_GAIN_I | Carrier PLL integral gain (signed 32-bit) |
| 2 | RW | COD_GAIN | Code DLL gain (signed 32-bit) |

### Channel 15 — global / acquisition

| Reg | R/W | Name | Description |
|-----|-----|------|-------------|
| 0 | RW | GLOB_CTRL | `{31'b0, enable}` |
| 1 | R | EPOCH_CNT | Global epoch counter |
| 2 | R | VERSION | `0x47505305` ("GPS" + v5) |
| 3 | W | ACQ_CTRL | bit 0: start, bit 1: abort |
| 3 | R | ACQ_STATUS | `{done, active, 20'b0, best_phase[9:0]}` |
| 4 | RW | ACQ_CFG | `{11'b0, prn[4:0], 16'b0}` |
| 5 | RW | ACQ_CARRIER | Carrier NCO freq word for acquisition |
| 6 | R | ACQ_POWER | Best |IP|+|QP| magnitude |

## Tracking loop filter

Runs entirely on-FPGA at the 1 kHz epoch rate (no USB in the loop).

**Discriminators** (normalized, amplitude-independent):
- PLL: `QP × sign(IP) / |IP|` — approximates atan(phase_error) near lock
- DLL: `(|IE|+|QE| - |IL|-|QL|) / (|IE|+|QE| + |IL|+|QL|)` — bounded [-1, 1]

**Normalization**: 20-cycle sequential restoring divider, Q4.16 fixed-point output.

**Pipeline** (46 sub-cycles per channel, 552 cycles per full 12-channel scan):

| Sub-cycle | Action |
|-----------|--------|
| 0 | Setup PLL division |
| 1–20 | PLL division (20 iterations) |
| 21 | Latch PLL result, setup DLL division |
| 22–41 | DLL division (20 iterations) |
| 42 | Latch DLL result, start carrier-P multiply |
| 43 | Save car_P, start carrier-I multiply |
| 44 | Save car_I, start code multiply |
| 45 | Writeback NCO updates, advance channel |

**Resource usage** (estimated):
- 552 / 4096 = 13.5% of epoch budget
- 4 DSP slices (shared 32×32 multiplier)
- 12 × 32-bit integrator registers
- Divider: ~100 LUTs (combinational compare + subtract)

## Clock domain

All RTL runs in the `lms_rx` domain (driven by LMS7002M MCLK1 at the sample rate). CDC to the `cd_sys` domain (77.5 MHz) is handled by `GPSTop.py` using `MultiReg` and `PulseSynchronizer`.

## Build

The RTL is integrated into the LimeSDR gateware via `LimeSDR_GW/gateware/GPSTop.py`. The build directory `LimeSDR_GW/gateware/gps_rtl/` is a directory junction pointing here, so edits to these files are automatically visible to the build. See the top-level README for build instructions.
