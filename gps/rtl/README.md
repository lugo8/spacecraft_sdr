# gps/rtl/

Verilog RTL for the GPS L1 C/A correlator, targeting the Lattice ECP5 on the LimeSDR Mini v2.

## Module hierarchy

```
gps_correlator          Top-level: 8-channel correlator + acquisition + tracking loop
├── gps_channel ×8      Per-channel E/P/L accumulate-and-dump with carrier/code NCOs
│   └── gps_ca_gen      Gold code generator (G1/G2 LFSRs, all 32 PRN tap selections)
└── gps_fft_acq         FFT-based acquisition (4096-pt pipelined FFT, all code phases)
    ├── gps_ca_gen      Code generator instance for acquisition
    └── fftmain         4-bit 4096-point pipelined FFT (dblclockfft, in fft/ subdir)
        ├── fftstage ×10   Butterfly stages with twiddle ROMs
        ├── qtrstage       Quarter-stage (N=4)
        └── laststage      Final radix-2 stage (N=2)
```

## Files

| File | Description |
|------|-------------|
| `gps_correlator.v` | Top-level. Register map, 8 channel instances, FFT acquisition engine, FLL/PLL/DLL tracking loop with sequential divider, carrier-aiding. Version 7 (`0x47505307`). |
| `gps_channel.v` | Single correlator channel. Carrier wipe-off (sign/negate), code NCO, early/prompt/late code mixing, accumulate-and-dump with output registers. |
| `gps_ca_gen.v` | C/A Gold code generator. Two 10-bit LFSRs (G1, G2), 32 PRN tap selections via parameter. |
| `gps_fft_acq.v` | FFT-based acquisition. 3 FFT passes per epoch: forward FFT of signal, forward FFT of code with complex multiply, inverse FFT (conjugate trick) with magnitude accumulation. All 4096 code phases tested simultaneously. |
| `gps_acq_engine.v` | (Legacy) Time-domain matched-filter acquisition with parallel lanes. Not instantiated in v7. |
| `fft/*.v, fft/*.hex` | 4-bit 4096-point pipelined FFT from [dblclockfft](https://github.com/ZipCPU/dblclockfft). Generated with `fftgen -f 4096 -n 4 -p 24 -s`. |
| `gps_spi_slave.v` | SPI slave interface (phase 2, flight computer access — not yet integrated). |
| `gps_top.v` | Standalone top-level wrapper (for simulation/testing outside LiteX). |

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
| 3 | RW | FLL_GAIN | FLL frequency gain (signed 32-bit) |

### Channel 13 — FLL config

| Reg | R/W | Name | Description |
|-----|-----|------|-------------|
| 0 | RW | FLL_EPOCHS | Number of epochs in FLL mode before switching to PLL (default 400) |

### Channel 15 — global / acquisition

| Reg | R/W | Name | Description |
|-----|-----|------|-------------|
| 0 | RW | GLOB_CTRL | `{31'b0, enable}` |
| 1 | R | EPOCH_CNT | Global epoch counter |
| 2 | R | VERSION | `0x47505307` ("GPS" + v7) |
| 3 | W | ACQ_CTRL | bit 0: start, bit 1: abort |
| 3 | R | ACQ_STATUS | `{done, active, 20'b0, best_phase[9:0]}` |
| 4 | RW | ACQ_CFG | `{5'b0, prn[4:0], 10'b0, num_epochs[5:0]}` |
| 5 | RW | ACQ_CARRIER | Carrier NCO freq word for acquisition |
| 6 | R | ACQ_POWER | Best |IP|+|QP| magnitude (accumulated over num_epochs) |

## Tracking loop filter

Runs entirely on-FPGA at the 1 kHz epoch rate (no USB in the loop).

**FLL** (first `FLL_EPOCHS` after tracking enable, cross/dot discriminator):
- `cross = prev_IP × QP - prev_QP × IP`
- `dot = prev_IP × IP + prev_QP × QP`
- `freq_err = cross × sign(dot)`
- `carrier_freq += (freq_err × FLL_GAIN) >> 16`

**PLL** (after FLL, normalized Costas):
- `norm_phase = (QP × sign(IP)) / |IP|`
- `carrier_freq += (norm_phase × CAR_GAIN_P + integral × CAR_GAIN_I) >> 16`

**DLL** (normalized early-minus-late):
- `norm_code = (|IE|+|QE| - |IL|-|QL|) / (|IE|+|QE| + |IL|+|QL|)`
- `code_freq -= (norm_code × COD_GAIN) >> 16`

**Carrier-aiding**: `code_freq = CODE_FREQ_NOM + (carrier_freq - car_f_base) / 1540`
(approximated as >>10, 0.26% error).

**Normalization**: 20-cycle sequential restoring divider, Q4.16 fixed-point output.

**Pipeline** (52 sub-cycles per channel, 624 cycles per full 12-channel scan):

| Sub-cycle | Action |
|-----------|--------|
| 0 | Setup PLL division |
| 1–20 | PLL division (20 iterations) |
| 21 | Latch PLL result, setup DLL division |
| 22–41 | DLL division (20 iterations) |
| 42 | Latch DLL result, start FLL cross multiply (prev_IP × QP) |
| 43 | Latch cross1, start prev_QP × IP |
| 44 | Compute cross, start FLL dot multiply (prev_IP × IP) |
| 45 | Latch dot1, start prev_QP × QP |
| 46 | Compute dot, start carrier gain multiply (FLL or PLL) |
| 47 | Save carrier term, start integral multiply (PLL) |
| 48 | Save integral term, start code DLL multiply |
| 49 | Code term ready |
| 50 | (available) |
| 51 | Writeback NCO updates, advance channel |

**Resource usage** (estimated):
- 624 / 4096 = 15% of epoch budget
- 4 DSP slices (shared 32×32 multiplier)
- 12 × 32-bit integrator + prev_ip/prev_qp/fll_count/car_f_base registers
- Divider: ~100 LUTs (combinational compare + subtract)

## Clock domain

All RTL runs in the `lms_rx` domain (driven by LMS7002M MCLK1 at the sample rate). CDC to the `cd_sys` domain (77.5 MHz) is handled by `GPSTop.py` using `MultiReg` and `PulseSynchronizer`.

## Build

The RTL is integrated into the LimeSDR gateware via `LimeSDR_GW/gateware/GPSTop.py`. The build directory `LimeSDR_GW/gateware/gps_rtl/` is a directory junction pointing here, so edits to these files are automatically visible to the build. See the top-level README for build instructions.
