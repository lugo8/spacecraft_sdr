# GPS Correlator FPGA Resource Analysis — LimeSDR Mini v2

## FPGA Device

**Lattice ECP5 LFE5U-45F-8MG285C** (from `limesdr_mini_v2_platform.py` in [LimeSDR_GW](https://github.com/myriadrf/LimeSDR_GW))

| Resource | Available |
|----------|-----------|
| LUT4 (logic cells) | 44,340 |
| Flip-flops | 44,340 |
| Distributed RAM (bits) | 355K |
| EBR (18 Kb block RAM) | 108 blocks (1,944 Kb = 243 KB) |
| DSP blocks (MULT18x18) | 72 |
| PLLs | 4 |
| SERDES | 0 (LFE5U, not UM) |
| Speed grade | -8 (commercial) |

Package: BGA-285, 197 user I/O.

## Existing Gateware Architecture

The v3.x gateware (LiteX/Migen-based, `develop` branch) contains:

| Module | Function |
|--------|----------|
| VexRiscV "minimal" CPU | SoC controller, firmware execution, LMS64C protocol handling |
| ROM (0x4800) + SRAM (0x2000) | CPU memory (18 KB ROM + 8 KB SRAM) |
| FT601 interface | 32-bit FTDI USB 3.0 bridge, FIFOs for control (1 KB) and stream (4 KB RX, 8 KB TX) |
| LMS7002Top | LMS7002M DDR I/Q interface (12-bit), clock management |
| RXTXTop (RXPathTop + TXPathTop) | RX/TX packet formation, timestamps, sample combiner, flow control |
| SPIMaster | SPI to LMS7002M and DAC |
| LiteI2C | I2C to EEPROM and temp sensor |
| FPGACfg | Configuration registers |
| GeneralPeriphTop | GPIO, LED, revision readback |
| TstTop | Clock test/verification |
| PPSDO | PPS disciplined oscillator (optional) |
| StreamStartController | PPS/trigger-based stream start |
| GNSSTop | NMEA sentence parser (for external GPS module, not a correlator) |

### Estimated Existing Resource Usage

No synthesis reports are checked into the repo. Estimated from module complexity (LiteX SoC with VexRiscV minimal + streaming + peripherals on ECP5):

| Resource | Estimated Usage | % of ECP5-45F |
|----------|----------------|---------------|
| LUT4 | ~12,000–15,000 | 27–34% |
| Flip-flops | ~6,000–8,000 | 14–18% |
| EBR (block RAM) | ~40–50 blocks | 37–46% |
| DSP blocks | 0–2 | 0–3% |
| PLLs | 1 (OSCG internal) | 25% |

The biggest consumers are the VexRiscV CPU (~2,500 LUTs), FT601 FIFOs (~15 EBR), TX/RX path FIFOs (~15 EBR), and CPU ROM/SRAM (~12 EBR). Logic is well under 50% but **block RAM is the tightest resource** at ~40%.

## GPS L1 C/A Correlator — Design

### Architecture: 12-Channel Parallel Correlator

Each channel tracks one satellite (PRN). Standard GPS receiver architecture:

```
LMS7002M I/Q (12-bit) --> Carrier Mixer --> Code Correlator (E/P/L) --> Accumulate & Dump
                              ^                    ^                          |
                         Carrier NCO           Code NCO + C/A Gen      1 ms accumulators
                              ^                    ^                          |
                         +----+--------------------+---------------------------
                         |              SPI Register Interface
                         +------------ Flight Computer (reads accumulators,
                                       writes NCO commands every 1 ms)
```

### Per-Channel Resource Breakdown

| Submodule | LUTs | FFs | EBR | DSP | Notes |
|-----------|------|-----|-----|-----|-------|
| **Carrier NCO** (32-bit phase accumulator) | 40 | 35 | 0 | 0 | Phase accumulator + frequency word register |
| **Carrier mixer** (sin/cos x 12-bit I/Q) | 80 | 30 | 0 | 0 | 1-bit sign from NCO MSBs x I/Q = add/subtract; or small LUT for 2-bit quadrant |
| **Code NCO** (32-bit + half-chip E/L spacing) | 50 | 40 | 0 | 0 | Generates code clock + E/P/L phase offsets |
| **C/A code generator** (2x 10-bit LFSR + tap select) | 60 | 25 | 0 | 0 | G1, G2 shift registers, PRN tap mux, epoch counter |
| **6 correlators** (E/P/L x I/Q multiply-accumulate) | 180 | 200 | 0 | 0 | 1-bit code x 12-bit I/Q = negate or pass-through; 20-bit accumulators x 6 |
| **Accumulate & dump** (latch + clear every 1 ms) | 30 | 130 | 0 | 0 | 6 x 20-bit output registers, epoch strobe |
| **Channel control** (PRN select, state, flags) | 30 | 20 | 0 | 0 | |
| **Per-channel total** | **~470** | **~480** | **0** | **0** | |

### Key Design Decisions

**No DSP blocks needed.** The C/A code is 1-bit (+/-1), so correlation is add/subtract of the I/Q sample, not a true multiply. This is the fundamental advantage of CDMA — the "multiply" is free in hardware.

**No block RAM needed per channel.** The C/A code is generated on-the-fly by two 10-bit LFSRs (Gold code), not stored in a lookup table. Accumulators are registers (flip-flops), not RAM.

**Carrier mixing simplification.** A full sin/cos lookup would need a ROM, but GPS correlation only needs the carrier stripped to within a fraction of a cycle. A 2-bit (quadrant) or 4-bit NCO output is sufficient for the mixer — this is standard in GPS ASICs and uses pure logic.

### System-Level Resources

| Module | LUTs | FFs | EBR | DSP | Notes |
|--------|------|-----|-----|-----|-------|
| **12 correlator channels** | 5,640 | 5,760 | 0 | 0 | 12 x per-channel |
| **Sample distributor** (fan-out I/Q to 12 channels) | 50 | 30 | 0 | 0 | Register + broadcast |
| **SPI slave interface** (flight computer link) | 200 | 150 | 0 | 0 | Register file: 12 ch x ~10 regs R/W |
| **Accumulator readout buffer** (double-buffered) | 300 | 1,500 | 0 | 0 | 12 ch x 6 accum x 20 bits, ping-pong |
| **Millisecond epoch timer** | 30 | 35 | 0 | 0 | Counts samples to 4096 (1 ms at 4.096 MS/s) |
| **Channel controller** (PRN assignment, enable) | 100 | 60 | 0 | 0 | |
| **GPS correlator total** | **~6,320** | **~7,535** | **0** | **0** | |

## Combined Resource Budget

| | LUTs | % of 44,340 | FFs | % of 44,340 | EBR | % of 108 | DSP | % of 72 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Existing gateware (est.) | 14,000 | 32% | 7,000 | 16% | 45 | 42% | 1 | 1% |
| GPS correlator (12 ch) | 6,320 | 14% | 7,535 | 17% | 0 | 0% | 0 | 0% |
| **Combined total** | **20,320** | **46%** | **14,535** | **33%** | **45** | **42%** | **1** | **1%** |
| **Remaining** | **24,020** | **54%** | **29,805** | **67%** | **63** | **58%** | **71** | **99%** |

## Verdict

**The GPS correlator fits comfortably.** The ECP5-45F has ample room:

- **LUTs**: 46% used (54% free = 24K LUTs remaining). Even with synthesis overhead (typically 10-20% more than hand estimates), total usage stays well under 60%.
- **Flip-flops**: 33% used. No concern.
- **Block RAM**: 42% used, and the correlator adds zero EBR. This is the tightest resource but is not affected by the correlator.
- **DSP blocks**: Essentially unused (72 available). The correlator doesn't need them, but they're available if you later want proper sin/cos carrier mixing or matched filtering.

### Margin for Growth

With 24K LUTs free after the correlator, there's room for:
- **Noise blanker / AGC** in the FPGA (~500 LUTs)
- **Code-phase measurement engine** for pseudorange output (~1K LUTs)
- **Navigation message bit extraction** (~500 LUTs per channel)
- Increasing to **24 channels** (add another ~6K LUTs, still under 60% total)

### Risk Factors

1. **Timing closure at -8 speed grade**: The correlator runs at the LMS7002M sample clock (up to 125 MHz for the DDR interface, but the actual sample processing is at 4.096 MHz). At GPS rates this is trivially easy to close timing on.

2. **Existing gateware estimate uncertainty**: Without actual synthesis reports, the existing usage could be 20-30% higher than estimated. Even at 18K LUTs existing, combined total would be ~24K (55%) — still fine.

3. **SPI interface to flight computer**: The existing gateware's SPI master talks to the LMS7002M. A new SPI *slave* port for the flight computer would need dedicated pins (4 GPIOs on J3 header are available: `FPGA_GPIO[0:7]` defined in `limesdr_mini_v2_platform.py`, plus 2 EGPIOs).

4. **Clock domain**: The correlator should run in the `lmk` (40 MHz) or `lms_rx` domain. At 4.096 MS/s the correlator processes one sample every ~10 clock cycles at 40 MHz — plenty of time for sequential channel processing if LUT savings are needed.

### Alternative: Time-Multiplexed Architecture

Instead of 12 parallel channels, process all channels sequentially using one correlator engine:

| | Parallel (12 ch) | Time-Multiplexed |
|---|---|---|
| LUTs | ~6,320 | ~1,500 |
| FFs | ~7,535 | ~2,000 |
| EBR | 0 | 2 (accumulator/state RAM) |
| Channels | 12 fixed | Up to ~40 (at 40 MHz / 4.096 MS/s) |
| Complexity | Simple, each channel independent | Needs scheduler, shared state RAM |

At 40 MHz with 4.096 MS/s input, there are ~9.7 clock cycles per sample — enough for ~8-9 channels if each takes 1 cycle, or requires pipelining for 12+. A 2-stage pipeline handles 12 easily. This uses ~1,500 LUTs (3.4%) but adds 2 EBR for state storage.
