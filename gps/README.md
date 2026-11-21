# gps/

GPS L1 C/A receiver — software receiver, FPGA correlator RTL, and host controller.

## Files

### Host software

| File | Description |
|------|-------------|
| `gps_rx.c` | Standalone software GPS receiver. Processes raw I/Q samples (file or stream) entirely on the CPU. FFT-based acquisition, software tracking loops, nav data decode, PVT. |
| `gps_fpga.c` | Host controller for the FPGA correlator. Drives acquisition, manages tracking channels, demodulates nav data, computes position fixes. Talks to the FPGA via LiteX CSR registers through `limeCSR`. |

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
gcc -O2 -o gps_fpga gps_fpga.c -lm
```

## Architecture: software vs FPGA receiver

| | `gps_rx` (software) | `gps_fpga` (FPGA host) |
|-|---------------------|------------------------|
| Correlation | CPU (FFT-based) | FPGA (matched-filter, 20 parallel lanes) |
| Acquisition | CPU, ~20 ms/PRN | FPGA engine, ~6 ms/PRN + USB overhead |
| Tracking loop | CPU, processes raw samples | FPGA, closed-loop at 1 kHz epoch rate |
| Discriminators | Normalized (atan2, (E-L)/(E+L)) | Normalized (QP/\|IP\|, (E-L)/(E+L)) via sequential divider |
| Nav data | CPU, from raw IP accumulator | Pi, polls IP register over USB |
| Position fix | CPU | Pi |
| Input | Raw I/Q samples (file or pipe) | LiteX CSR registers via USB |
