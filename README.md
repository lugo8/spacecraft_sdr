# spacecraft_sdr

GPS L1 C/A reception pipeline for a cubesat, built around a **LimeSDR Mini v2.0** connected to a **Raspberry Pi 4**.

The system splits the GPS workload between FPGA and Pi:
- **FPGA** (Lattice ECP5): correlation, matched-filter acquisition, tracking loop closure — all at the 4.096 MHz sample rate with zero USB latency
- **Pi**: channel management, nav data demodulation, ephemeris parsing, pseudorange computation, position fix (least squares)

This reduces FPGA-to-Pi USB bandwidth from ~12 MB/s (raw I/Q) to ~17 KB/s (register polls).

## Repository structure

```
gps/                  GPS receiver — software, RTL, and host controller
LimeSDR_GW/           LimeSDR gateware (Lime Microsystems fork with GPS integration)
argus2_docs/          Argus-2 cubesat comms subsystem documentation (PDFs)
*.iq                  Captured GPS L1 test signals
```

## Hardware

| Component | Details |
|-----------|---------|
| SDR | LimeSDR Mini v2.0 (ECP5 LFE5U-45F, LMS7002M) |
| Host | Raspberry Pi 4 (SSH: `comms@comms`, password `sdr`) |
| Antenna | GPS L1 patch antenna, 1575.42 MHz |
| Interface | USB 3.0 (FT601), 4.096 MS/s complex, 12-bit I/Q |

## Quick start

```bash
# 1. Build gateware on the Pi
cd ~/LimeSDR_GW
source .venv/bin/activate
python3 boards/targets/limesdr_mini_v2.py --build --with-gps

# 2. Flash bitstream
# (use LimeSuite or openFPGALoader)

# 3. Configure LimeSDR RX at 1575.42 MHz, 4.096 MS/s

# 4. Run host controller
cd ~/spacecraft_sdr/gps
gcc -O2 -o gps_fpga gps_fpga.c -lm
./gps_fpga --csr-csv ~/LimeSDR_GW/build/csr.csv
```

## Software-only receiver

The standalone software receiver processes I/Q files or live streams without FPGA gateware:

```bash
cd gps
gcc -O3 -march=native -o gps_rx gps_rx.c -lm
./gps_rx capture.iq                 # file mode
mkfifo /tmp/gps_iq && ./gps_rx < /tmp/gps_iq  # streaming mode
```
