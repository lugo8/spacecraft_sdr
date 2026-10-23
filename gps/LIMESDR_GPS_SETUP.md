# LimeSDR Mini v2.0 GPS Reception Setup

## Hardware

- **SDR**: LimeSDR Mini v2.0 (serial: 1DBB8106DEEE92)
- **Antenna**: Taoglas CGGBP.18.4.A.02 — passive ceramic patch GPS/GLONASS/Galileo/BeiDou antenna, 2.0 dBi gain at GPS L1, 50 ohm, SMA connector
- **Connection**: Antenna SMA → LimeSDR RX SMA port (the port closest to the USB connector)
- **USB**: Requires USB 3.0 port and cable. The LimeSDR Mini v2 uses an FTDI FT601 USB 3.0 bridge chip.

### Antenna Placement

Same as HackRF — the Taoglas is a patch antenna. Place it **outdoors with the patch facing the sky** and an unobstructed view of the horizon. GPS L1 signals are ~-130 dBm and cannot be received indoors with a passive antenna.

## LimeSDR Mini v2 RF Architecture

The LimeSDR Mini v2 has two RX LNA paths controlled by an RF switch:

| Path  | Frequency Range   | Use Case       |
|-------|-------------------|----------------|
| LNAW  | 700 MHz – 2.1 GHz | **GPS L1 (1575 MHz)** — correct path |
| LNAH  | 2.1 GHz – 3.5 GHz | Higher frequencies — poor gain at 1575 MHz |

**The correct antenna for GPS is LNAW.** LNAH has significantly less gain at 1575 MHz and will result in failed acquisition (all peak ratios ~1.0–1.3).

### Gateware RF Switch Bug (v2.2)

The factory gateware (v2.2) has a bug: **the RF switch always routes the RX SMA to LNAH**, regardless of what the software selects. This means even if you set `antenna=LNAW`, GPS signal still goes through the wrong amplifier.

- **Gateware v3.12** fixes this bug — LNAW is properly connected when selected.
- Confirmed by measurement: LNAW std=75.7 vs LNAH std=65.9 at 1575 MHz with gateware v3.12.

### Gateware Locations

| Version | File | Size |
|---------|------|------|
| v2.2 (factory) | `%APPDATA%\LimeSuite\images\23.11\LimeSDR-Mini_HW_2.0_r2.2.bit` | 1,074,354 bytes |
| v3.12 (RF fix) | Downloaded from [Lime Micro releases](https://github.com/myriadrf/LimeSDR-Mini-v2_GW/releases) | 663,350 bytes |

### Flashing Gateware

```
LimeUtil --update --fpga=<path_to_bitstream.bit>
```

**Warning**: Gateware v3.12 is **not compatible** with old LimeSuite (v23.11.0). It requires LimeSuiteNG. See compatibility matrix below.

## Software Compatibility Matrix

| Gateware | Software | OS | Streaming | GPS Viable? | Notes |
|----------|----------|-----|-----------|-------------|-------|
| v2.2 | LimeSuite v23.11.0 | Windows | Full speed (~32 MB/s) | No — LNAH only (RF switch bug) | Signal too weak for acquisition |
| v3.12 | LimeSuite v23.11.0 | Windows | ~100x too slow | No | USB protocol mismatch between old software and new gateware |
| v3.12 | LimeSuiteNG (develop) | Windows | Zero packets | No | Known FT601 streaming bug on Windows |
| v3.12 | LimeSuiteNG (develop) | Linux (native) | Full speed | **Yes** | Uses libusb, not FT601 D3XX |
| v3.12 | LimeSuiteNG (develop) | WSL2 + usbipd | ~1% throughput | No | USB/IP protocol too slow for bulk transfers |

**Bottom line**: As of October 2026, the only working full-speed configuration for GPS is **gateware v3.12 + LimeSuiteNG on native Linux**.

## Windows Driver

The LimeSDR Mini v2 uses the standard **FTDI D3XX** driver on Windows:
- Driver: `ftdibus3` + `FTD3XX.dll`
- **Do NOT use Zadig/WinUSB** — that replaces the FTDI driver and breaks LimeSuite

## Software Prerequisites

### Radioconda (Windows — for GNURadio pipeline)

Install [Radioconda](https://github.com/ryanvolz/radioconda), which bundles GNURadio, gr-osmosdr, Python, NumPy.

Radioconda PATH setup for command-line use:
```powershell
$env:PATH = "$env:USERPROFILE\radioconda;$env:USERPROFILE\radioconda\Library\mingw-w64\bin;$env:USERPROFILE\radioconda\Library\usr\bin;$env:USERPROFILE\radioconda\Library\bin;$env:USERPROFILE\radioconda\Scripts;$env:USERPROFILE\radioconda\bin;$env:PATH"
```

### LimeSuiteNG (required for gateware v3.12)

Old LimeSuite (bundled with Radioconda as `LMS7Support.dll`) does NOT work with gateware v3.12. You must build LimeSuiteNG from source.

#### Building on Linux (recommended)

```bash
sudo apt-get install -y cmake libusb-1.0-0-dev g++ make

git clone https://github.com/myriadrf/LimeSuiteNG.git
cd LimeSuiteNG
git checkout develop

cmake -B build -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_GUI=OFF -DBUILD_DOCS=OFF \
  -DENABLE_UNIT_TESTS=OFF \
  -DLIMESUITENG_PCIE=OFF -DBUILD_DRIVERS_LIMEPCIE=OFF

cmake --build build -j$(nproc)
sudo make -C build install
sudo ldconfig
```

#### Building on Windows (MSVC — streaming broken as of Oct 2026)

```powershell
# From VS 2022 Developer Command Prompt
cmake -S . -B build -G "NMake Makefiles" ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DBUILD_GUI=OFF -DBUILD_DOCS=OFF -DENABLE_UNIT_TESTS=OFF ^
  -DSoapySDR_DIR=%USERPROFILE%\radioconda\Library\lib\cmake\SoapySDR ^
  -DCMAKE_INSTALL_PREFIX=%USERPROFILE%\radioconda\Library ^
  -DFTD3XX_STATIC=OFF

nmake
nmake install
```

To use LimeSuiteNG as the SoapySDR plugin (replacing old LimeSuite):
```powershell
# Disable old plugin
ren %USERPROFILE%\radioconda\Library\lib\SoapySDR\modules0.8\LMS7Support.dll LMS7Support.dll.bak
# LimeSuiteNG installs limesuitengSupport.dll automatically
```

## Gain Settings

The LMS7002M transceiver in the LimeSDR Mini v2 has three RX gain stages:

| Stage | Parameter | Range | GPS Setting | osmosdr Mapping |
|-------|-----------|-------|-------------|-----------------|
| LNA   | Low Noise Amplifier | 0–30 dB | 30 dB | `gain` / `set_gain()` |
| TIA   | Trans-Impedance Amplifier | 0–12 dB | 12 dB | `if_gain` / `set_if_gain()` |
| PGA   | Programmable Gain Amplifier | -12–19 dB | 19 dB | `bb_gain` / `set_bb_gain()` |

These are maxed out for GPS because the passive antenna provides very weak signal.

## Capture Methods

### Method 1: GNURadio (Windows — only with old gateware v2.2)

Open `gps/gps_recv_lime.grc` in GNURadio Companion:

| Parameter     | Value                  |
|---------------|------------------------|
| Device args   | `driver=lime,soapy=0`  |
| Sample Rate   | 4.096 MS/s             |
| Frequency     | 1575.42 MHz            |
| RF Gain       | 30 dB (LNA)            |
| IF Gain       | 12 dB (TIA)            |
| BB Gain       | 19 dB (PGA)            |
| DC Offset     | 2 (Auto)               |
| Bandwidth     | 5 MHz                  |
| Antenna       | LNAW                   |

Or use the headless script:
```powershell
$env:USERPROFILE\radioconda\python.exe gps\gps_recv.py --device "driver=lime,soapy=0" --antenna LNAW --seconds 90 --output gps_captured.iq
```

### Method 2: LimeSuiteNG C++ capture (Linux — for gateware v3.12)

A custom capture program (`gps/gps_capture.cpp`) uses the LimeSuiteNG API directly:

```bash
g++ -O2 -std=c++17 gps/gps_capture.cpp -o gps_capture \
  -I/usr/local/include -L/usr/local/lib -llimesuiteng -Wl,-rpath,/usr/local/lib

sudo ./gps_capture /tmp/gps_capture.bin 30
```

The program configures: LNAW antenna, 1575.42 MHz, 4.096 MS/s, LNA=30 TIA=12 PGA=19, and writes raw complex float32 IQ to file.

**Note**: `sudo` is required unless udev rules are installed. To install udev rules:
```bash
sudo cp ~/LimeSuiteNG/udev-rules/64-limesuite.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Method 3: WSL2 + usbipd (not viable for GPS)

USB passthrough via usbipd-win works for device detection and low-rate control, but bulk streaming throughput is ~1% of native — not enough for continuous GPS capture.

Setup (for reference):
```powershell
# Windows (admin PowerShell)
usbipd bind -b <BUSID>
usbipd attach --wsl -b <BUSID>

# WSL must be running first:
Start-Process wsl -ArgumentList "-d Ubuntu -- bash -c 'sleep 3600'" -WindowStyle Hidden
```

## GPS Processing Pipeline

After capturing IQ data (by any method), run the GPS software receiver:

```
gps\gps_rx.exe [--tail] <iq_file>
```

- **File mode**: Process a completed capture file
- **Tail mode**: Follow a growing file in real-time (for streaming capture)

The IQ file format is complex float32 (interleaved I/Q, 8 bytes per sample) at 4.096 MS/s.

### Acquisition Parameters

| Parameter    | Value | Notes |
|--------------|-------|-------|
| `ACQ_MS`     | 20    | Coherent integration time (ms) |
| `ACQ_THRESH` | 1.8   | Peak/second-peak ratio threshold |
| `FS`         | 4.096e6 | Sample rate (Hz) |

## Troubleshooting

### No device found
- Check USB 3.0 connection (blue port)
- On Windows: verify FTDI D3XX driver in Device Manager (NOT WinUSB/Zadig)
- On Linux: check `lsusb` for `0403:601f` and USB permissions

### Streaming too slow (~100x)
- Gateware v3.12 + old LimeSuite = protocol mismatch. Must use LimeSuiteNG.
- WSL2 + usbipd = USB/IP overhead. Must use native Linux.

### Zero packets on Windows
- LimeSuiteNG FT601 streaming bug. Known issue — no fix as of Oct 2026.

### 0 satellites acquired (all ratios ~1.0)
- **Check antenna path**: gateware v2.2 forces LNAH regardless of software setting. Flash v3.12 or accept weak signal.
- **Check antenna placement**: must be outdoors, patch facing sky, SMA tight.
- **Try higher gains**: max out LNA=30, TIA=12, PGA=19.
- **Try longer integration**: increase `ACQ_MS` to 40 in `gps_rx.c` and lower `ACQ_THRESH` to 1.5.

### Device detected but "permission denied" (Linux)
- Run with `sudo`, or install udev rules (see capture method 2).

## Current Status (October 2026)

**GPS reception with LimeSDR Mini v2 on Windows is currently blocked** by the intersection of two bugs:
1. Gateware v2.2 RF switch bug routes GPS to wrong antenna (LNAH)
2. LimeSuiteNG (needed for gateware v3.12 fix) has a Windows FT601 streaming bug

**Working path**: Native Linux + gateware v3.12 + LimeSuiteNG — confirmed streaming at correct rate with LNAW antenna. GPS acquisition testing pending (need outdoor capture on native Linux).

**Recommended next step**: Boot Linux natively (or dual-boot) and run the capture pipeline with gateware v3.12 + LimeSuiteNG.
