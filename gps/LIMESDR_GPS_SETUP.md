# LimeSDR Mini v2.0 GPS Reception Setup

## Hardware

- **SDR**: LimeSDR Mini v2.0 (serial: 1DBB8106DEEE92)
- **Antenna**: Taoglas CGGBP.18.4.A.02 — passive ceramic patch GPS/GLONASS/Galileo/BeiDou antenna, 2.0 dBi gain at GPS L1, 50 ohm, SMA connector
- **Connection**: Antenna SMA → LimeSDR RX SMA port (the port closest to the USB connector)
- **USB**: The LimeSDR Mini v2 uses an FTDI FT601 USB 3.0 bridge chip. USB 3.0 streaming jams GPS reception with a passive antenna (see "USB 3.0 Self-Interference" below), and this board does not enumerate on USB 2.0.

### Antenna Placement

Same as HackRF — the Taoglas is a patch antenna. Place it **outdoors with the patch facing the sky** and an unobstructed view of the horizon. GPS L1 signals are ~-130 dBm and cannot be received indoors with a passive antenna.

**Do not screw the passive antenna directly onto the LimeSDR.** Use an SMA extension cable (1–3 m) so the patch is well away from the board, the USB cable, and the laptop. A 30 cm extension alone was not enough (see below).

### USB 3.0 Self-Interference (blocks GPS with a passive antenna)

While the LimeSDR streams over USB 3.0, it radiates broadband noise in the GPS L1 band that a nearby passive antenna picks up, raising the noise floor by ~20 dB and burying the satellites. Measured October 2026, same antenna, same outdoor spot:

| Measurement | Result |
|---|---|
| LimeSDR noise at 1575.42 MHz, antenna on vs. off | **+22 dB** (a clean passive antenna should add only a few dB) |
| HackRF (USB 2.0) noise, antenna on vs. off | +2.3 dB |
| HackRF acquisition, 60 s | **3 satellites** (PRN 8 ratio 9.1, PRN 2 5.0, PRN 7 4.3) |
| LimeSDR acquisition, 60 s | 0 satellites (best ratio 1.40) |

Using the HackRF as a noise probe next to the LimeSDR (HackRF noise at 1575.42 MHz):

| LimeSDR state | HackRF noise |
|---|---|
| Unplugged / plugged in idle | baseline (idle ≤ +1 dB) |
| RX fully configured (LO at 1575.42 MHz, ADCs running), **not streaming** | no change |
| **Streaming** (any LO: 1000 / 1575 / 2400 MHz; any rate: 2–10 MS/s) | **+12.5 dB** (amp off) to +16.5 dB (amp on), flat across the band |

The noise appears only while data is streaming over USB 3.0 and is independent of LO frequency and sample rate. This matches USB 3.0 link emissions (a continuous scrambled 5 Gbps signal while transferring), not the LMS7002M RF chain. The USB 3.0 cable and the on-board FT601 can't be told apart with these tests.

Fixes tried and remaining, roughly in order of cost:
1. **USB 2.0 — works, but unreliable to get.** USB 2.0 carries 4.096 MS/s with zero loss and removes the interference. This LimeSDR did not enumerate through a genuine USB 2.0 hub (QinHeng `1a86:8095`, 480 Mbps; board LED on, no connect attempt in the kernel log), nor with the SuperSpeed port disabled in sysfs (`usbN-portM/disable`). It **did** come up at USB 2.0 directly on the laptop port after being wrapped in aluminium foil (Oct 28), presumably because the foil disturbed the SuperSpeed contacts. `limeDevice` then reports `media=USB2.0`. Note: a small USB-A→C adapter may contain a USB 3.0 hub (check `cat /sys/bus/usb/devices/<dev>/speed`: `480` = USB 2.0, `5000` = USB 3.0).
2. **SMA extension cable — helps but not enough on its own.** A 30 cm (1 ft) extension lowered the L1 noise from −22.6 to **−34.5 dBFS (−12 dB)**, still ~10 dB above the no-antenna floor (−44.4 dBFS). The LimeSDR still acquired 0 satellites (best ratio 1.42), while the HackRF through the same extension got a full fix (see table below). Longer cables (1–3 m) should help further.
3. **Metal sheet as shield + ground plane** (untested): antenna PCB on top of a baking tray / foil-covered board, patch facing up; LimeSDR and USB cable underneath, cable leading away. The antenna's own PCB ground plane (Taoglas specifies a 70 × 70 mm plane) only shields directly below it.
4. **Ferrite chokes** on the USB cable and a **shielded enclosure** (or foil wrap) for the board (untested).
5. **Active GPS antenna + external bias tee** (the LimeSDR Mini has no bias tee). The antenna's LNA raises the signal before local noise is added — the most robust fix.

Results with the 30 cm SMA extension (October 27, 2026, same antenna and spot, 60 s captures):

| Receiver | L1 noise level | Satellites acquired | Position fix |
|---|---|---|---|
| LimeSDR (USB 3.0), antenna on board | −22.6 dBFS | 0 (Oct 23) | No |
| LimeSDR (USB 3.0), 30 cm extension | −34.5 dBFS | 0 (best ratio 1.42) | No |
| HackRF (USB 2.0), 30 cm extension | −28.4 dBFS (≈ −28.2 direct) | **5** (PRN 16, 26, 31, 3, 4; ratios 2.5–5.5) | **Yes** — 4 ephemerides, 59 fixes in 60 s |

The HackRF result confirms the extension is a proper SMA cable (not RP-SMA) with negligible loss, and that the sky view was good. Noise levels are only comparable within the same receiver.

## LimeSDR Mini v2 RF Architecture

The LimeSDR Mini v2 has two RX LNA paths controlled by an RF switch:

| Path  | Frequency Range   | Use Case       |
|-------|-------------------|----------------|
| LNAW  | 700 MHz – 2.1 GHz | **GPS L1 (1575 MHz)** — correct path |
| LNAH  | 2.1 GHz – 3.5 GHz | Higher frequencies — poor gain at 1575 MHz |

**The correct antenna for GPS is LNAW.** LNAH has significantly less gain at 1575 MHz and will result in failed acquisition (all peak ratios ~1.0–1.3).

### Gateware RF Switch Bug (v2.2)

The factory gateware (v2.2) has a bug: **the RF switch always routes the RX SMA to LNAH**, regardless of what the software selects. This means even if you set `antenna=LNAW`, GPS signal still goes through the wrong amplifier.

- **Gateware v3.x** fixes this bug — LNAW is properly connected when selected.
- Confirmed by measurement: LNAW std=75.7 vs LNAH std=65.9 at 1575 MHz with gateware v3.12.

### Gateware v3.12 Streaming Bug — use v3.11

**Gateware v3.12 delivers only ~1% of the configured sample rate** (e.g. ~40 kS/s when 4.096 MS/s is requested). This happens on every OS and host library tested — it is a gateware bug, not a driver/USB issue:

- The FPGA's own hardware timestamps advance at ~1/103 of the configured rate, with **zero** packet loss/overrun — the board simply produces too few samples.
- The ratio is constant across sample rates (4.096 / 10 / 20 MS/s), link formats (I12/I16), oversample settings, and with TX enabled or disabled.
- LMS7002M clocks (CGEN locked, RXTSP correct) read back fine.
- v3.12 release notes list a change to Mini V1/V2 clock configuration, the likely culprit.

**Gateware v3.11 streams at full rate** (measured on native Linux with LimeSuiteNG, Oct 2026):

| Requested | v3.12 | v3.11 |
|-----------|-------|-------|
| 4.096 MS/s | 39.6 kS/s | 4.096 MS/s, 0 loss |
| 10 MS/s | 96.7 kS/s | 9.99 MS/s, 0 loss |

With v3.11 the RX RF switch register (FPGA `0x0017` bits 9:8) reads `0x2` = LNAW when LNAW is selected. Physical routing to LNAW still needs to be confirmed with an outdoor capture.

### Gateware Locations

| Version | File | Size |
|---------|------|------|
| v2.2 (factory) | `%APPDATA%\LimeSuite\images\23.11\LimeSDR-Mini_HW_2.0_r2.2.bit`, or https://downloads.myriadrf.org/project/limesuite/23.11/LimeSDR-Mini_HW_2.0_r2.2.bit | 1,074,354 bytes |
| **v3.11 (recommended)** | `LimeSDR_GW/bitstream/LimeSDR_Mini_V2/limesdr_mini_v2.bit` inside `LimeSDR_GW_v3.11_bitstream.zip` from [LimeSDR_GW releases](https://github.com/myriadrf/LimeSDR_GW/releases/tag/v3.11) | 626,914 bytes |
| v3.12 (streaming bug) | Same path inside `LimeSDR_GW_v3.12_bitstream.zip` from [LimeSDR_GW releases](https://github.com/myriadrf/LimeSDR_GW/releases/tag/v3.12) | 663,350 bytes |

Use `limesdr_mini_v2.bit`, not `limesdr_mini_v2_golden.bit` or the `.mcs`.

### Flashing Gateware

With LimeSuiteNG (Linux):
```bash
limeFLASH -t FPGA/FLASH limesdr_mini_v2.bit
```

With old LimeSuite:
```
LimeUtil --update --fpga=<path_to_bitstream.bit>
```

- Flashing is slow: **~1.6 KB/s** (≈7 min for v3.11, ≈11 min for v2.2). Do **not** wrap it in a `timeout` or unplug the board mid-flash — an interrupted write leaves a partial image in flash, and the board may not boot it after power loss. If a flash is interrupted, the FPGA keeps running the old image from RAM until power-cycled, so immediately re-flash a complete image.
- After "Programming completed", **unplug and replug** the board to boot the new gateware. Confirm with `limeDevice --full` (Gateware version / revision).

**Warning**: Gateware v3.x is **not compatible** with old LimeSuite (v23.11.0). It requires LimeSuiteNG. See compatibility matrix below.

## Software Compatibility Matrix

| Gateware | Software | OS | Streaming | GPS Viable? | Notes |
|----------|----------|-----|-----------|-------------|-------|
| v2.2 | LimeSuite v23.11.0 | Windows | Full speed (~32 MB/s) | No — LNAH only (RF switch bug) | Signal too weak for acquisition |
| v3.12 | LimeSuite v23.11.0 | Windows | ~100x too slow | No | v3.12 gateware streaming bug (originally blamed on a protocol mismatch) |
| v3.12 | LimeSuiteNG (develop) | Windows | Zero packets | No | Known FT601 streaming bug on Windows |
| v3.12 | LimeSuiteNG (develop) | Linux (native) | ~1% of requested rate | No | v3.12 gateware streaming bug |
| v3.12 | LimeSuiteNG (develop) | WSL2 + usbipd | ~1% throughput | No | Most likely the same v3.12 gateware bug (originally blamed on USB/IP) |
| **v3.11** | **LimeSuiteNG (develop, 5bc2d182)** | **Linux (native, Ubuntu 26.04)** | **Full speed** | **Yes (pending outdoor test)** | Uses libusb, not FT601 D3XX |

**Bottom line**: As of October 2026, the working full-speed configuration for GPS is **gateware v3.11 + LimeSuiteNG on native Linux**. Avoid v3.12. v3.11 with LimeSuiteNG on Windows/WSL2 has not been tested; the Windows FT601 zero-packet bug is a host-side issue and probably still applies there.

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

### LimeSuiteNG (required for gateware v3.x)

Old LimeSuite (bundled with Radioconda as `LMS7Support.dll`) does NOT work with gateware v3.x. You must build LimeSuiteNG from source.

#### Building on Linux (recommended)

Tested on Ubuntu 26.04 with LimeSuiteNG `develop` at commit `5bc2d182`.

```bash
sudo apt-get install -y build-essential cmake pkg-config libusb-1.0-0-dev

git clone -b develop https://github.com/myriadrf/LimeSuiteNG.git
cd LimeSuiteNG

cmake -B build -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_GUI=OFF -DBUILD_DOCS=OFF \
  -DENABLE_UNIT_TESTS=OFF \
  -DLIMESUITENG_PCIE=OFF -DBUILD_DRIVERS_LIMEPCIE=OFF

cmake --build build -j$(nproc)
sudo make -C build install
sudo ldconfig
sudo udevadm control --reload-rules && sudo udevadm trigger
```

`make install` also installs the udev rules (`/etc/udev/rules.d/65-limesuiteng-usb.rules`), so after replugging the board no `sudo` is needed to use it. Verify with `limeDevice --full`.

`sudo` needs a real terminal for the password; run these in a normal terminal, not through a non-interactive shell.

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

### Method 2: LimeSuiteNG C++ capture (Linux — for gateware v3.11)

A custom capture program (`gps/gps_capture.cpp`) uses the LimeSuiteNG API directly:

```bash
g++ -O2 -std=c++17 gps/gps_capture.cpp -o gps_capture \
  -I/usr/local/include -L/usr/local/lib -llimesuiteng -Wl,-rpath,/usr/local/lib

./gps_capture /tmp/gps_capture.bin 30
```

The program configures: LNAW antenna, 1575.42 MHz, 4.096 MS/s, LNA=30 TIA=12 PGA=19, DC/IQ calibration, and writes raw complex float32 IQ to file.

DC/IQ calibration (`CalibrationFlag::DCIQ`) is required on LNAW: without it the Q channel is ~4x the I channel (~12 dB imbalance). LNAH is balanced either way. The first ~0.5 s of every capture contains start-up transients; `gps_rx` starts acquisition at 0.5 s so this is harmless.

A healthy 10 s capture is 40,960,000 samples (327.68 MB) in ~10.0 s; if it takes ~100x longer, check the gateware version.

**Note**: `sudo` is only required if the udev rules are missing. LimeSuiteNG's `make install` installs them; to install manually:
```bash
sudo cp ~/LimeSuiteNG/udev-rules/65-limesuiteng-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Method 3: WSL2 + usbipd (not viable for GPS)

USB passthrough via usbipd-win works for device detection and low-rate control. Streaming measured ~1% of native, but that test used gateware v3.12, whose streaming bug produces exactly this rate — so the USB/IP diagnosis is unconfirmed. Untested with v3.11.

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

On Linux, build it locally (the checked-in `gps/gps_rx` binary was built with `-march=native` on another machine and may crash with "Illegal instruction"):
```bash
gcc -O3 -march=native -o gps_rx gps/gps_rx.c -lm
./gps_rx /tmp/gps_capture.bin
```

- **File mode**: Process a completed capture file
- **Tail mode**: Follow a growing file in real-time (for streaming capture)

The IQ file format is complex float32 (interleaved I/Q, 8 bytes per sample) at 4.096 MS/s.

### LimeSDR Reference Frequency Offset (~16.5 ppm)

The LimeSDR Mini v2 tested (serial 1DBB8106DEEE92) has a reference oscillator (VCTCXO) error of about **16.5 ppm**: every satellite appears **+24.5 to +29.5 kHz** from its true Doppler (true Doppler for a stationary receiver is within ±5 kHz). The offset was constant over a 60 s capture and identical for all satellites, so it is the receiver, not the signal. The HackRF showed no such offset.

The original `gps_rx` searched only ±20 kHz, so it could never find any satellite on this LimeSDR, independently of the interference problem. `ACQ_DOPP_MAX` now defaults to ±40 kHz. Because the sample clock comes from the same reference, the offset behaves like extra Doppler everywhere (carrier and code), and tracking/PVT handle it without further correction.

Not done: trimming the board's VCTCXO DAC (reported as 1049 by `limeDevice --full`) or offsetting the LO in `gps_capture.cpp`. Either would bring satellites back near 0 Hz.

### Acquisition and Tracking Parameters

All tunables can be overridden at build time, e.g. `gcc -O3 -march=native -DACQ_MS=80 -o gps_rx gps/gps_rx.c -lm`.

| Parameter      | Default | Notes |
|----------------|---------|-------|
| `ACQ_MS`       | 40      | Non-coherent integration (ms). Was 20; 40 is needed for the LimeSDR's weaker signals |
| `ACQ_THRESH`   | 1.5     | Peak/second-peak ratio. Was 1.8; false detections are rejected later by the lock check or never produce ephemeris |
| `ACQ_DOPP_MAX` | 40000   | Doppler search ±Hz. Was 20000; see reference frequency offset above |
| `ACQ_START_S`  | 0.5     | Time of the first acquisition window (s) |
| `ACQ_TRIES`    | 5       | While fewer than 4 satellites are found, acquisition is retried every 2 s and new PRNs are added |
| `REFINE_MS`    | 400     | Fine Doppler search after acquisition (starts tracking within a few Hz instead of ±125 Hz) |
| `FLL_GAIN`     | 0.05    | FLL pull-in gain per 1 ms. Was 0.25, which was too noisy for weak signals |
| `PLL_BN`       | 15      | PLL noise bandwidth (Hz). Was 20 |
| `LOCK_RETRIES` | 3       | Failed lock checks re-run pull-in instead of dropping the satellite |
| `FS`           | 4.096e6 | Sample rate (Hz) |

Other receiver fixes (October 28, 2026):
- **Code Doppler compensation in acquisition**: the code peak drifts ~0.07 samples/ms at +26 kHz, which smeared long integrations. Each 1 ms correlation is now shifted back before summing.
- **Subframe timing by majority vote**: `decode_nav` used to take the first parity-valid subframe as the timing reference. A false preamble at 846 ms (TOW 6189 vs. true ~54305) passed parity and caused every genuine subframe to be rejected (1 valid subframe instead of 9). All parity-valid subframes are now collected and the timing most of them agree on is used.

Acquisition takes ~25 s per window on a laptop at the defaults (±40 kHz, 40 ms), so a weak capture needing retries takes 2–3 minutes before tracking starts.

## Troubleshooting

### No device found
- Check USB 3.0 connection (blue port)
- On Windows: verify FTDI D3XX driver in Device Manager (NOT WinUSB/Zadig)
- On Linux: check `lsusb` for `0403:601f` and USB permissions

### Streaming too slow (~100x)
- **Gateware v3.12 streaming bug** — check `limeDevice --full`; if it reports Gateware version 3 / revision 12, flash v3.11 (see "Gateware v3.12 Streaming Bug").
- Gateware v3.x + old LimeSuite is unsupported. Must use LimeSuiteNG.

### `gps_rx` crashes with "Illegal instruction"
- The binary was built for a different CPU. Rebuild from `gps/gps_rx.c` on the machine running it.

### Zero packets on Windows
- LimeSuiteNG FT601 streaming bug. Known issue — no fix as of Oct 2026.

### 0 satellites acquired (all ratios ~1.0)
- **USB 3.0 self-interference** (most likely with a passive antenna on the LimeSDR): measure the noise floor at 1575.42 MHz with the antenna connected and disconnected. A rise of more than a few dB means local interference — see "USB 3.0 Self-Interference".
- **Cross-check with the HackRF**: if the HackRF acquires satellites with the same antenna and spot, the antenna and sky view are fine.
- **Check I/Q balance**: Q/I rms ratio should be ~1.0. If it is ~4, DC/IQ calibration is not enabled.
- **Check antenna path**: gateware v2.2 forces LNAH regardless of software setting. Flash v3.11 or accept weak signal.
- **Check antenna placement**: must be outdoors, patch facing sky, SMA tight.
- **Check the Doppler range**: satellites on this LimeSDR appear at +24.5 to +29.5 kHz (reference frequency offset). A `gps_rx` built with `ACQ_DOPP_MAX=20000` (the old default) will never find them.
- **Try higher gains**: max out LNA=30, TIA=12, PGA=19.
- **Try longer integration**: build with `-DACQ_MS=80`.

### Device detected but "permission denied" (Linux)
- Run with `sudo`, or install udev rules (see capture method 2).

## Test Summary (October 2026)

All outdoor tests, same location in Pittsburgh. LimeSDR settings: gateware v3.11, LNAW, LNA=30 TIA=12 PGA=19, 4.096 MS/s, DC/IQ calibration on. Noise is the rms level at 1575.42 MHz over 0.5–1.0 s of the capture. "Old `gps_rx`" is the version before commit `1da59bb` (±20 kHz search, 20 ms, threshold 1.8); "new `gps_rx`" is the current default build.

**LimeSDR Mini v2** (no-antenna floor: −44.4 dBFS)

| Date | Antenna / setup | USB | Noise (dBFS) | Above floor | Receiver | Acquired | Ephemeris | Position fix |
|---|---|---|---|---|---|---|---|---|
| Oct 23 | No antenna (reference) | 3.0 | −44.4 | — | — | — | — | — |
| Oct 23 | Taoglas screwed onto board | 3.0 | −22.6 | +21.8 | old | 0 (best 1.40)¹ | 0 | No |
| Oct 23 | Taoglas on board, LNAH path | 3.0 | −23.3 | +21.1 | old | 0 (best 1.35) | 0 | No |
| Oct 27 | Taoglas, 30 cm SMA extension | 3.0 | −34.5 | +9.9 | old | 0 (best 1.42)¹ | 0 | No |
| Oct 28 | Taoglas, extension, board wrapped in foil | 2.0 | −43.9 | +0.5 | old, ±40 kHz, 40 ms, thr 1.5, start 5 s | 5 | 1 (PRN 26) | No |
| Oct 28 | + foil ground plane under antenna | 2.0 | −44.0 | +0.4 | old, ±40 kHz, 40 ms, thr 1.5, start 5 s | 4 | 2 (PRN 26, 31) | No |
| Oct 28 | same capture | 2.0 | −44.0 | +0.4 | **new** | 5 | **4** | **Yes — 55 fixes** |
| Oct 28 | same setup, fresh capture | 2.0 | −44.0 | +0.4 | **new** | 4 | **4** | **Yes — 52 fixes** |
| Oct 28 | MAX RangeAnt + foil ground plane | 2.0 | −36.2 | +8.2 | new | 0 (best 1.23) | 0 | No |
| Oct 28 | MAX RangeAnt, no ground plane | 2.0 | −19.7 | +24.7 | new | 0 (best 1.39) | 0 | No |

¹ The Oct 23 60 s acquisition capture was taken with calibration off (I/Q imbalance 4:1); the noise figure is from a calibrated capture of the same setup. The old receiver searched only ±20 kHz, so it could not have found satellites on this LimeSDR anyway (they sit at +24.5 to +29.5 kHz; see "LimeSDR Reference Frequency Offset"). On USB 3.0 the interference was also ~20 dB too high.

**HackRF Pro** (same Taoglas antenna; HackRF dBFS values are on a different scale and not comparable with the LimeSDR's)

| Date | Antenna / setup | USB | Noise on / off (dBFS) | Rise | Receiver | Acquired | Ephemeris | Position fix |
|---|---|---|---|---|---|---|---|---|
| Oct 23 | Taoglas, direct | 2.0 | −28.2 / −30.5 | +2.3 | old | 3 (ratios 9.1, 5.0, 4.3) | — | No (3 satellites) |
| Oct 27 | Taoglas, 30 cm SMA extension | 2.0 | −28.4 / — | — | old | 5 (ratios 2.5–5.5) | 4 | **Yes — 59 fixes** |

The HackRF has not yet been tested with the new `gps_rx`.

## Current Status (October 2026)

**GPS reception with LimeSDR Mini v2 on Windows is currently blocked** by the intersection of three bugs (plus the USB 3.0 interference issue below, which affects every OS):
1. Gateware v2.2 RF switch bug routes GPS to wrong antenna (LNAH)
2. Gateware v3.12 streams at ~1% of the configured rate (on any OS)
3. LimeSuiteNG (needed for gateware v3.x) has a Windows FT601 streaming bug

**Software/gateware path that works**: Native Linux (Ubuntu 26.04) + **gateware v3.11** + LimeSuiteNG + DC/IQ calibration — confirmed full-rate streaming (4.096 MS/s, zero loss) with LNAW selected, and the capture → `gps_rx` pipeline runs end to end.

**Outdoor result (Oct 2026): 0 satellites on the LimeSDR**, because USB 3.0 streaming raises the noise floor at L1 by ~20 dB when the passive antenna is mounted directly on the board (see "USB 3.0 Self-Interference"). The HackRF with the same antenna at the same spot acquired 3 satellites, so the antenna, location and `gps_rx` are fine.

**Follow-up (Oct 27, 2026)**: USB 2.0 is not an option (the board does not enumerate on a USB 2.0 hub). A 30 cm SMA extension cut the LimeSDR's L1 noise by 12 dB but it still acquired 0 satellites, while the HackRF through the same extension acquired 5 and produced a position fix.

**Position fix achieved (Oct 28, 2026)**: LimeSDR wrapped in aluminium foil, 30 cm SMA extension, foil sheet as ground plane under the antenna PCB, outdoors. With the foil wrap the board enumerated at **USB 2.0** directly on the laptop port, and L1 noise fell to −44.0 dBFS (the no-antenna floor). The updated `gps_rx` (default build) on a 60 s capture:

| | Result |
|---|---|
| Satellites with ephemeris | 4 (PRN 26, 31, 3, 4), all at +24.5 to +29.2 kHz apparent Doppler |
| Position fixes | 55 in 60 s, horizontal scatter ~10–13 m, altitude 263 m (plausible for Pittsburgh) |
| Same capture, original `gps_rx` | 0 satellites (±20 kHz search); with the search widened, 2 satellites with ephemeris, no fix |

A second, fresh 60 s capture with the same setup also gave a fix: 52 fixes from PRN 4, 26, 9 and 16 (two found only on acquisition retries), mean position ~12 m from the HackRF fix in `HACKRF_GPS_SETUP.md`.

What it took, in order of discovery: gateware v3.11 (not v3.12), DC/IQ calibration, avoiding USB 3.0 interference (foil + USB 2.0), a ground plane, a wider Doppler search for the reference offset, and the acquisition/tracking/decoding fixes above. The ground plane alone raised PRN 26 from ratio 3.4 to 9.2 and PRN 31 from 1.9 to 5.2.

**Remaining issues**: signals are still a few dB weaker than on the HackRF (with USB noise gone, the LimeSDR's post-LNA stages dominate the noise floor; LNA gain 30→0 changes the noise by only 2.7 dB), so only 4 satellites reach ephemeris. The USB 2.0 connection depends on how the foil sits. An active antenna with a bias tee would give margin for both.

**MAX RangeAnt antenna (Oct 28, 2026) — no satellites.** Swapped in for the Taoglas patch, same foil-wrapped LimeSDR on USB 2.0, same spot:

| Setup | L1 noise (rms) | Broadband floor | LO reflection spike at 0 Hz | Satellites (5 acquisition windows) |
|---|---|---|---|---|
| Taoglas + foil ground plane | −44.0 dBFS | — | small | 4 with ephemeris, position fix |
| RangeAnt + foil ground plane | −36.2 dBFS | −2.3 dB | 24.0 dB (+28.7 dB above floor) | 0 (best ratio 1.23) |
| RangeAnt, no ground plane | −19.7 dBFS | 14.5 dB (+16.8) | 41.7 dB (+17.7) | 0 (best ratio 1.39) |

The large spike at 0 Hz is the LimeSDR's own LO leaking out of the RX port and reflecting back, which points to a poor impedance match at 1575 MHz. Without the ground plane, broadband noise rose ~17 dB while the board stayed on USB 2.0, so the antenna/cable is picking up far more local noise. Not determined: what band the RangeAnt is designed for, and whether it is an active antenna (the LimeSDR Mini cannot power one). Use the Taoglas patch for GPS.
