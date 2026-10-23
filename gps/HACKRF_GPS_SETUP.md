# HackRF + Taoglas Antenna GPS Reception Setup

## Hardware

- **SDR**: HackRF One (firmware v2026.01.3 or later)
- **Antenna**: Taoglas CGGBP.18.4.A.02 — passive ceramic patch GPS/GLONASS/Galileo/BeiDou antenna, 2.0 dBi gain at GPS L1, 50 ohm, SMA connector
- **Connection**: Antenna SMA → HackRF ANT port (the single SMA connector)
- **USB**: Use a reliable USB cable. If you get `hackrf_open() failed: HackRF not found (-5)`, try a different cable or port — some USB-C cables don't carry data.

### Antenna Placement

The Taoglas CGGBP.18.4.A.02 is a patch antenna — it radiates upward from its flat face. Place it **outdoors with the patch facing the sky** and an unobstructed view of the horizon. GPS signals are too weak (~-130 dBm) to receive indoors with a passive antenna.

Double-check the SMA connection is tight — a loose antenna connector was the cause of a failed acquisition during initial testing.

## Software Prerequisites

Install [Radioconda](https://github.com/ryanvolz/radioconda) (conda-based distribution), which bundles:
- GNURadio 3.10+
- gr-osmosdr (SDR source block)
- Python, NumPy

Launch GNURadio Companion from the Radioconda environment (e.g. from the Radioconda start menu shortcut or by activating the conda environment first).

Build the GPS software receiver:

```
gcc -O3 -o gps/gps_rx.exe gps/gps_rx.c -lm
```

On Windows with MSYS2/MinGW:
```
gcc -O3 -march=native -o gps/gps_rx.exe gps/gps_rx.c -lm
```

## GNURadio Flowgraph Configuration

Open `gps/gps_recv.grc` in GNURadio Companion. The flowgraph uses:

### osmosdr Source Block

| Parameter     | Value         | Notes                                    |
|---------------|---------------|------------------------------------------|
| args          | `""`          | Auto-detects HackRF                      |
| Sample Rate   | 4.096 MS/s    | Set via `samp_rate` variable             |
| Frequency     | 1575.42 MHz   | GPS L1 C/A center frequency              |
| RF Gain       | 25 dB         | `gain0` — HackRF LNA (0 or 14 dB steps; 25 worked best in testing) |
| IF Gain       | 40 dB         | `if_gain0` — HackRF IF stage (0–40 dB)  |
| BB Gain       | 40 dB         | `bb_gain0` — HackRF VGA (0–62 dB)       |
| DC Offset     | 0 (Off)       | `dc_offset_mode0`                        |
| Antenna       | (empty)       | HackRF has one port, no selection needed |

> **Note on gain**: The GRC file originally shipped with `gain0=14`. During testing, increasing RF gain to 25 improved acquisition significantly. If you get 0 satellites at gain0=14, try 25.

### File Sink Block

| Parameter | Value              | Notes                              |
|-----------|--------------------|------------------------------------|
| File      | `gps_captured.iq`  | Written to GNURadio's working directory (project root, **not** `gps/`) |
| Type      | Complex            | complex float32 (8 bytes/sample)   |
| Append    | False              | Overwrites on each run             |
| Unbuffered| False              |                                    |

### Frequency Sink (optional)

A QT GUI Frequency Sink is connected for live spectrum visualization during capture. Not required for GPS processing.

## Running the Pipeline

### Option A: Batch capture then process

1. Open `gps/gps_recv.grc` in GNURadio Companion
2. Click **Run** (or press F6) — recording starts immediately to `gps_captured.iq`
3. Let it record for **at least 90 seconds** (ephemeris decode needs subframes 1, 2, 3 back to back, which can take up to ~50 s)
4. Stop the flowgraph
5. Run the receiver from the project root:
   ```
   gps\gps_rx.exe gps_captured.iq
   ```

### Option B: Real-time streaming (tail mode)

Either order works — `gps_rx --tail` polls for new data:

1. Open and run the GNURadio flowgraph — it starts writing to `gps_captured.iq`
2. In a separate terminal, start the GPS receiver:
   ```
   gps\gps_rx.exe --tail gps_captured.iq
   ```
3. `gps_rx` follows the growing file, acquires satellites, and begins tracking as data arrives

> **Do not use TCP streaming.** An earlier attempt using GNURadio's TCP Sink + a Python bridge (`tcp_to_stdin.py`) to pipe data to `gps_rx` via stdin caused GNURadio crashes and buffer overflows ('O' characters corrupting the IQ stream). The file-based tail mode is reliable and avoids these issues.

## Expected Output

Successful acquisition shows satellites with peak/ratio above 1.8. A real capture from Pittsburgh, PA produced:

```
Acquisition: searching 32 PRNs, +/-20000 Hz ...
...
Acquired 5 satellites:
  PRN  8  ratio   3.4  doppler   -1250 Hz  codephase  590
  PRN  2  ratio   2.9  doppler    2500 Hz  codephase  200
  PRN  7  ...
  PRN  9  ...
  PRN 27  ...
```

After tracking and ephemeris decode (~20–90 s of data), a position fix is printed:

```
40.442246, -79.946124, 268.6m
```

First fix was obtained at ~20 seconds with 5 satellites. Expect tens of meters of error (no iono/tropo corrections applied).

## Acquisition Parameter Tuning

The default acquisition parameters in `gps_rx.c` were tuned for the HackRF + Taoglas setup:

| Parameter   | Original | Tuned  | Why                                        |
|-------------|----------|--------|--------------------------------------------|
| `ACQ_MS`    | 10       | 20     | Longer coherent integration improves SNR for weak passive antenna |
| `ACQ_THRESH`| 2.0      | 1.8    | Lower threshold detects more marginal satellites |

If you are using a different antenna with more gain, the original values (10 / 2.0) may work fine and run faster.

## Troubleshooting

- **`hackrf_open() failed: HackRF not found (-5)`**: HackRF not detected. Try a different USB cable or port. Some USB-C cables are charge-only.
- **0 satellites acquired, all ratios ~1.0**: No GPS signal reaching the receiver. Check: (1) antenna is outdoors with patch facing sky, (2) SMA connector is tight, (3) try increasing RF gain to 25 dB.
- **"cannot open gps_captured.iq"**: The file is created in GNURadio's working directory (project root), **not** in `gps/`. Run `gps_rx` from the project root: `gps\gps_rx.exe gps_captured.iq`.
- **"not enough data for acquisition"**: Need at least 0.5 s of data. In file mode, ensure the recording completed. In tail mode, ensure GNURadio is running and writing to the same file path.
- **Satellites acquired but no position fix**: Need ≥90 s of continuous recording for ephemeris decode. Ensure at least 4 satellites are acquired.

## Signal Parameters Reference

| Parameter        | Value           |
|------------------|-----------------|
| GPS L1 C/A freq  | 1575.42 MHz    |
| C/A code rate     | 1.023 Mchip/s  |
| Sample rate       | 4.096 MS/s     |
| Sample format     | Complex float32 (interleaved I/Q, 8 bytes/sample) |
| Acquisition integration | 20 ms (non-coherent, ACQ_MS=20) |
| Acquisition threshold   | 1.8 peak/second-peak ratio |
| Doppler search    | ±20 kHz, 250 Hz step |
