# sextante

[![CI](https://github.com/olagopirez/sextante/actions/workflows/ci.yml/badge.svg)](https://github.com/olagopirez/sextante/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python](https://img.shields.io/badge/python-3.9%2B-blue.svg)](pyproject.toml)

*Sextante* — Spanish for **sextant**, the classic instrument for finding your attitude and heading from the sky.

Python driver for the InvenSense **MPU-9250** 9-axis IMU — 3-axis accelerometer, 3-axis gyroscope and on-board **AK8963** 3-axis magnetometer — read over I2C, designed for the Raspberry Pi.

The driver samples the sensor from a background thread at a configurable rate and lets you pull instantaneous readings or per-interval averages. The importable package keeps the hardware's name: `import mpu9250`.

## Features

- Background sampling thread with independent accel/gyro and magnetometer tick rates.
- Per-interval averaging: `get_avg()` returns the average of everything sampled since the previous call.
- Configurable accelerometer range (±2/4/8/16 g), gyro range (±250/500/1000/2000 °/s) and digital low-pass filter (derived from the sample rate).
- Reads the AK8963 factory sensitivity adjustment (fuse ROM) at startup.
- Magnetometer in continuous 16-bit mode (0.15 µT/LSB), validity-gated by the `ST1`/`ST2` status registers and reported in the accel/gyro frame.
- Hardware self-check on startup: verifies `WHO_AM_I` and the AK8963 device id, catching the common relabeled/magnetometer-less boards before configuring anything.
- User calibration hooks (`MPUCalData`): hardware biases and a magnetometer rescaling matrix.
- Session recording to CSV on the Pi and Markdown/PNG analysis reports (`sextante-record`, `sextante-report`).
- BMP280/BME280 barometer support, auto-detected at `0x76`/`0x77`: pressure, temperature and barometric altitude (adjustable QNH), recorded, streamed and reported alongside the IMU.
- Live motion viewing from any PC: Mahony sensor fusion on the Pi and a zero-install web viewer served by the Pi itself (`sextante-stream`).
- Fully testable without hardware — the I2C bus is injectable, and every CLI accepts `--demo` to run on synthetic motion.

## Project layout

```
mpu9250/            The package
├── __init__.py     Public API (MPU9250, MPUData, MPUCalData, ranges, MahonyAHRS, Recorder, DemoMPU)
├── driver.py       MPU9250 class: chip setup, sampling loop, averaging
├── constants.py    Register map and configuration bits (MPU-9250 + AK8963)
├── ranges.py       Accel/gyro range definitions and LPF selection
├── data.py         MPUData / MPUCalData value objects
├── ticker.py       Drift-free periodic ticker thread used by the sampling loop
├── fusion.py       Mahony AHRS attitude filter + quaternion helpers
├── bmp280.py       Bosch BMP280/BME280 barometer (pressure, temperature, altitude)
├── recorder.py     CSV session recorder
├── report.py       Session analysis and Markdown/PNG report rendering
├── streamer.py     Sampling hub + SSE HTTP server (serves the live viewer)
├── demo.py         Synthetic motion source (run everything without hardware)
├── cli.py          sextante-record / sextante-stream / sextante-report
└── web/viewer.html Self-contained live viewer app (no external dependencies)
docs/
├── hardware.md     What the MPU-9250 actually is (dies, buses, formats, identification)
└── architecture.md How the driver works (threads, queues, data path, decisions)
examples/
└── read_avg.py     Prints averaged readings once per second
tests/              Unit tests (no hardware required)
```

## Requirements

- Python ≥ 3.9
- `numpy`, `smbus2` (installed automatically)
- On the Raspberry Pi: I2C enabled (`raspi-config` → Interface Options → I2C)

### Wiring (Raspberry Pi)

| MPU-9250 pin | Raspberry Pi pin |
|--------------|------------------|
| VCC          | 3V3 (pin 1)      |
| GND          | GND (pin 6)      |
| SDA          | GPIO 2 / SDA (pin 3) |
| SCL          | GPIO 3 / SCL (pin 5) |

The chip must answer at address `0x68` (`i2cdetect -y 1`). If AD0 is pulled high it answers at `0x69`; pass `address=0x69` in that case.

## Installation

```bash
git clone https://github.com/olagopirez/sextante.git
cd sextante
pip install -e .
```

## Usage

```python
import time
from mpu9250 import MPU9250, AccelRange, GyroRange

mpu = MPU9250(accel_range=AccelRange.RANGE_2_G,
              gyro_range=GyroRange.RANGE_250_DPS,
              rate=50)
mpu.initialize()

while True:
    time.sleep(1)
    data = mpu.get_avg()   # average of the last interval; blocks until computed
    print(data.get_json())
```

`examples/read_avg.py` contains the same loop ready to run.

### Hardware self-check

`initialize()` first verifies the chip is a genuine MPU-9250/9255 with a responding
AK8963, and raises `HardwareMismatchError` with a diagnosis otherwise (e.g. a
relabeled MPU-6500 without magnetometer). Use `initialize(check_hardware=False)` to
skip it, or call `mpu.self_check()` yourself to identify a board. See
[docs/hardware.md](docs/hardware.md#identification-is-your-chip-real) for the
`WHO_AM_I` value table.

### Reading model

- `mpu.mpuDate` — most recent instantaneous sample (updated at `rate` Hz).
- `mpu.get_avg()` — average over the interval since the previous `get_avg()` call. Synchronous: it waits for the sampling thread to compute the result.

### `MPUData` fields

| Field | Meaning | Units |
|-------|---------|-------|
| `G1`, `G2`, `G3` | Gyro X/Y/Z | °/s |
| `A1`, `A2`, `A3` | Accel X/Y/Z | g |
| `M1`, `M2`, `M3` | Magnetometer X/Y/Z, in the accel/gyro frame | µT (after factory sensitivity adjustment) |
| `Temp` | Die temperature | °C |
| `N`, `NM` | Samples averaged (accel/gyro, magnetometer) | — |
| `T`, `TM` | Timestamp of the last accel/gyro and mag sample | — |
| `DT`, `DTM` | Length of the averaged interval | ms |
| `MsgError` | Error description, `None` when the reading is valid | — |

At rest, expect `A3 ≈ 1.0` (gravity), `A1 ≈ A2 ≈ 0`, gyros ≈ 0 and `Temp` at a realistic ambient value — a quick way to sanity-check the wiring.

### Calibration

`mpu.mpuCalDate` (`MPUCalData`) holds user calibration: per-axis hardware biases for gyro (`G0*`), accel (`A0*`) and magnetometer (`M0*`), plus a 3×3 magnetometer rescaling matrix (`Ms*`, identity by default). Set these before `initialize()`. Gyro/accel biases are in raw LSB; the magnetometer biases and `Ms` matrix apply **in the accel/gyro frame**, after factory scaling and the axis remap — use them for hard-iron/soft-iron correction.

## Recording, streaming and reports

The package installs three commands that turn the driver into a full data pipeline:

```bash
# 1. Record a session on the Pi: one CSV row per interval average
sextante-record -o session.csv --interval 0.2

# 2. Watch the movement live from your PC — the Pi serves the viewer app itself
sextante-stream --port 8000              # then open http://<pi-address>:8000
sextante-stream --record session.csv     # stream and record at the same time

# 3. Analyze what you captured
sextante-report session.csv -o report.md
sextante-report session.csv --plots out/   # PNG time series (pip install matplotlib)
```

**The viewer needs nothing installed on the PC**: the Pi serves a single self-contained
page at `/` and streams data over Server-Sent Events at `/events` (pure standard
library — no websockets, no frameworks). The page shows the live 3D attitude cube,
telemetry and rolling charts; orientation is estimated on the Pi by a **Mahony AHRS
filter** fusing gyro + accel + magnetometer, all in the body frame the driver reports.

**Reports** summarize a recorded session: per-channel statistics, accumulated rotation
per axis, stillness share, peak specific force, and a magnetometer health check
(Earth's field magnitude should sit in ~25–65 µT).

**Barometer**: when a BMP280/BME280 answers on the bus (the Stratux AHRS board pairs
one with the MPU-9250), every command picks it up automatically — pressure/altitude
columns in the CSV, `PRESS`/`ALT` in the live viewer, altitude statistics and range in
reports. Set the sea-level pressure with `--qnh 1020.5` for true altitude, or skip the
sensor with `--no-baro`.

Every command accepts `--demo` to run against a synthetic motion source — the whole
pipeline works on any machine, no hardware needed. Note: the recorder is the only
component that may call `get_avg()` (it resets the interval accumulators); the
streamer reads instantaneous samples, so recording and streaming coexist fine.

## Running the tests

The tests replace the I2C bus with an in-memory fake, so they run anywhere:

```bash
pip install -e ".[dev]"
pytest
```

## Documentation

- **Website:** [olagopirez.github.io/sextante](https://olagopirez.github.io/sextante/) — project landing page (source in [`site/`](site/), deployed to GitHub Pages on every push to `master`).
- [docs/hardware.md](docs/hardware.md) — what the MPU-9250 actually is: the two dies, buses and access modes, data formats and the endianness trap, scale factors, magnetometer axis rotation, and how to tell a genuine chip from a relabeled one.
- [docs/architecture.md](docs/architecture.md) — how the driver works: module map, thread and queue model, the `get_avg()` contract, data path and design decisions.

## Roadmap

- Interrupt/FIFO-driven sampling as an alternative to polling.
- A tilt-compensated compass-heading example.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) — no hardware is needed to develop or run the tests. Please also read the [Code of Conduct](CODE_OF_CONDUCT.md).

## Security

See [SECURITY.md](SECURITY.md) for how to report vulnerabilities privately.

## License

[MIT](LICENSE) © Oscar Lago
