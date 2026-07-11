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
- User calibration hooks (`MPUCalData`): hardware biases and a magnetometer rescaling matrix.
- Fully testable without hardware — the I2C bus is injectable.

## Project layout

```
mpu9250/            The package
├── __init__.py     Public API (MPU9250, MPUData, MPUCalData, AccelRange, GyroRange, LPF)
├── driver.py       MPU9250 class: chip setup, sampling loop, averaging
├── constants.py    Register map and configuration bits (MPU-9250 + AK8963)
├── ranges.py       Accel/gyro range definitions and LPF selection
├── data.py         MPUData / MPUCalData value objects
└── ticker.py       Drift-free periodic ticker thread used by the sampling loop
examples/
└── read_avg.py     Prints averaged readings once per second
tests/              Unit tests (no hardware required)
```

## Requirements

- Python ≥ 3.8
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

### Reading model

- `mpu.mpuDate` — most recent instantaneous sample (updated at `rate` Hz).
- `mpu.get_avg()` — average over the interval since the previous `get_avg()` call. Synchronous: it waits for the sampling thread to compute the result.

### `MPUData` fields

| Field | Meaning | Units |
|-------|---------|-------|
| `G1`, `G2`, `G3` | Gyro X/Y/Z | °/s |
| `A1`, `A2`, `A3` | Accel X/Y/Z | g |
| `M1`, `M2`, `M3` | Magnetometer X/Y/Z | µT (after factory sensitivity adjustment) |
| `Temp` | Die temperature | °C |
| `N`, `NM` | Samples averaged (accel/gyro, magnetometer) | — |
| `T`, `TM` | Timestamp of the last accel/gyro and mag sample | — |
| `DT`, `DTM` | Length of the averaged interval | ms |
| `MsgError` | Error description, `None` when the reading is valid | — |

At rest, expect `A3 ≈ 1.0` (gravity), `A1 ≈ A2 ≈ 0`, gyros ≈ 0 and `Temp` at a realistic ambient value — a quick way to sanity-check the wiring.

### Calibration

`mpu.mpuCalDate` (`MPUCalData`) holds user calibration: per-axis hardware biases for gyro (`G0*`), accel (`A0*`) and magnetometer (`M0*`), plus a 3×3 magnetometer rescaling matrix (`Ms*`, identity by default). Set these before `initialize()` to apply your own calibration; biases are in raw LSB units.

## Running the tests

The tests replace the I2C bus with an in-memory fake, so they run anywhere:

```bash
pip install -e ".[dev]"
pytest
```

## Roadmap

- Architecture documentation and diagrams.
- Hardware self-check on `initialize()` (`WHO_AM_I` / AK8963 `WIA`) to detect relabeled or magnetometer-less chips.
- Magnetometer status-register (ST1/ST2) validity checks and axis alignment with the accel/gyro frame.
- Optional continuous-measurement mode for the AK8963.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) — no hardware is needed to develop or run the tests. Please also read the [Code of Conduct](CODE_OF_CONDUCT.md).

## Security

See [SECURITY.md](SECURITY.md) for how to report vulnerabilities privately.

## License

[MIT](LICENSE) © Oscar Lago
