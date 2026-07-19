# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Project renamed to **sextante**; MIT license, contribution guide, security policy, code of conduct and CI workflow.
- Hardware self-check on `initialize()`: verifies `WHO_AM_I` (MPU-9250/9255) and the AK8963 `WIA` device id, raising `HardwareMismatchError` with a diagnosis for relabeled or magnetometer-less chips. Skippable with `initialize(check_hardware=False)`; also exposed as `MPU9250.self_check()`.
- `docs/hardware.md`: the MPU-9250 as it actually is — two dies, access modes, endianness, scale factors, magnetometer axis rotation, chip identification.
- `docs/architecture.md`: module map, thread/queue model, `get_avg()` contract, data path and design decisions.
- Project website (`site/`, Astro + Three.js) with a HUD-style interface: an interactive 3D scene (stylized Pi board, holographic body-frame IMU cube with a data beam from the chip, polar grid with radar sweep), a live telemetry panel driven by the cube's actual motion, and a typed self-check boot sequence. The cube is draggable and can be driven by a phone's real IMU. Deployed to GitHub Pages on every push to `master`.
- Data pipeline around the driver, all standard library + numpy:
  - `sextante-record`: CSV session recorder (interval averages via `get_avg()`).
  - `sextante-stream`: live streaming server — Mahony AHRS fusion (`fusion.py`) on the Pi and a self-contained web viewer (3D attitude cube, telemetry, rolling charts) served by the Pi over HTTP + Server-Sent Events; `--record` streams and records simultaneously.
  - `sextante-report`: Markdown session reports (per-channel stats, accumulated rotation, stillness, peak force, magnetometer health) with optional matplotlib PNG plots (`pip install "sextante[plots]"`).
  - `DemoMPU` synthetic motion source: every command accepts `--demo`, so the whole pipeline runs without hardware; the fusion test suite closes the loop against its ground-truth attitude.
- Dead accel-Z workaround: `sextante-stream --fix-az up|down` reconstructs the vertical accel component from the 1 g constraint (`fusion.reconstruct_az`) for the attitude filter only — recorded data keeps the real sensor values. Motivated by a real Stratux AHRS whose accel Z is stuck at −0.71 g.
- Mounting correction: `MPU9250(mount='x180')` / CLI `--mount SPEC` applies a quarter-turn rotation sequence (chip frame → vehicle frame) to gyro, accel and mag after calibration, so boards that mount the chip rotated or upside down (Stratux AHRS) read level when level. Gyro calibration folds the bias back into the chip frame automatically.
- Gyro bias auto-calibration: `MPU9250.calibrate_gyro(duration)` measures the at-rest bias and folds it into `MPUCalData`; the CLI runs it at startup by default (`--calibrate SECONDS`, 0 to skip). Removes the constant ~1 °/s per-axis offset that made the attitude wander with the device still.
- BMP280/BME280 barometer support (`bmp280.py`): auto-detected at 0x76/0x77, Bosch datasheet compensation (validated against the datasheet's worked example), barometric altitude with adjustable QNH (`--qnh`). Integrated across the pipeline — pressure/altitude CSV columns, `PRESS`/`ALT` in the live viewer, altitude statistics/range in reports and plots — plus `DemoBaro` for hardware-free runs. Completes coverage of the Stratux AHRS board (MPU-9250 + BMP280).

- LSM9DS1 driver (`lsm9ds1.py`) for Ozzmaker BerryGPS-IMU boards: same reading surface as `MPU9250` (`mpuDate`, `get_avg()`, `self_check()`, `calibrate_gyro()`, `mount=`), so the entire pipeline — recorder, fusion, streamer, viewer, reports — runs unchanged on either chip. Fixed profile: gyro 245 dps @ 119 Hz, accel ±2 g, mag ±4 gauss continuous 80 Hz with the die's mirrored X axis un-mirrored into the accel/gyro frame. The CLI auto-detects the chip (`--imu auto`, default) by probing WHO_AM_I at 0x68 and 0x6A.

### Changed
- The AK8963 now runs in **continuous measurement mode (100 Hz, 16-bit)** instead of per-sample single-measurement retriggering. 16-bit output matches the driver's 0.15 µT/LSB scale — the old 14-bit mode under-reported the field by 4×.
- Magnetometer samples are validated against the real status registers: skipped unless `ST1.DRDY` is set, and discarded on `ST2.HOFL` magnetic overflow (replaces the vestigial checks inherited from the Go port).
- Magnetometer readings are **remapped into the accel/gyro frame** by default (body X = mag Y, body Y = mag X, body Z = −mag Z). `MPUCalData` mag biases and the `Ms` matrix now operate in the body frame.
- The sampling loop no longer rewrites the aux-master slave-0 configuration on every mag tick — it is set once at `initialize()`.

## [0.2.0] - 2026-07-11

### Fixed
- Gyro/accel/temperature words were read byte-swapped (SMBus words are little-endian, the MPU-9250 output registers are big-endian), so readings bore no relation to movement.
- The accelerometer range was written to `GYRO_CONFIG` instead of `ACCEL_CONFIG`, and the ±4 g / ±8 g range definitions carried the wrong bit patterns.
- The digital low-pass filter cutoff was derived from the sample-rate divider byte instead of the rate in Hz (a 50 Hz rate got a 5 Hz filter).
- The AK8963 factory sensitivity values were read from the MPU's own registers instead of the magnetometer's fuse ROM.
- Magnetometer samples were accumulated twice per interval, inflating averaged values by ~2×.
- `get_avg()` returned the previous interval's averages; it now waits for the freshly computed result.
- DMP memory writes landed at the wrong address (`address >> 0xFF` instead of `address & 0xFF`).
- Temperature now uses the MPU-9250 formula (333.87 LSB/°C, +21 °C offset) instead of the MPU-6050 one.
- `DT`/`DTM` interval lengths no longer wrap for intervals over one second.
- Raw word to `int16` conversion is done in plain Python; `np.int16()` raises `OverflowError` for negative readings on NumPy ≥ 2.

### Changed
- Restructured into an installable package (`mpu9250/` with `driver`, `constants`, `ranges`, `data`, `ticker`); the old `test.py` became `examples/read_avg.py`.
- Ported to Python 3 (the prototype required Python 2).
- The I2C bus is injectable (`MPU9250(bus=...)`) and `smbus2` is imported lazily, so the package can be used and tested without hardware.
- The `gevent_ticker` dependency was replaced by a small drift-free monotonic-clock ticker thread.
- Sampling and ticker threads are daemons, so host programs can exit cleanly.

### Added
- `pyproject.toml` packaging, README (wiring, usage, field units, calibration) and a 31-test pytest suite running against an in-memory fake bus.

## [0.1.0]

- Original prototype (Python 2): MPU-9250 setup over I2C, background sampling loop, per-interval averaging.
