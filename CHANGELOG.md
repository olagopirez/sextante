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
