# Contributing to sextante

Thanks for your interest in improving sextante! Contributions of all kinds are welcome: bug reports, hardware test results, documentation and code.

## Development setup

No hardware is required to develop or run the test suite — the I2C bus is injectable and the tests use an in-memory fake.

```bash
git clone https://github.com/olagopirez/sextante.git
cd sextante
python3 -m venv .venv && source .venv/bin/activate
pip install -e ".[dev]"
pytest
```

## Guidelines

- **Open an issue first** for anything non-trivial, so we can agree on the approach before you invest time.
- **Every code change needs tests.** Unit tests must not require hardware: inject a fake bus (see `tests/conftest.py`) instead of talking to `smbus2`.
- **Keep hardware facts referenced.** Register addresses, scale factors and timing values must match the MPU-9250 / AK8963 datasheets; mention the datasheet section in the PR when you change one.
- **Match the existing style.** Plain Python, no new runtime dependencies without discussion, comments only where the hardware constraint isn't obvious from the code.
- **Update the docs.** If your change affects behavior, update `README.md` and add an entry to `CHANGELOG.md` under *Unreleased*.

## Testing on real hardware

CI only runs the unit suite. If you can, before submitting a driver change run `examples/read_avg.py` on a Raspberry Pi with a real MPU-9250 and include a sample of the output in the PR description: at rest, expect `A3 ≈ 1.0`, `A1 ≈ A2 ≈ 0`, gyros ≈ 0 and a realistic `Temp`.

## Pull requests

- Branch from `master`, one logical change per PR.
- Make sure `pytest` passes locally.
- Fill in the PR template, including how the change was verified.
