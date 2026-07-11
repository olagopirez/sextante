# Driver architecture

How sextante is put together and why. For the hardware facts underneath, see
[hardware.md](hardware.md).

## Module map

```mermaid
flowchart TD
    INIT["mpu9250/__init__.py<br/>public API"] --> DRV[driver.py<br/>MPU9250]
    DRV --> CONST[constants.py<br/>register map + config bits]
    DRV --> RNG[ranges.py<br/>AccelRange / GyroRange / LPF]
    DRV --> DATA[data.py<br/>MPUData / MPUCalData]
    DRV --> TICK[ticker.py<br/>TickerThread]
    RNG --> CONST
```

| Module | Responsibility |
|--------|----------------|
| `driver.py` | Chip setup, self-check, the sampling loop, averaging, bus I/O |
| `constants.py` | Every register address and configuration bit, named after the datasheet |
| `ranges.py` | Full-scale range definitions (bits + LSB scale) and low-pass filter selection |
| `data.py` | Value objects: a reading (`MPUData`) and user calibration (`MPUCalData`) |
| `ticker.py` | Drift-free periodic tick source used by the sampling loop |

The I2C bus is **injected** (`MPU9250(bus=...)`); `smbus2` is imported lazily only
when no bus is given. That single seam is what makes the whole test suite run
without hardware.

## Runtime model

After `initialize()`, one background **reader thread** owns all I2C traffic. It
multiplexes three event sources into a single queue — a `select` over channels,
inherited from the driver's Go ancestry (goflying/stratux), expressed with
`queue.Queue` and forwarder threads:

```mermaid
flowchart LR
    T1[TickerThread<br/>accel/gyro rate] --> F1[forwarder] --> C{{combined queue}}
    T2[TickerThread<br/>mag rate, capped 100 Hz] --> F2[forwarder] --> C
    Q[request queue<br/>get_avg] --> F3[forwarder] --> C
    C --> R[reader thread]
    R -- "reads registers, accumulates" --> BUS[(I2C bus)]
    R -- "publishes" --> D[mpuDate / mpuAvgDate]
```

Per event:

- **accel/gyro tick** — read the six gyro/accel words and the temperature
  (big-endian), scale the freshest magnetometer values, publish an instantaneous
  `MPUData` in `mpuDate`, add everything to the running accumulators.
- **mag tick** — point slave 0 at the AK8963 data registers, read the words the aux
  master copied into `EXT_SENS_DATA` (little-endian), accumulate them separately.
  The mag rate is capped at 100 Hz, the AK8963's maximum.
- **request** — compute the interval average, hand it back, reset the accumulators.

All sampling state (accumulators, counters, timestamps) lives in **locals of the
reader loop**, not on the object — nothing else may touch it, so there are no locks.

## The `get_avg()` contract

`get_avg()` is synchronous by design: the caller gets the average of *everything
sampled since the previous call*, computed at the moment of the request.

```mermaid
sequenceDiagram
    participant App
    participant Reader as reader thread
    App->>Reader: request queue ← reply queue
    Note over Reader: computes average,<br/>resets accumulators
    Reader-->>App: reply queue ← MPUData
    App->>App: returns MPUData
```

The reply travels on a queue created per call, so concurrent callers can't steal
each other's answers. The previous design returned `mpuAvgDate` immediately after
*queuing* the request — every caller read the **previous** interval's data, one full
poll behind reality.

## Data path

```
raw register pair ──► int16 (endianness per die) ──► − hardware bias (MPUCalData)
   ──► × range scale (LSB → physical unit) ──► MPUData  [°/s, g, µT, °C]
```

The magnetometer additionally multiplies by the per-axis factory sensitivity
(`mcal1..3`, from the AK8963 fuse ROM) and then by the 3×3 `Ms` rescaling matrix —
identity by default, and the hook where soft-iron correction or the axis remap
(see [hardware.md → Axes](hardware.md#axes-the-magnetometer-frame-is-rotated)) plugs in.

## Design decisions

- **Polling, not FIFO/interrupts.** The FIFO and interrupt paths are disabled and the
  driver reads output registers on its own clock. Simpler, and accurate enough at
  ≤ 200 Hz; the cost is sensitivity to scheduler jitter, mitigated by the
  monotonic-clock ticker.
- **Hardware self-check before configuration.** `initialize()` refuses to configure
  a chip whose `WHO_AM_I`/`WIA` don't match (relabeled MPU-6500 boards are common);
  `initialize(check_hardware=False)` opts out.
- **One reader thread owns the bus.** After `initialize()`, no other thread issues
  I2C transactions, so transactions never interleave. Corollary: one `MPU9250`
  instance per bus.
- **Daemon threads.** Sampling never blocks process exit; there is no shutdown
  ceremony to forget.
- **Value objects out, calibration in.** Readers get immutable-in-practice
  `MPUData` snapshots; calibration enters only through `MPUCalData` before
  `initialize()`.

## Known limitations

- The AK8963 `ST1`/`ST2` status checks in the mag path are vestigial (inherited from
  the Go port) — data-ready and overflow are not reliably detected yet.
- The magnetometer is reported in its native, rotated frame unless the user sets the
  `Ms` matrix (see above).
- `DT` in `MPUData` measures wall-clock interval, not sample count × period; under
  heavy CPU load the effective rate can droop silently.
- One instance per bus is assumed, not enforced.

These are the roadmap items, in priority order.
