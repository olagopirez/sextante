"""Session analysis: turns a recorded CSV into summary statistics and a report."""

import csv
import math
from datetime import datetime

_CHANNELS = [
    ('g1', 'Gyro X', '°/s', 1), ('g2', 'Gyro Y', '°/s', 1), ('g3', 'Gyro Z', '°/s', 1),
    ('a1', 'Accel X', 'g', 1), ('a2', 'Accel Y', 'g', 1), ('a3', 'Accel Z', 'g', 1),
    ('m1', 'Mag X', 'µT', 1), ('m2', 'Mag Y', 'µT', 1), ('m3', 'Mag Z', 'µT', 1),
    ('temp', 'Temperature', '°C', 1),
]

# Present only in sessions recorded with a barometer attached
_BARO_CHANNELS = [
    ('press_pa', 'Pressure', 'hPa', 0.01),
    ('baro_temp', 'Baro temperature', '°C', 1),
    ('alt_m', 'Altitude', 'm', 1),
]

# Present only in sessions recorded with a GPS attached
_GPS_CHANNELS = [
    ('speed_kmh', 'GPS speed', 'km/h', 1),
    ('sats', 'Satellites', 'sats', 1),
    ('gps_alt_m', 'GPS altitude', 'm', 1),
]


def _haversine_m(lat1, lon1, lat2, lon2):
    """Great-circle distance between two points, in meters."""
    r_earth = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r_earth * math.asin(math.sqrt(a))

STILL_GYRO_DPS = 1.5  # below this on every axis, the device counts as still


def load_session(path):
    """Reads a recorder CSV into a dict of column lists, keyed by the header.
    Numeric cells parse to float; empty cells (e.g. no barometer) to None."""
    with open(path, newline='') as fh:
        reader = csv.DictReader(fh)
        fields = reader.fieldnames or []
        rows = list(reader)
    if not rows:
        raise ValueError(f'{path}: no data rows')

    session = {key: [] for key in fields}
    for row in rows:
        for key in fields:
            raw = row.get(key) or ''
            if key == 'timestamp':
                session[key].append(datetime.fromisoformat(raw))
            elif key == 'error':
                session[key].append(raw)
            else:
                session[key].append(float(raw) if raw != '' else None)
    return session


def _stats(values):
    n = len(values)
    mean = sum(values) / n
    var = sum((v - mean) ** 2 for v in values) / n
    return {
        'mean': mean,
        'std': math.sqrt(var),
        'min': min(values),
        'max': max(values),
        'rms': math.sqrt(sum(v * v for v in values) / n),
    }


def analyze(session):
    """Computes the summary metrics a report is rendered from."""
    ts = session['timestamp']
    duration = (ts[-1] - ts[0]).total_seconds()
    rows = len(ts)

    channels = {}
    for key, _, _, scale in _CHANNELS + _BARO_CHANNELS + _GPS_CHANNELS:
        values = [v * scale for v in session.get(key, []) if v is not None]
        if values:
            channels[key] = _stats(values)

    result = {
        'rows': rows,
        'duration_s': duration,
        'row_rate_hz': (rows - 1) / duration if duration > 0 else 0.0,
        'samples': int(sum(session['n'])),
        'errors': sum(1 for e in session['error'] if e),
        'channels': channels,
    }

    # Motion: accumulated rotation per axis (∫|ω|dt) and stillness share
    dts = [(dt or 0) / 1000.0 for dt in session['dt_ms']]
    for axis in ('g1', 'g2', 'g3'):
        result[f'rotation_{axis}_deg'] = sum(abs(w) * dt for w, dt in zip(session[axis], dts))
    still = sum(
        1 for i in range(rows)
        if all(abs(session[a][i]) < STILL_GYRO_DPS for a in ('g1', 'g2', 'g3'))
    )
    result['still_pct'] = 100.0 * still / rows

    # Peak specific force and when it happened
    amag = [math.sqrt(session['a1'][i] ** 2 + session['a2'][i] ** 2 + session['a3'][i] ** 2)
            for i in range(rows)]
    peak = max(range(rows), key=lambda i: amag[i])
    result['peak_accel_g'] = amag[peak]
    result['peak_accel_at'] = ts[peak]

    # Magnetometer health: Earth's field magnitude should sit in ~25–65 µT
    mmag = [math.sqrt(session['m1'][i] ** 2 + session['m2'][i] ** 2 + session['m3'][i] ** 2)
            for i in range(rows)]
    result['mag_mean_ut'] = sum(mmag) / rows
    result['mag_plausible'] = 25.0 <= result['mag_mean_ut'] <= 65.0

    # Altitude excursion, when a barometer was recorded
    alts = [v for v in session.get('alt_m', []) if v is not None]
    if alts:
        result['alt_min_m'] = min(alts)
        result['alt_max_m'] = max(alts)

    # Track metrics, when a GPS was recorded
    points = [(la, lo) for la, lo in zip(session.get('lat', []), session.get('lon', []))
              if la is not None and lo is not None]
    if len(points) >= 2:
        result['gps_distance_m'] = sum(
            _haversine_m(*points[i - 1], *points[i]) for i in range(1, len(points)))
    speeds = [v for v in session.get('speed_kmh', []) if v is not None]
    if speeds:
        result['gps_max_kmh'] = max(speeds)

    return result


def render_markdown(path, session, analysis):
    """Renders the analysis as a Markdown report."""
    a = analysis
    lines = [
        '# sextante session report',
        '',
        f'Source: `{path}`',
        '',
        '## Session',
        '',
        '| Metric | Value |',
        '|--------|-------|',
        f"| Start | {session['timestamp'][0].isoformat()} |",
        f"| Duration | {a['duration_s']:.1f} s |",
        f"| Rows | {a['rows']} ({a['row_rate_hz']:.2f} rows/s) |",
        f"| Sensor samples averaged | {a['samples']} |",
        f"| Rows with errors | {a['errors']} |",
        '',
        '## Channels',
        '',
        '| Channel | Unit | Mean | Std | Min | Max | RMS |',
        '|---------|------|------|-----|-----|-----|-----|',
    ]
    for key, label, unit, _ in _CHANNELS + _BARO_CHANNELS + _GPS_CHANNELS:
        if key not in a['channels']:
            continue
        s = a['channels'][key]
        lines.append(
            f"| {label} | {unit} | {s['mean']:.3f} | {s['std']:.3f} "
            f"| {s['min']:.3f} | {s['max']:.3f} | {s['rms']:.3f} |")
    mag_verdict = 'plausible Earth field ✓' if a['mag_plausible'] else 'OUT OF RANGE — check calibration/mounting'
    lines += [
        '',
        '## Motion',
        '',
        '| Metric | Value |',
        '|--------|-------|',
        f"| Accumulated rotation X | {a['rotation_g1_deg']:.1f}° |",
        f"| Accumulated rotation Y | {a['rotation_g2_deg']:.1f}° |",
        f"| Accumulated rotation Z | {a['rotation_g3_deg']:.1f}° |",
        f"| Still (gyro < {STILL_GYRO_DPS}°/s) | {a['still_pct']:.1f}% of the session |",
        f"| Peak specific force | {a['peak_accel_g']:.3f} g at {a['peak_accel_at'].isoformat()} |",
        f"| Magnetic field magnitude | {a['mag_mean_ut']:.1f} µT — {mag_verdict} |",
    ]
    if 'alt_min_m' in a:
        span = a['alt_max_m'] - a['alt_min_m']
        lines.append(
            f"| Altitude range | {a['alt_min_m']:.1f} – {a['alt_max_m']:.1f} m (Δ {span:.1f} m) |")
    if 'gps_distance_m' in a:
        lines.append(f"| GPS distance | {a['gps_distance_m']:.1f} m |")
    if 'gps_max_kmh' in a:
        lines.append(f"| GPS max speed | {a['gps_max_kmh']:.1f} km/h |")
    lines.append('')
    return '\n'.join(lines)


def write_plots(session, out_dir):
    """Writes one PNG per sensor into out_dir. Requires matplotlib."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError('plots require matplotlib: pip install matplotlib') from exc

    import os
    os.makedirs(out_dir, exist_ok=True)
    t0 = session['timestamp'][0]
    seconds = [(t - t0).total_seconds() for t in session['timestamp']]
    groups = [
        ('gyro', '°/s', ['g1', 'g2', 'g3']),
        ('accel', 'g', ['a1', 'a2', 'a3']),
        ('mag', 'µT', ['m1', 'm2', 'm3']),
        ('temp', '°C', ['temp']),
    ]
    if any(v is not None for v in session.get('alt_m', [])):
        groups.append(('altitude', 'm', ['alt_m']))
    written = []
    for name, unit, keys in groups:
        fig, ax = plt.subplots(figsize=(9, 3.2), dpi=110)
        for key in keys:
            pairs = [(s, v) for s, v in zip(seconds, session[key]) if v is not None]
            ax.plot([p[0] for p in pairs], [p[1] for p in pairs], label=key, linewidth=0.9)
        ax.set_xlabel('s')
        ax.set_ylabel(unit)
        ax.set_title(name)
        ax.legend(loc='upper right', fontsize=8)
        fig.tight_layout()
        path = os.path.join(out_dir, f'{name}.png')
        fig.savefig(path)
        plt.close(fig)
        written.append(path)
    return written
