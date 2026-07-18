"""Session analysis: turns a recorded CSV into summary statistics and a report."""

import csv
import math
from datetime import datetime

_CHANNELS = [
    ('g1', 'Gyro X', '°/s'), ('g2', 'Gyro Y', '°/s'), ('g3', 'Gyro Z', '°/s'),
    ('a1', 'Accel X', 'g'), ('a2', 'Accel Y', 'g'), ('a3', 'Accel Z', 'g'),
    ('m1', 'Mag X', 'µT'), ('m2', 'Mag Y', 'µT'), ('m3', 'Mag Z', 'µT'),
    ('temp', 'Temperature', '°C'),
]

STILL_GYRO_DPS = 1.5  # below this on every axis, the device counts as still


def load_session(path):
    """Reads a recorder CSV into a dict of column lists (floats where numeric)."""
    rows = []
    with open(path, newline='') as fh:
        for row in csv.DictReader(fh):
            rows.append(row)
    if not rows:
        raise ValueError(f'{path}: no data rows')

    session = {'timestamp': [], 'error': []}
    for key in [c[0] for c in _CHANNELS] + ['n', 'nm', 'dt_ms', 'dtm_ms']:
        session[key] = []
    for row in rows:
        session['timestamp'].append(datetime.fromisoformat(row['timestamp']))
        session['error'].append(row.get('error') or '')
        for key in session:
            if key in ('timestamp', 'error'):
                continue
            session[key].append(float(row[key] or 0))
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

    result = {
        'rows': rows,
        'duration_s': duration,
        'row_rate_hz': (rows - 1) / duration if duration > 0 else 0.0,
        'samples': int(sum(session['n'])),
        'errors': sum(1 for e in session['error'] if e),
        'channels': {key: _stats(session[key]) for key, _, _ in _CHANNELS},
    }

    # Motion: accumulated rotation per axis (∫|ω|dt) and stillness share
    dts = [dt / 1000.0 for dt in session['dt_ms']]
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
    for key, label, unit in _CHANNELS:
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
        '',
    ]
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
    written = []
    for name, unit, keys in groups:
        fig, ax = plt.subplots(figsize=(9, 3.2), dpi=110)
        for key in keys:
            ax.plot(seconds, session[key], label=key, linewidth=0.9)
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
