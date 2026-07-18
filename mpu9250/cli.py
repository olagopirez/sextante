"""Command-line entry points: sextante-record, sextante-stream, sextante-report."""

import argparse
import sys
import time
from datetime import datetime


def _add_source_args(parser):
    parser.add_argument('--rate', type=int, default=50,
                        help='driver sample rate in Hz (default 50)')
    parser.add_argument('--demo', action='store_true',
                        help='use a synthetic motion source instead of hardware')
    parser.add_argument('--skip-check', action='store_true',
                        help='skip the WHO_AM_I/WIA hardware self-check')
    parser.add_argument('--address', type=lambda v: int(v, 0), default=None,
                        help='I2C address (default 0x68; use 0x69 with AD0 high)')
    parser.add_argument('--no-baro', action='store_true',
                        help='do not look for a BMP280 barometer')
    parser.add_argument('--baro-address', type=lambda v: int(v, 0), default=None,
                        help='BMP280 I2C address (default: probe 0x76 then 0x77)')
    parser.add_argument('--qnh', type=float, default=1013.25, metavar='hPa',
                        help='sea-level pressure for barometric altitude (default 1013.25)')
    parser.add_argument('--calibrate', type=float, default=2.0, metavar='SECONDS',
                        help='measure gyro bias at startup with the device still '
                             '(default 2.0; use 0 to skip)')


def _make_source(args):
    if args.demo:
        from .demo import DemoBaro, DemoMPU
        mpu = DemoMPU(rate=args.rate)
        mpu.initialize()
        baro = None if args.no_baro else DemoBaro(sea_level_pa=args.qnh * 100.0)
        return mpu, baro, 'demo'

    from .constants import MPU_ADDRESS
    from .driver import MPU9250
    mpu = MPU9250(address=args.address or MPU_ADDRESS, rate=args.rate)
    mpu.initialize(check_hardware=not args.skip_check)

    if args.calibrate > 0:
        print(f'calibrating gyro bias — keep the device still for {args.calibrate:g}s...')
        bias = mpu.calibrate_gyro(args.calibrate)
        print(f'gyro bias removed: [{bias[0]:+.2f}, {bias[1]:+.2f}, {bias[2]:+.2f}] °/s')

    baro = None
    if not args.no_baro:
        from .bmp280 import BMP280
        from .driver import HardwareMismatchError
        try:
            baro = BMP280(address=args.baro_address, sea_level_pa=args.qnh * 100.0)
            chip_id = baro.initialize()
            print(f'barometer: {"BME280" if chip_id == 0x60 else "BMP280"} found')
        except (HardwareMismatchError, OSError) as exc:
            print(f'barometer: none ({exc})')
            baro = None
    return mpu, baro, 'mpu9250'


def record_main(argv=None):
    parser = argparse.ArgumentParser(
        prog='sextante-record',
        description='Records interval-averaged IMU readings to a CSV session file.')
    _add_source_args(parser)
    parser.add_argument('-o', '--out', default=None,
                        help='output CSV path (default sextante-<timestamp>.csv)')
    parser.add_argument('--interval', type=float, default=0.2,
                        help='seconds between rows; each row averages the interval (default 0.2)')
    parser.add_argument('--duration', type=float, default=None,
                        help='stop after this many seconds (default: run until Ctrl-C)')
    args = parser.parse_args(argv)

    from .recorder import Recorder
    path = args.out or datetime.now().strftime('sextante-%Y%m%d-%H%M%S.csv')
    mpu, baro, source = _make_source(args)
    print(f'recording {source} @ {args.rate} Hz -> {path} '
          f'(one row every {args.interval:g}s; Ctrl-C to stop)')

    recorder = Recorder(mpu, path, interval=args.interval, baro=baro)
    recorder.start()
    try:
        if args.duration:
            time.sleep(args.duration)
        else:
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.stop()
        print(f'\n{recorder.rows} rows written to {path}')
    return 0


def stream_main(argv=None):
    parser = argparse.ArgumentParser(
        prog='sextante-stream',
        description='Serves the live web viewer and an SSE data stream.')
    _add_source_args(parser)
    parser.add_argument('--host', default='0.0.0.0', help='bind address (default 0.0.0.0)')
    parser.add_argument('--port', type=int, default=8000, help='HTTP port (default 8000)')
    parser.add_argument('--stream-rate', type=int, default=30,
                        help='SSE events per second per client (default 30)')
    parser.add_argument('--record', default=None, metavar='CSV',
                        help='also record the session to this CSV while streaming')
    args = parser.parse_args(argv)

    from .streamer import serve
    mpu, baro, source = _make_source(args)

    recorder = None
    if args.record:
        from .recorder import Recorder
        recorder = Recorder(mpu, args.record, interval=0.2, baro=baro).start()
        print(f'recording to {args.record}')

    print(f'streaming {source} on http://{args.host}:{args.port}/  (Ctrl-C to stop)')
    print('open that address from any browser on your network to see the viewer')
    try:
        serve(mpu, host=args.host, port=args.port, rate=args.rate,
              stream_rate=args.stream_rate, source=source, baro=baro)
    except KeyboardInterrupt:
        pass
    finally:
        if recorder is not None:
            recorder.stop()
            print(f'\n{recorder.rows} rows written to {args.record}')
    return 0


def report_main(argv=None):
    parser = argparse.ArgumentParser(
        prog='sextante-report',
        description='Analyzes a recorded CSV session and renders a Markdown report.')
    parser.add_argument('csv', help='session file written by sextante-record')
    parser.add_argument('-o', '--out', default=None,
                        help='write the report here (default: stdout)')
    parser.add_argument('--plots', default=None, metavar='DIR',
                        help='also write PNG time-series plots (requires matplotlib)')
    args = parser.parse_args(argv)

    from .report import analyze, load_session, render_markdown, write_plots
    session = load_session(args.csv)
    analysis = analyze(session)
    markdown = render_markdown(args.csv, session, analysis)

    if args.out:
        with open(args.out, 'w') as fh:
            fh.write(markdown + '\n')
        print(f'report written to {args.out}')
    else:
        print(markdown)

    if args.plots:
        try:
            for path in write_plots(session, args.plots):
                print(f'plot written to {path}')
        except RuntimeError as exc:
            print(f'warning: {exc}', file=sys.stderr)
            return 1
    return 0
