"""Live streaming: samples the IMU, fuses attitude, serves a web viewer + SSE.

Pure standard library — the Raspberry Pi serves both the data stream
(``/events``, Server-Sent Events) and the viewer app itself (``/``), so the
"app" on the PC is just a browser pointed at the Pi.
"""

import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib import resources

from .fusion import MahonyAHRS, reconstruct_az

DEG = math.pi / 180


class StreamHub(threading.Thread):
    """Samples ``mpu.mpuDate`` at a fixed rate, runs the Mahony filter and
    keeps the latest JSON-ready payload for any number of SSE clients."""

    def __init__(self, mpu, rate=50, source='mpu9250', baro=None, fix_az=0, gps=None):
        threading.Thread.__init__(self)
        self.daemon = True
        self.__mpu = mpu
        self.__baro = baro
        self.__gps = gps
        self.__fix_az = fix_az  # 0 = off; ±1 = sign of a healthy vertical axis
        self.__baro_every = max(1, rate // 10)  # ~10 Hz is plenty for pressure
        self.__period = 1.0 / rate
        self.__stop = threading.Event()
        self.fusion = MahonyAHRS()
        self.source = source
        self.rate = rate
        self.latest = None
        self.__lock = threading.Lock()

    def run(self):
        prev = time.monotonic()
        tick = 0
        baro_data = None
        while not self.__stop.is_set():
            d = self.__mpu.mpuDate
            now = time.monotonic()
            dt = min(max(now - prev, 1e-4), 0.2)
            prev = now

            if self.__baro is not None and tick % self.__baro_every == 0:
                try:
                    baro_data = self.__baro.read()
                except OSError:
                    pass  # transient bus error: keep the previous reading
            tick += 1

            mag_ok = d.NM > 0 and not (d.M1 == 0 and d.M2 == 0 and d.M3 == 0)
            az = float(d.A3)
            if self.__fix_az:
                az = reconstruct_az(float(d.A1), float(d.A2), self.__fix_az)
            q = self.fusion.update(
                float(d.G1) * DEG, float(d.G2) * DEG, float(d.G3) * DEG,
                float(d.A1), float(d.A2), az,
                float(d.M1) if mag_ok else None,
                float(d.M2) if mag_ok else None,
                float(d.M3) if mag_ok else None,
                dt=dt,
            )
            roll, pitch, yaw = self.fusion.euler()
            payload = {
                't': d.T.isoformat(),
                'src': self.source,
                'hz': self.rate,
                'g': [round(float(d.G1), 3), round(float(d.G2), 3), round(float(d.G3), 3)],
                'a': [round(float(d.A1), 4), round(float(d.A2), 4), round(float(d.A3), 4)],
                'm': [round(float(d.M1), 2), round(float(d.M2), 2), round(float(d.M3), 2)],
                'temp': round(float(d.Temp), 2),
                'q': [round(c, 5) for c in q],
                'e': [round(roll / DEG, 2), round(pitch / DEG, 2), round(yaw / DEG, 2)],
            }
            if baro_data is not None:
                payload['press'] = round(baro_data.Pressure / 100.0, 2)  # hPa
                payload['alt'] = round(baro_data.Altitude, 2)
                payload['btemp'] = round(baro_data.Temp, 2)
            if self.__gps is not None:
                gfix = self.__gps.snapshot()
                if gfix is not None:
                    payload['fix'] = 1 if gfix.Fix else 0
                    payload['sats'] = gfix.Sats
                    if gfix.Fix and gfix.Lat is not None:
                        payload['lat'] = round(gfix.Lat, 6)
                        payload['lon'] = round(gfix.Lon, 6)
                        payload['spd'] = round(gfix.SpeedKmh, 1)
                        payload['crs'] = round(gfix.Course, 1)
                        payload['galt'] = round(gfix.Altitude, 1)
            with self.__lock:
                self.latest = payload
            self.__stop.wait(self.__period)

    def snapshot(self):
        with self.__lock:
            return self.latest

    def stop(self):
        self.__stop.set()


def _viewer_html():
    return resources.files('mpu9250.web').joinpath('viewer.html').read_bytes()


class _Handler(BaseHTTPRequestHandler):
    hub = None
    stream_period = 1 / 30

    def log_message(self, *args):
        pass

    def __send(self, code, ctype, body):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path == '/':
            self.__send(200, 'text/html; charset=utf-8', _viewer_html())
        elif path == '/status':
            body = json.dumps({'source': self.hub.source, 'rate': self.hub.rate}).encode()
            self.__send(200, 'application/json', body)
        elif path == '/events':
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            try:
                while True:
                    payload = self.hub.snapshot()
                    if payload is not None:
                        data = json.dumps(payload)
                        self.wfile.write(f'data: {data}\n\n'.encode())
                        self.wfile.flush()
                    time.sleep(self.stream_period)
            except (BrokenPipeError, ConnectionResetError, OSError):
                return
        else:
            self.__send(404, 'text/plain', b'not found')


def create_server(mpu, host='0.0.0.0', port=8000, rate=50, stream_rate=30, source='mpu9250', baro=None, fix_az=0, gps=None):
    """Starts the sampling hub and returns (httpd, hub); caller serves/shuts down."""
    hub = StreamHub(mpu, rate=rate, source=source, baro=baro, fix_az=fix_az, gps=gps)
    hub.start()

    handler = type('Handler', (_Handler,), {'hub': hub, 'stream_period': 1.0 / stream_rate})
    httpd = ThreadingHTTPServer((host, port), handler)
    httpd.daemon_threads = True
    return httpd, hub


def serve(mpu, host='0.0.0.0', port=8000, rate=50, stream_rate=30, source='mpu9250', baro=None, fix_az=0, gps=None):
    """Blocks serving the viewer and the SSE stream until interrupted."""
    httpd, hub = create_server(mpu, host, port, rate, stream_rate, source, baro=baro, fix_az=fix_az, gps=gps)
    try:
        httpd.serve_forever()
    finally:
        hub.stop()
        httpd.server_close()
