"""Synthetic MPU9250 stand-in producing smooth, physically consistent motion.

Lets the whole pipeline — recorder, streamer, viewer, reports — run on any
machine without hardware: gyro, accel and mag are all derived from one
analytic orientation path, in driver units (°/s, g, µT).
"""

import math
import time
from datetime import datetime

from .data import BaroData, MPUData
from .fusion import q_conjugate, q_from_euler, q_multiply, q_rotate

# Earth frame (z up): gravity specific force and a ~38 µT field with dip
_GRAVITY = (0.0, 0.0, 1.0)
_MAG = (22.0, 0.0, -31.0)


def _attitude(t):
    """The scripted orientation path: gentle roll/pitch sways plus a slow yaw."""
    roll = 0.55 * math.sin(0.31 * t)
    pitch = 0.38 * math.sin(0.23 * t + 1.1)
    yaw = 0.25 * t
    return q_from_euler(roll, pitch, yaw)


class DemoMPU:
    """Drop-in stand-in for MPU9250: same reading surface, synthetic data."""

    def __init__(self, rate=50, noisy=True):
        self.__rate = rate
        self.__noisy = noisy
        self.__t0 = time.monotonic()
        self.mpuCalDate = None  # parity attribute; unused

    def initialize(self, check_hardware=True):
        pass

    def self_check(self):
        return 0x71

    def __sample(self, t):
        q = _attitude(t)

        # Body-frame gyro from a finite quaternion difference, in °/s
        eps = 1e-3
        dq = q_multiply(q_conjugate(q), _attitude(t + eps))
        gx = 2 * dq[1] / eps * 180 / math.pi
        gy = 2 * dq[2] / eps * 180 / math.pi
        gz = 2 * dq[3] / eps * 180 / math.pi

        # World references rotated into the body frame
        qi = q_conjugate(q)
        a = q_rotate(qi, _GRAVITY)
        m = q_rotate(qi, _MAG)

        n = 1.0
        if self.__noisy:
            # deterministic pseudo-noise, cheap and repeatable
            n = math.sin(t * 997.1)
        return (
            gx + 0.3 * n, gy + 0.3 * math.sin(t * 883.3), gz + 0.3 * math.sin(t * 761.7),
            a[0] + 0.004 * n, a[1] + 0.004 * math.sin(t * 653.9), a[2] + 0.004 * math.sin(t * 547.3),
            m[0] + 0.25 * n, m[1] + 0.25 * math.sin(t * 431.1), m[2] + 0.25 * math.sin(t * 389.7),
            36.4 + 0.3 * math.sin(t / 9),
        )

    def attitude(self, t=None):
        """The ground-truth orientation quaternion (used by tests)."""
        if t is None:
            t = time.monotonic() - self.__t0
        return _attitude(t)

    @property
    def mpuDate(self):
        t = time.monotonic() - self.__t0
        g1, g2, g3, a1, a2, a3, m1, m2, m3, temp = self.__sample(t)
        now = datetime.now()
        return MPUData(g1=g1, g2=g2, g3=g3, a1=a1, a2=a2, a3=a3,
                       m1=m1, m2=m2, m3=m3, temp=temp, t=now, tm=now, n=1, nm=1)

    def calibrate_gyro(self, duration=2.0):
        return (0.0, 0.0, 0.0)

    def get_avg(self):
        d = self.mpuDate
        d.N = self.__rate
        d.NM = min(self.__rate, 100)
        d.DT = 1000.0 / self.__rate * d.N
        d.DTM = d.DT
        return d


class DemoBaro:
    """Synthetic BMP280 stand-in: a gentle altitude sway around ~12 m."""

    def __init__(self, sea_level_pa=101325.0):
        self.__t0 = time.monotonic()
        self.sea_level_pa = float(sea_level_pa)
        self.chip_id = 0x58

    def initialize(self):
        return self.chip_id

    def read(self):
        t = time.monotonic() - self.__t0
        altitude = 12.0 + 4.0 * math.sin(t / 9) + 0.15 * math.sin(t * 431.7)
        pressure = self.sea_level_pa * (1.0 - altitude / 44330.0) ** 5.255
        temp = 24.5 + 0.2 * math.sin(t / 13)
        return BaroData(pressure=pressure, temp=temp, altitude=altitude,
                        t=datetime.now())


class DemoGPS:
    """Synthetic GPS: a slow walk in a ~50 m circle off Cape Finisterre."""

    LAT0, LON0 = 42.8806, -9.2711

    def __init__(self):
        self.__t0 = time.monotonic()

    def start(self):
        return self

    def stop(self):
        pass

    def snapshot(self):
        from .data import GPSData
        t = time.monotonic() - self.__t0
        angle = t * (2 * math.pi / 240)  # one lap every 4 minutes
        radius = 0.00045                 # ~50 m in degrees of latitude
        lat = self.LAT0 + radius * math.sin(angle)
        lon = self.LON0 + radius * math.cos(angle) / math.cos(math.radians(self.LAT0))
        return GPSData(
            lat=lat, lon=lon,
            speed_kmh=4.7 + 0.2 * math.sin(t * 1.3),
            course=(90.0 - math.degrees(angle)) % 360.0,
            sats=9, hdop=0.9,
            altitude=63.0 + 0.5 * math.sin(t / 7),
            fix=True,
            t=datetime.now(),
        )
