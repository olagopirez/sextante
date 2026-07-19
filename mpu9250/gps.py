"""GPS over serial NMEA — the uBlox receiver on BerryGPS-IMU boards.

Pure standard library: the serial port is configured with ``termios`` and the
NMEA sentences (RMC for position/speed/course, GGA for fix quality/satellites/
altitude) are parsed by hand, checksum-validated. A daemon thread keeps the
latest merged state; consumers call ``snapshot()``.
"""

import os
import threading
from datetime import datetime

from .data import GPSData

KNOTS_TO_KMH = 1.852


def nmea_checksum_ok(line):
    """Validates ``$body*HH`` — HH is the XOR of every byte in body."""
    if not line.startswith('$') or '*' not in line:
        return False
    body, _, tail = line[1:].partition('*')
    try:
        expected = int(tail[:2], 16)
    except ValueError:
        return False
    check = 0
    for ch in body:
        check ^= ord(ch)
    return check == expected


def parse_latlon(value, hemisphere):
    """NMEA ddmm.mmmm / dddmm.mmmm plus N/S/E/W into signed decimal degrees."""
    if not value:
        return None
    raw = float(value)
    degrees = int(raw // 100)
    result = degrees + (raw - degrees * 100) / 60.0
    return -result if hemisphere in ('S', 'W') else result


def _open_serial(device, baud):
    import termios  # POSIX-only; imported here so the parser works anywhere
    speeds = {
        4800: termios.B4800, 9600: termios.B9600, 19200: termios.B19200,
        38400: termios.B38400, 57600: termios.B57600, 115200: termios.B115200,
    }
    if baud not in speeds:
        raise ValueError(f'unsupported baud rate {baud}')

    fd = os.open(device, os.O_RDONLY | os.O_NOCTTY)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0                      # iflag: raw input
    attrs[1] = 0                      # oflag
    attrs[2] = termios.CREAD | termios.CLOCAL | termios.CS8  # cflag: 8N1
    attrs[3] = 0                      # lflag: no canonical mode, no echo
    attrs[4] = attrs[5] = speeds[baud]
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return os.fdopen(fd, 'rb')


class GPS(threading.Thread):
    """Reads NMEA from a serial device and keeps the latest merged fix."""

    def __init__(self, device='/dev/serial0', baud=9600):
        threading.Thread.__init__(self)
        self.daemon = True
        self.__device = device
        self.__baud = baud
        self.__stop = threading.Event()
        self.__lock = threading.Lock()
        self.__has_data = False
        self.error = None

        self.__lat = None
        self.__lon = None
        self.__speed_kmh = 0.0
        self.__course = 0.0
        self.__sats = 0
        self.__hdop = 0.0
        self.__altitude = 0.0
        self.__fix = False

    def run(self):
        try:
            fh = self.__open()
        except (OSError, ValueError) as exc:
            self.error = str(exc)
            return
        with fh:
            while not self.__stop.is_set():
                raw = fh.readline()
                if not raw:
                    continue
                self._process_line(raw.decode('ascii', 'replace').strip())

    def __open(self):
        return _open_serial(self.__device, self.__baud)

    def _process_line(self, line):
        """Merges one NMEA sentence into the state. Safe on garbage input."""
        if not nmea_checksum_ok(line):
            return
        parts = line[1:line.index('*')].split(',')
        kind = parts[0][2:5]  # sentence type, whatever the talker (GP/GN/GL...)

        if kind == 'RMC' and len(parts) >= 9:
            with self.__lock:
                self.__fix = parts[2] == 'A'
                if self.__fix:
                    self.__lat = parse_latlon(parts[3], parts[4])
                    self.__lon = parse_latlon(parts[5], parts[6])
                    self.__speed_kmh = float(parts[7]) * KNOTS_TO_KMH if parts[7] else 0.0
                    self.__course = float(parts[8]) if parts[8] else 0.0
                self.__has_data = True
        elif kind == 'GGA' and len(parts) >= 10:
            with self.__lock:
                self.__sats = int(parts[7]) if parts[7] else 0
                self.__hdop = float(parts[8]) if parts[8] else 0.0
                self.__altitude = float(parts[9]) if parts[9] else 0.0
                self.__has_data = True

    def snapshot(self):
        """Returns the latest merged GPSData, or None before any sentence."""
        with self.__lock:
            if not self.__has_data:
                return None
            return GPSData(lat=self.__lat, lon=self.__lon,
                           speed_kmh=self.__speed_kmh, course=self.__course,
                           sats=self.__sats, hdop=self.__hdop,
                           altitude=self.__altitude, fix=self.__fix,
                           t=datetime.now())

    def stop(self):
        self.__stop.set()
