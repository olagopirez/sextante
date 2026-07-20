import time
from functools import reduce
from operator import xor

import pytest

from mpu9250 import GPS, DemoGPS
from mpu9250.gps import nmea_checksum_ok, parse_latlon


def nmea(body):
    """Builds a valid NMEA sentence, computing the checksum independently."""
    checksum = reduce(xor, (ord(c) for c in body), 0)
    return f'${body}*{checksum:02X}'


RMC = nmea('GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W')
GGA = nmea('GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,')
RMC_NO_FIX = nmea('GNRMC,,V,,,,,,,,,,N')


class TestChecksum:
    def test_accepts_a_valid_sentence(self):
        assert nmea_checksum_ok(RMC)

    def test_rejects_a_corrupted_sentence(self):
        assert not nmea_checksum_ok(RMC.replace('4807', '4808'))

    def test_rejects_garbage(self):
        assert not nmea_checksum_ok('')
        assert not nmea_checksum_ok('not nmea at all')
        assert not nmea_checksum_ok('$GPRMC,no,checksum')


class TestLatLon:
    def test_north_and_east_are_positive(self):
        assert parse_latlon('4807.038', 'N') == pytest.approx(48.1173)
        assert parse_latlon('01131.000', 'E') == pytest.approx(11.516667)

    def test_south_and_west_are_negative(self):
        assert parse_latlon('3436.000', 'S') == pytest.approx(-34.6)
        assert parse_latlon('05822.800', 'W') == pytest.approx(-58.38)

    def test_empty_field_is_none(self):
        assert parse_latlon('', 'N') is None


class TestSentenceMerging:
    def test_rmc_and_gga_merge_into_one_fix(self):
        gps = GPS()
        gps._process_line(RMC)
        gps._process_line(GGA)

        s = gps.snapshot()
        assert s.Fix is True
        assert s.Lat == pytest.approx(48.1173)
        assert s.Lon == pytest.approx(11.516667)
        assert s.SpeedKmh == pytest.approx(22.4 * 1.852)
        assert s.Course == pytest.approx(84.4)
        assert s.Sats == 8
        assert s.Hdop == pytest.approx(0.9)
        assert s.Altitude == pytest.approx(545.4)

    def test_no_sentences_means_no_snapshot(self):
        assert GPS().snapshot() is None

    def test_a_fixless_rmc_reports_fix_false_without_crashing(self):
        gps = GPS()
        gps._process_line(RMC_NO_FIX)

        s = gps.snapshot()
        assert s is not None
        assert s.Fix is False
        assert s.Lat is None

    def test_corrupted_lines_are_ignored(self):
        gps = GPS()
        gps._process_line(RMC.replace('A', 'X', 1)[:-2] + 'ZZ')
        gps._process_line('garbage')

        assert gps.snapshot() is None


class TestDemoGPS:
    def test_walks_a_plausible_track(self):
        demo = DemoGPS()
        a = demo.snapshot()
        time.sleep(0.05)
        b = demo.snapshot()

        assert a.Fix is True
        assert a.Lat == pytest.approx(42.88, abs=0.01)
        assert a.Lon == pytest.approx(-9.27, abs=0.01)
        assert 3 < a.SpeedKmh < 7
        assert 0 <= a.Course < 360
        assert (a.Lat, a.Lon) != (b.Lat, b.Lon)  # it moves
