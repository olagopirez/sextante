import csv
import time
from datetime import datetime

from mpu9250 import BaroData, MPUData, Recorder
from mpu9250.recorder import FIELDS


class StubMPU:
    """Minimal get_avg() provider for recorder tests."""

    def __init__(self):
        self.calls = 0

    def get_avg(self):
        self.calls += 1
        return MPUData(g1=1.5, g2=-2.0, g3=0.25, a1=0.01, a2=-0.02, a3=0.99,
                       m1=20.0, m2=-5.0, m3=-30.0, temp=36.5,
                       n=10, nm=5, t=datetime(2026, 7, 14, 12, 0, 0), dt=200, dtm=200)


class StubBaro:
    def read(self):
        return BaroData(pressure=100650.0, temp=24.5, altitude=56.2)


class TestRecorder:
    def test_writes_header_and_rows(self, tmp_path):
        path = tmp_path / 'session.csv'
        mpu = StubMPU()

        with Recorder(mpu, path, interval=0.05) as recorder:
            time.sleep(0.35)
        assert recorder.rows >= 3

        with open(path, newline='') as fh:
            rows = list(csv.reader(fh))
        assert rows[0] == FIELDS
        assert len(rows) - 1 == recorder.rows

        first = dict(zip(FIELDS, rows[1]))
        assert first['timestamp'] == '2026-07-14T12:00:00'
        assert float(first['g1']) == 1.5
        assert float(first['a3']) == 0.99
        assert first['n'] == '10'
        assert first['error'] == ''

    def test_stop_is_idempotent_and_final(self, tmp_path):
        mpu = StubMPU()
        recorder = Recorder(mpu, tmp_path / 's.csv', interval=0.05).start()
        time.sleep(0.12)
        recorder.stop()
        rows_after_stop = recorder.rows
        time.sleep(0.15)
        recorder.stop()
        assert recorder.rows == rows_after_stop

    def test_records_barometer_columns_when_attached(self, tmp_path):
        path = tmp_path / 'baro.csv'
        with Recorder(StubMPU(), path, interval=0.05, baro=StubBaro()) as recorder:
            time.sleep(0.15)
        assert recorder.rows >= 1

        with open(path, newline='') as fh:
            rows = list(csv.reader(fh))
        first = dict(zip(FIELDS, rows[1]))
        assert float(first['press_pa']) == 100650.0
        assert float(first['baro_temp']) == 24.5
        assert float(first['alt_m']) == 56.2

    def test_leaves_baro_columns_empty_without_a_barometer(self, tmp_path):
        path = tmp_path / 'nobaro.csv'
        with Recorder(StubMPU(), path, interval=0.05):
            time.sleep(0.12)

        with open(path, newline='') as fh:
            rows = list(csv.reader(fh))
        first = dict(zip(FIELDS, rows[1]))
        assert first['press_pa'] == ''
        assert first['alt_m'] == ''
        assert first['error'] == ''
