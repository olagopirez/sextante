import time
from datetime import datetime

from mpu9250 import MPUCalData, MPUData


class TestMPUData:
    def test_defaults_are_zeroed_and_valid(self):
        d = MPUData()

        assert d.G1 == 0.0
        assert d.A3 == 0.0
        assert d.M2 == 0.0
        assert d.Temp == 0.0
        assert d.MsgError is None
        assert isinstance(d.T, datetime)

    def test_default_timestamps_are_fresh_per_instance(self):
        first = MPUData()
        time.sleep(0.01)
        second = MPUData()

        assert second.T > first.T

    def test_get_json_exposes_every_field(self):
        d = MPUData(g1=1.5, a3=0.98, temp=25.0, n=10)
        j = d.get_json()

        assert j['G1'] == 1.5
        assert j['A3'] == 0.98
        assert j['Temp'] == 25.0
        assert j['N'] == 10
        assert set(j) == {
            'G1', 'G2', 'G3', 'A1', 'A2', 'A3', 'M1', 'M2', 'M3',
            'Temp', 'T', 'TM', 'DT', 'DTM', 'N', 'NM', 'MsgError',
        }


class TestMPUCalData:
    def test_biases_default_to_zero(self):
        c = MPUCalData()

        assert (c.G01, c.G02, c.G03) == (0.0, 0.0, 0.0)
        assert (c.A01, c.A02, c.A03) == (0.0, 0.0, 0.0)
        assert (c.M01, c.M02, c.M03) == (0.0, 0.0, 0.0)

    def test_rescaling_matrix_defaults_to_identity(self):
        c = MPUCalData()

        assert (c.Ms11, c.Ms22, c.Ms33) == (1.0, 1.0, 1.0)
        assert (c.Ms12, c.Ms13, c.Ms21, c.Ms23, c.Ms31, c.Ms32) == (0.0,) * 6
