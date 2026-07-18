import struct

import pytest

from mpu9250 import BMP280, DemoBaro, HardwareMismatchError
from mpu9250.constants import (
    BME280_CHIP_ID,
    BMP280_ADDRESS_PRIMARY,
    BMP280_ADDRESS_SECONDARY,
    BMP280_CHIP_ID,
    BMP280_REG_CALIB,
    BMP280_REG_CHIP_ID,
    BMP280_REG_CONFIG,
    BMP280_REG_CTRL_MEAS,
    BMP280_REG_DATA,
)

# Worked example from the Bosch BMP280 datasheet (section 3.12):
# these calibration words with adc_T=519888, adc_P=415148 must yield
# T = 25.08 °C and p ≈ 100653 Pa
DATASHEET_CAL = (27504, 26435, -1000, 36477, -10685, 3024, 2855, 140, -7, 15500, -14600, 6000)
ADC_T = 519888
ADC_P = 415148


def load_bmp(fake_bus, address=BMP280_ADDRESS_PRIMARY, chip_id=BMP280_CHIP_ID,
             cal=DATASHEET_CAL, adc_p=ADC_P, adc_t=ADC_T):
    fake_bus.byte_regs[(address, BMP280_REG_CHIP_ID)] = chip_id
    packed = struct.pack('<HhhHhhhhhhhh', *cal)
    for i, byte in enumerate(packed):
        fake_bus.byte_regs[(address, BMP280_REG_CALIB + i)] = byte
    fake_bus.byte_regs[(address, BMP280_REG_DATA + 0)] = (adc_p >> 12) & 0xFF
    fake_bus.byte_regs[(address, BMP280_REG_DATA + 1)] = (adc_p >> 4) & 0xFF
    fake_bus.byte_regs[(address, BMP280_REG_DATA + 2)] = (adc_p & 0xF) << 4
    fake_bus.byte_regs[(address, BMP280_REG_DATA + 3)] = (adc_t >> 12) & 0xFF
    fake_bus.byte_regs[(address, BMP280_REG_DATA + 4)] = (adc_t >> 4) & 0xFF
    fake_bus.byte_regs[(address, BMP280_REG_DATA + 5)] = (adc_t & 0xF) << 4


class TestInitialize:
    def test_finds_and_configures_the_sensor(self, fake_bus):
        load_bmp(fake_bus)
        baro = BMP280(bus=fake_bus)

        assert baro.initialize() == BMP280_CHIP_ID
        ctrl = [(r, v) for a, r, v in fake_bus.byte_writes
                if a == BMP280_ADDRESS_PRIMARY and r in (BMP280_REG_CONFIG, BMP280_REG_CTRL_MEAS)]
        assert (BMP280_REG_CONFIG, 0x10) in ctrl     # IIR 16, 0.5 ms standby
        assert (BMP280_REG_CTRL_MEAS, 0x57) in ctrl  # x2/x16 oversampling, normal mode

    def test_probes_the_secondary_address(self, fake_bus):
        load_bmp(fake_bus, address=BMP280_ADDRESS_SECONDARY)

        baro = BMP280(bus=fake_bus)
        baro.initialize()

        assert baro.read().Temp == pytest.approx(25.08, abs=0.1)

    def test_accepts_a_bme280(self, fake_bus):
        load_bmp(fake_bus, chip_id=BME280_CHIP_ID)

        assert BMP280(bus=fake_bus).initialize() == BME280_CHIP_ID

    def test_rejects_an_unknown_chip(self, fake_bus):
        fake_bus.byte_regs[(BMP280_ADDRESS_PRIMARY, BMP280_REG_CHIP_ID)] = 0x42

        with pytest.raises(HardwareMismatchError, match='BMP280'):
            BMP280(bus=fake_bus).initialize()


class TestCompensation:
    def test_matches_the_datasheet_worked_example(self, fake_bus):
        load_bmp(fake_bus)
        baro = BMP280(bus=fake_bus)
        baro.initialize()

        d = baro.read()

        assert d.Temp == pytest.approx(25.08, abs=0.05)
        assert d.Pressure == pytest.approx(100653.0, abs=10.0)

    def test_altitude_against_the_standard_atmosphere(self, fake_bus):
        load_bmp(fake_bus)
        baro = BMP280(bus=fake_bus)
        baro.initialize()

        assert baro.altitude(101325.0) == pytest.approx(0.0, abs=0.01)
        assert baro.altitude(89874.6) == pytest.approx(1000.0, abs=5.0)  # ISA 1000 m

    def test_altitude_uses_the_configured_sea_level(self, fake_bus):
        load_bmp(fake_bus)
        baro = BMP280(bus=fake_bus, sea_level_pa=100653.0)
        baro.initialize()

        # with QNH set to the ambient pressure, the datasheet example sits at ~0 m
        assert baro.read().Altitude == pytest.approx(0.0, abs=1.0)


class TestDemoBaro:
    def test_produces_plausible_readings(self):
        d = DemoBaro().read()

        assert 100000 < d.Pressure < 102000
        assert 5 < d.Altitude < 20
        assert 23 < d.Temp < 26
