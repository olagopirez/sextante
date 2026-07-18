"""Driver for the Bosch BMP280 barometric pressure/temperature sensor.

The Stratux AHRS board pairs one with the MPU-9250; standalone breakouts sit
at I2C 0x76 or 0x77. Compensation follows the double-precision formulas from
the Bosch datasheet (section 3.11.3), and altitude uses the international
barometric formula against a configurable sea-level pressure (QNH).
"""

import math
import struct
import time
from datetime import datetime

from .constants import (
    BME280_CHIP_ID,
    BMP280_ADDRESS_PRIMARY,
    BMP280_ADDRESS_SECONDARY,
    BMP280_CHIP_ID,
    BMP280_REG_CALIB,
    BMP280_REG_CHIP_ID,
    BMP280_REG_CONFIG,
    BMP280_REG_CTRL_MEAS,
    BMP280_REG_DATA,
    BMP280_REG_RESET,
    BMP280_SOFT_RESET,
)
from .data import BaroData
from .driver import HardwareMismatchError

SEA_LEVEL_PA = 101325.0


class BMP280:
    def __init__(self, address=None, bus=None, sea_level_pa=SEA_LEVEL_PA):
        if bus is None:
            import smbus2  # imported lazily so the package works without hardware
            bus = smbus2.SMBus(1)

        self.__bus = bus
        self.__address = address
        self.__cal = None
        self.sea_level_pa = float(sea_level_pa)
        self.chip_id = None

    def __probe(self):
        candidates = [self.__address] if self.__address is not None else \
            [BMP280_ADDRESS_PRIMARY, BMP280_ADDRESS_SECONDARY]
        seen = {}
        for addr in candidates:
            try:
                chip_id = self.__bus.read_byte_data(addr, BMP280_REG_CHIP_ID)
            except OSError:
                continue  # nothing answering at this address
            seen[addr] = chip_id
            if chip_id in (BMP280_CHIP_ID, BME280_CHIP_ID):
                return addr, chip_id

        detail = ', '.join(f'0x{a:02X} answered id 0x{i:02X}' for a, i in seen.items()) \
            or 'no device answered'
        raise HardwareMismatchError(
            f'no BMP280/BME280 found (expected chip id 0x{BMP280_CHIP_ID:02X} or '
            f'0x{BME280_CHIP_ID:02X}; {detail})')

    def initialize(self):
        """
        Finds the sensor, loads its factory calibration and starts continuous
        measurements. Returns the chip id (0x58 BMP280, 0x60 BME280).

        Raises HardwareMismatchError when no BMP280/BME280 answers.
        """
        self.__address, self.chip_id = self.__probe()

        self.__bus.write_byte_data(self.__address, BMP280_REG_RESET, BMP280_SOFT_RESET)
        time.sleep(5e-3)

        raw = bytes(self.__bus.read_i2c_block_data(self.__address, BMP280_REG_CALIB, 24))
        self.__cal = struct.unpack('<HhhHhhhhhhhh', raw)

        # Handheld/indoor-navigation profile: 0.5 ms standby, IIR filter 16,
        # osrs_t x2, osrs_p x16, normal (continuous) mode
        self.__bus.write_byte_data(self.__address, BMP280_REG_CONFIG, 0x10)
        self.__bus.write_byte_data(self.__address, BMP280_REG_CTRL_MEAS, 0x57)
        time.sleep(0.1)  # let the first conversion land

        return self.chip_id

    def read(self):
        """Returns a BaroData with compensated pressure (Pa), temperature (°C)
        and barometric altitude (m) against the configured sea-level pressure."""
        data = self.__bus.read_i2c_block_data(self.__address, BMP280_REG_DATA, 6)
        adc_p = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        adc_t = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)

        t1, t2, t3, p1, p2, p3, p4, p5, p6, p7, p8, p9 = self.__cal

        # Temperature (datasheet double-precision compensation)
        var1 = (adc_t / 16384.0 - t1 / 1024.0) * t2
        var2 = ((adc_t / 131072.0 - t1 / 8192.0) ** 2) * t3
        t_fine = var1 + var2
        temp = t_fine / 5120.0

        # Pressure
        var1 = t_fine / 2.0 - 64000.0
        var2 = var1 * var1 * p6 / 32768.0
        var2 = var2 + var1 * p5 * 2.0
        var2 = var2 / 4.0 + p4 * 65536.0
        var1 = (p3 * var1 * var1 / 524288.0 + p2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * p1
        if var1 == 0:
            pressure = 0.0
        else:
            pressure = 1048576.0 - adc_p
            pressure = (pressure - var2 / 4096.0) * 6250.0 / var1
            var1 = p9 * pressure * pressure / 2147483648.0
            var2 = pressure * p8 / 32768.0
            pressure = pressure + (var1 + var2 + p7) / 16.0

        return BaroData(
            pressure=pressure,
            temp=temp,
            altitude=self.altitude(pressure),
            t=datetime.now(),
        )

    def altitude(self, pressure_pa):
        """International barometric formula, in meters above sea_level_pa."""
        if pressure_pa <= 0:
            return 0.0
        return 44330.0 * (1.0 - math.pow(pressure_pa / self.sea_level_pa, 1 / 5.255))
