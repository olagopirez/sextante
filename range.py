import numpy as np
import json

from Constants import *


class _Range:
    def __init__(self, bits, scale):
        self.__bits = bits
        self.__scale = scale

    def get_bits(self):
        return self.__bits

    def get_scale(self):
        return self.__scale

    def __str__(self):
        return json.dumps({
            "bits": self.__bits,
            "scale": self.__scale,
        })


class AccelRange:
    """
    Sets the accelerometer sensitivity of the MPU9250;
    it must be one of the following values:
    2, 4, 8, 16, all in G (gravity).
    """

    def __init__(self):
        pass

    RANGE_2_G = _Range(
        bits=BITS_FS_2G,
        scale=np.float64(2) / np.float64(np.iinfo(np.int16).max)
    )

    RANGE_4_G = _Range(
        bits=BITS_FS_4G,
        scale=np.float64(4) / np.float64(np.iinfo(np.int16).max)
    )
    RANGE_8_G = _Range(
        bits=BITS_FS_8G,
        scale=np.float64(8) / np.float64(np.iinfo(np.int16).max)
    )
    RANGE_16_G = _Range(
        bits=BITS_FS_16G,
        scale=np.float64(16) / np.float64(np.iinfo(np.int16).max)
    )


class GyroRange:
    """
    Sets the gyro sensitivity of the MPU9250;
    it must be one of the following values: 250, 500, 1000, 2000 (all in degree/s).
    """

    def __init__(self):
        pass

    RANGE_250_DPS = _Range(
        bits=BITS_FS_250DPS,
        scale=np.float64(250) / np.float64(np.iinfo(np.int16).max)
    )
    RANGE_500_DPS = _Range(
        bits=BITS_FS_500DPS,
        scale=np.float64(500) / np.float64(np.iinfo(np.int16).max)
    )
    RANGE_1000_DPS = _Range(
        bits=BITS_FS_1000DPS,
        scale=np.float64(1000) / np.float64(np.iinfo(np.int16).max)
    )
    RANGE_2000_DPS = _Range(
        bits=BITS_FS_2000DPS,
        scale=np.float64(2000) / np.float64(np.iinfo(np.int16).max)
    )


class LPF:
    """
    Sets the low pass filter for the gyro.
    """

    def __init__(self, rate=None):
        if not rate:
            pass

        self.__rate = rate
        simple_rate = np.byte(1000 / rate - 1)

        # LPF cutoff is chosen from the sample rate in Hz, not from the divider byte
        self.__gyro_bits = self.__get_gyro_rate(rate_byte=rate >> 1)
        self.__accel_bits = self.__get_accel_rate(rate_byte=rate >> 1)

        self.__simple_rate = simple_rate

    @staticmethod
    def __get_gyro_rate(rate_byte):
        if rate_byte >= 188:
            return BITS_DLPF_CFG_188HZ
        elif rate_byte >= 98:
            return BITS_DLPF_CFG_98HZ
        elif rate_byte >= 42:
            return BITS_DLPF_CFG_42HZ
        elif rate_byte >= 20:
            return BITS_DLPF_CFG_20HZ
        elif rate_byte >= 10:
            return BITS_DLPF_CFG_10HZ
        else:
            return BITS_DLPF_CFG_5HZ

    @staticmethod
    def __get_accel_rate(rate_byte):
        if rate_byte >= 218:
            return BITS_DLPF_CFG_188HZ
        elif rate_byte >= 99:
            return BITS_DLPF_CFG_98HZ
        elif rate_byte >= 45:
            return BITS_DLPF_CFG_42HZ
        elif rate_byte >= 21:
            return BITS_DLPF_CFG_20HZ
        elif rate_byte >= 10:
            return BITS_DLPF_CFG_10HZ
        else:
            return BITS_DLPF_CFG_5HZ

    def get_gyro_bits(self):
        return self.__gyro_bits

    def get_accel_bits(self):
        return self.__accel_bits

    def get_rate(self):
        return self.__rate

    def get_simple_rate(self):
        return self.__simple_rate

    def get_simple_rate_byte(self):
        return np.byte(self.__simple_rate)
