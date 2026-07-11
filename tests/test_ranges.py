import pytest

from mpu9250.constants import (
    BITS_DLPF_CFG_5HZ,
    BITS_DLPF_CFG_20HZ,
    BITS_DLPF_CFG_42HZ,
    BITS_DLPF_CFG_98HZ,
    BITS_FS_2G,
    BITS_FS_4G,
    BITS_FS_8G,
    BITS_FS_16G,
    BITS_FS_250DPS,
    BITS_FS_500DPS,
    BITS_FS_1000DPS,
    BITS_FS_2000DPS,
)
from mpu9250.ranges import AccelRange, GyroRange, LPF


class TestAccelRange:
    def test_bits_match_the_selected_full_scale(self):
        assert AccelRange.RANGE_2_G.get_bits() == BITS_FS_2G
        assert AccelRange.RANGE_4_G.get_bits() == BITS_FS_4G
        assert AccelRange.RANGE_8_G.get_bits() == BITS_FS_8G
        assert AccelRange.RANGE_16_G.get_bits() == BITS_FS_16G

    def test_scale_converts_full_scale_counts_to_g(self):
        assert AccelRange.RANGE_2_G.get_scale() == pytest.approx(2 / 32767)
        assert AccelRange.RANGE_16_G.get_scale() == pytest.approx(16 / 32767)


class TestGyroRange:
    def test_bits_match_the_selected_full_scale(self):
        assert GyroRange.RANGE_250_DPS.get_bits() == BITS_FS_250DPS
        assert GyroRange.RANGE_500_DPS.get_bits() == BITS_FS_500DPS
        assert GyroRange.RANGE_1000_DPS.get_bits() == BITS_FS_1000DPS
        assert GyroRange.RANGE_2000_DPS.get_bits() == BITS_FS_2000DPS

    def test_scale_converts_full_scale_counts_to_dps(self):
        assert GyroRange.RANGE_250_DPS.get_scale() == pytest.approx(250 / 32767)
        assert GyroRange.RANGE_2000_DPS.get_scale() == pytest.approx(2000 / 32767)


class TestLPF:
    @pytest.mark.parametrize("rate,expected_bits", [
        (50, BITS_DLPF_CFG_20HZ),
        (100, BITS_DLPF_CFG_42HZ),
        (200, BITS_DLPF_CFG_98HZ),
        (10, BITS_DLPF_CFG_5HZ),
    ])
    def test_cutoff_is_half_the_sample_rate(self, rate, expected_bits):
        lpf = LPF(rate=rate)

        assert lpf.get_gyro_bits() == expected_bits
        assert lpf.get_accel_bits() == expected_bits

    def test_sample_rate_divider(self):
        assert LPF(rate=50).get_simple_rate() == 19   # 1000 / (1 + 19) = 50 Hz
        assert LPF(rate=100).get_simple_rate() == 9

    def test_rate_is_kept(self):
        assert LPF(rate=50).get_rate() == 50
