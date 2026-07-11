import threading
from datetime import datetime, timedelta

import pytest

from mpu9250 import AccelRange, GyroRange, HardwareMismatchError, MPUData
from mpu9250.constants import (
    AK8963_ASAX,
    AK8963_ASAY,
    AK8963_ASAZ,
    AK8963_CNTL1,
    AK8963_Device_ID,
    AK8963_I2C_ADDR,
    AK8963_WIA,
    AKM_16BIT,
    AKM_CONTINUOUS_100HZ,
    AKM_FUSE_ROM_ACCESS,
    AKM_POWER_DOWN,
    BITS_FS_8G,
    BITS_FS_500DPS,
    CFG_MOTION_BIAS,
    MPU6050_ID,
    MPU6500_ID,
    MPU9250_ID,
    MPU9255_ID,
    MPU_ADDRESS,
    MPUREG_ACCEL_CONFIG,
    MPUREG_BANK_SEL,
    MPUREG_EXT_SENS_DATA_00,
    MPUREG_EXT_SENS_DATA_01,
    MPUREG_EXT_SENS_DATA_03,
    MPUREG_EXT_SENS_DATA_05,
    MPUREG_EXT_SENS_DATA_07,
    MPUREG_GYRO_CONFIG,
    MPUREG_INT_PIN_CFG,
    MPUREG_MEM_R_W,
    MPUREG_WHOAMI,
)
from mpu9250.driver import _be_word_to_int16, _to_int16


class TestWordEndianness:
    def test_swaps_smbus_little_endian_into_big_endian_value(self):
        # +1 g on accel Z at rest: registers hold 0x40 0x00, SMBus delivers 0x0040
        assert _be_word_to_int16(0x0040) == 16384

    def test_preserves_sign(self):
        # registers hold 0xFF 0xF0 (-16), SMBus delivers 0xF0FF
        assert _be_word_to_int16(0xF0FF) == -16

    def test_extremes(self):
        assert _be_word_to_int16(0x0000) == 0
        assert _be_word_to_int16(0xFF7F) == 32767
        assert _be_word_to_int16(0x0080) == -32768

    def test_little_endian_negative_values(self):
        # AK8963 words arrive already little-endian; negative raw words must not overflow
        assert _to_int16(0xFFF0) == -16
        assert _to_int16(0x8000) == -32768
        assert _to_int16(0x7FFF) == 32767


class TestSelfCheck:
    @staticmethod
    def _identify_as(fake_bus, whoami, wia=AK8963_Device_ID):
        fake_bus.byte_regs[(MPU_ADDRESS, MPUREG_WHOAMI)] = whoami
        fake_bus.byte_regs[(AK8963_I2C_ADDR, AK8963_WIA)] = wia

    def test_passes_on_a_genuine_mpu9250(self, mpu, fake_bus):
        self._identify_as(fake_bus, MPU9250_ID)

        assert mpu.self_check() == MPU9250_ID

    def test_accepts_the_mpu9255_sibling(self, mpu, fake_bus):
        self._identify_as(fake_bus, MPU9255_ID)

        assert mpu.self_check() == MPU9255_ID

    def test_rejects_a_relabeled_mpu6500(self, mpu, fake_bus):
        self._identify_as(fake_bus, MPU6500_ID)

        with pytest.raises(HardwareMismatchError, match='MPU-6500'):
            mpu.self_check()

    def test_rejects_an_mpu6050(self, mpu, fake_bus):
        self._identify_as(fake_bus, MPU6050_ID)

        with pytest.raises(HardwareMismatchError, match='MPU-6050'):
            mpu.self_check()

    def test_rejects_a_silent_magnetometer(self, mpu, fake_bus):
        self._identify_as(fake_bus, MPU9250_ID, wia=0x00)

        with pytest.raises(HardwareMismatchError, match='magnetometer'):
            mpu.self_check()

    def test_leaves_bypass_mode_disabled(self, mpu, fake_bus):
        self._identify_as(fake_bus, MPU9250_ID)

        mpu.self_check()

        int_pin_writes = [v for a, r, v in fake_bus.byte_writes
                          if a == MPU_ADDRESS and r == MPUREG_INT_PIN_CFG]
        assert int_pin_writes[-1] == 0x00


class TestRangeConfiguration:
    def test_accel_range_goes_to_accel_config_register(self, mpu, fake_bus):
        mpu.set_accel_range(AccelRange.RANGE_8_G)

        assert (MPU_ADDRESS, MPUREG_ACCEL_CONFIG, BITS_FS_8G) in fake_bus.byte_writes
        assert [w for w in fake_bus.byte_writes if w[1] == MPUREG_GYRO_CONFIG] == []

    def test_gyro_range_goes_to_gyro_config_register(self, mpu, fake_bus):
        mpu.set_gyro_range(GyroRange.RANGE_500_DPS)

        assert (MPU_ADDRESS, MPUREG_GYRO_CONFIG, BITS_FS_500DPS) in fake_bus.byte_writes


class TestMagSetup:
    def test_reads_fuse_rom_from_the_ak8963_address(self, mpu, fake_bus):
        fake_bus.byte_regs[(AK8963_I2C_ADDR, AK8963_ASAX)] = 170
        fake_bus.byte_regs[(AK8963_I2C_ADDR, AK8963_ASAY)] = 178
        fake_bus.byte_regs[(AK8963_I2C_ADDR, AK8963_ASAZ)] = 166

        mpu._MPU9250__mag_setup()

        scale = 9830.0 / 65536.0
        assert mpu.mcal1 == pytest.approx((170 + 128) / 256.0 * scale)
        assert mpu.mcal2 == pytest.approx((178 + 128) / 256.0 * scale)
        assert mpu.mcal3 == pytest.approx((166 + 128) / 256.0 * scale)

    def test_leaves_the_ak8963_in_continuous_16bit_mode(self, mpu, fake_bus):
        mpu._MPU9250__mag_setup()

        cntl1_writes = [v for a, r, v in fake_bus.byte_writes
                        if a == AK8963_I2C_ADDR and r == AK8963_CNTL1]
        assert cntl1_writes == [AKM_POWER_DOWN, AKM_FUSE_ROM_ACCESS, AKM_POWER_DOWN,
                                AKM_16BIT | AKM_CONTINUOUS_100HZ]


class TestMagSampling:
    @staticmethod
    def _load_mag(fake_bus, hx=0, hy=0, hz=0, drdy=True, hofl=False):
        fake_bus.byte_regs[(MPU_ADDRESS, MPUREG_EXT_SENS_DATA_00)] = 0x01 if drdy else 0x00
        fake_bus.word_regs[(MPU_ADDRESS, MPUREG_EXT_SENS_DATA_01)] = hx & 0xFFFF
        fake_bus.word_regs[(MPU_ADDRESS, MPUREG_EXT_SENS_DATA_03)] = hy & 0xFFFF
        fake_bus.word_regs[(MPU_ADDRESS, MPUREG_EXT_SENS_DATA_05)] = hz & 0xFFFF
        fake_bus.byte_regs[(MPU_ADDRESS, MPUREG_EXT_SENS_DATA_07)] = 0x08 if hofl else 0x00

    def test_remaps_the_mag_axes_into_the_body_frame(self, mpu, fake_bus):
        mpu.mcal1 = mpu.mcal2 = mpu.mcal3 = 1.0
        self._load_mag(fake_bus, hx=100, hy=200, hz=300)

        assert mpu._read_mag_sample() == (200.0, 100.0, -300.0)

    def test_applies_factory_sensitivity_per_ak_axis(self, mpu, fake_bus):
        mpu.mcal1, mpu.mcal2, mpu.mcal3 = 2.0, 3.0, 4.0
        self._load_mag(fake_bus, hx=10, hy=20, hz=30)

        m1, m2, m3 = mpu._read_mag_sample()

        assert m1 == pytest.approx(20 * 3.0)   # body X = mag Y * ASAY
        assert m2 == pytest.approx(10 * 2.0)   # body Y = mag X * ASAX
        assert m3 == pytest.approx(-30 * 4.0)  # body Z = -mag Z * ASAZ

    def test_handles_negative_little_endian_words(self, mpu, fake_bus):
        mpu.mcal1 = mpu.mcal2 = mpu.mcal3 = 1.0
        self._load_mag(fake_bus, hz=-16)

        m1, m2, m3 = mpu._read_mag_sample()

        assert m3 == pytest.approx(16.0)

    def test_skips_when_data_is_not_ready(self, mpu, fake_bus):
        mpu.mcal1 = mpu.mcal2 = mpu.mcal3 = 1.0
        self._load_mag(fake_bus, hx=100, drdy=False)

        assert mpu._read_mag_sample() is None

    def test_skips_on_magnetic_overflow(self, mpu, fake_bus):
        mpu.mcal1 = mpu.mcal2 = mpu.mcal3 = 1.0
        self._load_mag(fake_bus, hx=100, hofl=True)

        assert mpu._read_mag_sample() is None


class TestMemWrite:
    def test_splits_the_address_into_bank_and_offset(self, mpu, fake_bus):
        mpu._MPU9250__mem_write(address=CFG_MOTION_BIAS, data=[1, 2, 3])

        assert fake_bus.block_writes[0] == (MPU_ADDRESS, MPUREG_BANK_SEL, [0x04, 0xB8])
        assert fake_bus.block_writes[1] == (MPU_ADDRESS, MPUREG_MEM_R_W, [1, 2, 3])

    def test_rejects_writes_past_the_bank_boundary(self, mpu):
        with pytest.raises(Exception):
            mpu._MPU9250__mem_write(address=0x04F8, data=[0] * 16)


class TestAveraging:
    def test_scales_accumulators_into_physical_units(self, mpu):
        mpu.mcal1 = mpu.mcal2 = mpu.mcal3 = 1.0
        t0 = datetime(2026, 7, 11, 12, 0, 0)
        t = t0 + timedelta(seconds=1, milliseconds=500)

        d = mpu._MPU9250__make_avg_mpu_data(
            avg1=32767.0 * 10, avg2=0.0, avg3=0.0,
            ava1=0.0, ava2=0.0, ava3=16384.0 * 10,
            avm1=100 * 5, avm2=0, avm3=0,
            avtmp=3338.7 * 10,
            n=10, nm=5, t=t, tm=t, t0=t0, t0m=t0,
        )

        assert d.G1 == pytest.approx(250.0)                  # full-scale 250 dps
        assert d.A3 == pytest.approx(16384.0 * 2 / 32767.0)  # ~1 g
        assert d.M1 == pytest.approx(100.0)
        assert d.Temp == pytest.approx(31.0, abs=0.01)       # 3338.7 LSB ~ +10 degC over the 21 degC offset
        assert d.N == 10
        assert d.NM == 5
        assert d.MsgError is None

    def test_interval_length_supports_more_than_one_second(self, mpu):
        mpu.mcal1 = mpu.mcal2 = mpu.mcal3 = 1.0
        t0 = datetime(2026, 7, 11, 12, 0, 0)
        t = t0 + timedelta(seconds=2, milliseconds=250)

        d = mpu._MPU9250__make_avg_mpu_data(
            avg1=0.0, avg2=0.0, avg3=0.0, ava1=0.0, ava2=0.0, ava3=0.0,
            avm1=0, avm2=0, avm3=0, avtmp=0.0,
            n=10, nm=5, t=t, tm=t, t0=t0, t0m=t0,
        )

        assert d.DT == pytest.approx(2250.0)
        assert d.DTM == pytest.approx(2250.0)

    def test_flags_intervals_without_samples(self, mpu):
        mpu.mcal1 = mpu.mcal2 = mpu.mcal3 = 1.0
        now = datetime.now()

        d = mpu._MPU9250__make_avg_mpu_data(
            avg1=0.0, avg2=0.0, avg3=0.0, ava1=0.0, ava2=0.0, ava3=0.0,
            avm1=0, avm2=0, avm3=0, avtmp=0.0,
            n=0, nm=0, t=now, tm=now, t0=now, t0m=now,
        )

        assert d.MsgError is not None


class TestGetAvg:
    def test_blocks_until_the_reader_thread_answers(self, mpu):
        sentinel = MPUData()

        def serve():
            reply = mpu._MPU9250__QUEUE.get(timeout=2)
            reply.put(sentinel)

        threading.Thread(target=serve, daemon=True).start()

        assert mpu.get_avg() is sentinel
