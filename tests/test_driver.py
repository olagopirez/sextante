import threading
from datetime import datetime, timedelta

import pytest

from mpu9250 import AccelRange, GyroRange, MPUData
from mpu9250.constants import (
    AK8963_ASAX,
    AK8963_ASAY,
    AK8963_ASAZ,
    AK8963_CNTL1,
    AK8963_I2C_ADDR,
    AKM_FUSE_ROM_ACCESS,
    AKM_POWER_DOWN,
    BITS_FS_8G,
    BITS_FS_500DPS,
    CFG_MOTION_BIAS,
    MPU_ADDRESS,
    MPUREG_ACCEL_CONFIG,
    MPUREG_BANK_SEL,
    MPUREG_GYRO_CONFIG,
    MPUREG_MEM_R_W,
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


class TestRangeConfiguration:
    def test_accel_range_goes_to_accel_config_register(self, mpu, fake_bus):
        mpu.set_accel_range(AccelRange.RANGE_8_G)

        assert (MPU_ADDRESS, MPUREG_ACCEL_CONFIG, BITS_FS_8G) in fake_bus.byte_writes
        assert [w for w in fake_bus.byte_writes if w[1] == MPUREG_GYRO_CONFIG] == []

    def test_gyro_range_goes_to_gyro_config_register(self, mpu, fake_bus):
        mpu.set_gyro_range(GyroRange.RANGE_500_DPS)

        assert (MPU_ADDRESS, MPUREG_GYRO_CONFIG, BITS_FS_500DPS) in fake_bus.byte_writes


class TestMagCalibration:
    def test_reads_fuse_rom_from_the_ak8963_address(self, mpu, fake_bus):
        fake_bus.byte_regs[(AK8963_I2C_ADDR, AK8963_ASAX)] = 170
        fake_bus.byte_regs[(AK8963_I2C_ADDR, AK8963_ASAY)] = 178
        fake_bus.byte_regs[(AK8963_I2C_ADDR, AK8963_ASAZ)] = 166

        mpu._MPU9250__mag_calibration()

        scale = 9830.0 / 65536.0
        assert mpu.mcal1 == pytest.approx((170 + 128) / 256.0 * scale)
        assert mpu.mcal2 == pytest.approx((178 + 128) / 256.0 * scale)
        assert mpu.mcal3 == pytest.approx((166 + 128) / 256.0 * scale)

    def test_cycles_the_ak8963_through_fuse_rom_mode(self, mpu, fake_bus):
        mpu._MPU9250__mag_calibration()

        cntl1_writes = [v for a, r, v in fake_bus.byte_writes
                        if a == AK8963_I2C_ADDR and r == AK8963_CNTL1]
        assert cntl1_writes == [AKM_POWER_DOWN, AKM_FUSE_ROM_ACCESS, AKM_POWER_DOWN]


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
