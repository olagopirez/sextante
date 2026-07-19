from datetime import datetime, timedelta

import pytest

from mpu9250 import LSM9DS1, HardwareMismatchError, MPUData
from mpu9250.detect import detect_imu
from mpu9250.lsm9ds1 import (
    ACCEL_G_PER_LSB,
    GYRO_DPS_PER_LSB,
    LSM9DS1_AG_ADDRESS,
    LSM9DS1_AG_ID,
    LSM9DS1_MAG_ADDRESS,
    LSM9DS1_MAG_ID,
    MAG_AUTO_INCREMENT,
    MAG_UT_PER_LSB,
    REG_CTRL_REG1_G,
    REG_CTRL_REG1_M,
    REG_CTRL_REG8,
    REG_OUT_TEMP_L,
    REG_OUT_X_G,
    REG_OUT_X_L_M,
    REG_OUT_X_XL,
    REG_WHO_AM_I,
)


def identify(fake_bus, ag=LSM9DS1_AG_ID, mag=LSM9DS1_MAG_ID):
    fake_bus.byte_regs[(LSM9DS1_AG_ADDRESS, REG_WHO_AM_I)] = ag
    fake_bus.byte_regs[(LSM9DS1_MAG_ADDRESS, REG_WHO_AM_I)] = mag


def load_words(fake_bus, address, register, values):
    for i, value in enumerate(values):
        raw = value & 0xFFFF
        fake_bus.byte_regs[(address, register + 2 * i)] = raw & 0xFF
        fake_bus.byte_regs[(address, register + 2 * i + 1)] = raw >> 8


@pytest.fixture
def lsm(fake_bus):
    return LSM9DS1(bus=fake_bus)


class TestSelfCheck:
    def test_passes_when_both_dies_answer(self, lsm, fake_bus):
        identify(fake_bus)

        assert lsm.self_check() == LSM9DS1_AG_ID

    def test_rejects_a_wrong_accel_gyro_id(self, lsm, fake_bus):
        identify(fake_bus, ag=0x71)  # an MPU id where the LSM should be

        with pytest.raises(HardwareMismatchError, match='accel/gyro'):
            lsm.self_check()

    def test_rejects_a_silent_magnetometer(self, lsm, fake_bus):
        identify(fake_bus, mag=0x00)

        with pytest.raises(HardwareMismatchError, match='magnetometer'):
            lsm.self_check()


class TestConfiguration:
    def test_configures_both_dies(self, lsm, fake_bus):
        lsm._LSM9DS1__configure()

        ag_writes = {(r, v) for a, r, v in fake_bus.byte_writes if a == LSM9DS1_AG_ADDRESS}
        mag_writes = {(r, v) for a, r, v in fake_bus.byte_writes if a == LSM9DS1_MAG_ADDRESS}
        assert (REG_CTRL_REG8, 0x44) in ag_writes       # BDU + auto-increment
        assert (REG_CTRL_REG1_G, 0x60) in ag_writes     # 119 Hz, 245 dps
        assert (REG_CTRL_REG1_M, 0x7C) in mag_writes    # 80 Hz, ultra-high perf
        assert (0x22, 0x00) in mag_writes               # continuous mode


class TestRawReads:
    def test_imu_words_are_little_endian_and_signed(self, lsm, fake_bus):
        load_words(fake_bus, LSM9DS1_AG_ADDRESS, REG_OUT_X_G, [1000, -1000, 0])
        load_words(fake_bus, LSM9DS1_AG_ADDRESS, REG_OUT_X_XL, [16393, 0, -16393])
        load_words(fake_bus, LSM9DS1_AG_ADDRESS, REG_OUT_TEMP_L, [160])

        g1, g2, g3, a1, a2, a3, temp = lsm._read_imu_raw()

        assert (g1, g2, g3) == (1000, -1000, 0)
        assert (a1, a2, a3) == (16393, 0, -16393)
        assert temp == 160
        # scales: 1000 counts ≈ 8.75 °/s; 16393 counts ≈ 1 g; 160 -> +10 °C over 25
        assert 1000 * GYRO_DPS_PER_LSB == pytest.approx(8.75)
        assert 16393 * ACCEL_G_PER_LSB == pytest.approx(1.0, abs=0.001)

    def test_mag_is_scaled_and_x_is_unmirrored(self, lsm, fake_bus):
        load_words(fake_bus, LSM9DS1_MAG_ADDRESS, MAG_AUTO_INCREMENT | REG_OUT_X_L_M,
                   [1000, 2000, -3000])

        m1, m2, m3 = lsm._read_mag_sample()

        assert m1 == pytest.approx(-1000 * MAG_UT_PER_LSB)  # X mirrored on the die
        assert m2 == pytest.approx(2000 * MAG_UT_PER_LSB)
        assert m3 == pytest.approx(-3000 * MAG_UT_PER_LSB)


class TestAveraging:
    def test_scales_accumulators_into_physical_units(self, lsm):
        t0 = datetime(2026, 7, 19, 12, 0, 0)
        t = t0 + timedelta(seconds=1)

        d = lsm._LSM9DS1__make_avg_mpu_data(
            avg1=1000.0 * 10, avg2=0.0, avg3=0.0,
            ava1=0.0, ava2=0.0, ava3=16393.0 * 10,
            avm1=14.0 * 5, avm2=0.0, avm3=-28.0 * 5,
            avtmp=160.0 * 10,
            n=10, nm=5, t=t, tm=t, t0=t0, t0m=t0,
        )

        assert d.G1 == pytest.approx(8.75)
        assert d.A3 == pytest.approx(1.0, abs=0.001)
        assert d.M1 == pytest.approx(14.0)
        assert d.M3 == pytest.approx(-28.0)
        assert d.Temp == pytest.approx(35.0)
        assert d.MsgError is None

    def test_mount_rotates_into_the_vehicle_frame(self, fake_bus):
        lsm = LSM9DS1(bus=fake_bus, mount='x180')
        t0 = datetime(2026, 7, 19, 12, 0, 0)
        t = t0 + timedelta(seconds=1)

        d = lsm._LSM9DS1__make_avg_mpu_data(
            avg1=0.0, avg2=0.0, avg3=0.0,
            ava1=0.0, ava2=0.0, ava3=-16393.0 * 10,  # chip reads gravity on -Z
            avm1=0.0, avm2=0.0, avm3=0.0,
            avtmp=0.0, n=10, nm=5, t=t, tm=t, t0=t0, t0m=t0,
        )

        assert d.A3 == pytest.approx(1.0, abs=0.001)  # level in the vehicle frame

    def test_calibrate_gyro_folds_bias_in_counts(self, lsm):
        lsm.mpuDate = MPUData(g1=8.75, g2=0.0, g3=-8.75)

        bias = lsm.calibrate_gyro(duration=0.05)

        assert bias == pytest.approx((8.75, 0.0, -8.75))
        assert float(lsm.mpuCalDate.G01) == pytest.approx(1000.0)
        assert float(lsm.mpuCalDate.G03) == pytest.approx(-1000.0)


class TestDetectIMU:
    def test_finds_an_mpu9250(self, fake_bus):
        fake_bus.byte_regs[(0x68, 0x75)] = 0x71

        assert detect_imu(fake_bus) == 'mpu9250'

    def test_finds_an_lsm9ds1(self, fake_bus):
        fake_bus.byte_regs[(LSM9DS1_AG_ADDRESS, REG_WHO_AM_I)] = LSM9DS1_AG_ID

        assert detect_imu(fake_bus) == 'lsm9ds1'

    def test_prefers_reporting_what_it_saw(self, fake_bus):
        fake_bus.byte_regs[(0x68, 0x75)] = 0x70  # relabeled MPU-6500

        with pytest.raises(HardwareMismatchError, match='0x70'):
            detect_imu(fake_bus)
