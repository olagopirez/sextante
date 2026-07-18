import queue
import threading
import time
from datetime import datetime

import numpy as np

from .constants import *
from .data import MPUData, MPUCalData
from .ranges import AccelRange, GyroRange, LPF
from .ticker import TickerThread


class HardwareMismatchError(RuntimeError):
    """The chip answering on the bus is not the hardware this driver expects."""


def _to_int16(word):
    # Two's complement done in plain Python: np.int16() raises OverflowError on
    # values above 0x7FFF since NumPy 2
    word &= 0xFFFF
    return word - 0x10000 if word >= 0x8000 else word


def _be_word_to_int16(word):
    # Reassembles a big-endian register pair delivered as a little-endian SMBus word
    return _to_int16(((word & 0xFF) << 8) | ((word >> 8) & 0xFF))


# Quarter-turn rotation matrices (rows), exact in integers
_MOUNT_TURNS = {
    ('x', 90): ((1, 0, 0), (0, 0, -1), (0, 1, 0)),
    ('x', 180): ((1, 0, 0), (0, -1, 0), (0, 0, -1)),
    ('x', 270): ((1, 0, 0), (0, 0, 1), (0, -1, 0)),
    ('y', 90): ((0, 0, 1), (0, 1, 0), (-1, 0, 0)),
    ('y', 180): ((-1, 0, 0), (0, 1, 0), (0, 0, -1)),
    ('y', 270): ((0, 0, -1), (0, 1, 0), (1, 0, 0)),
    ('z', 90): ((0, -1, 0), (1, 0, 0), (0, 0, 1)),
    ('z', 180): ((-1, 0, 0), (0, -1, 0), (0, 0, 1)),
    ('z', 270): ((0, 1, 0), (-1, 0, 0), (0, 0, 1)),
}


def _mount_matrix(spec):
    """
    Parses a mounting spec like ``'x180'`` or ``'x180,z90'`` — a sequence of
    quarter turns describing how the chip sits relative to the vehicle — into
    a 3x3 rotation matrix.
    """
    matrix = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    for token in spec.split(','):
        token = token.strip().lower()
        axis, angle = token[:1], token[1:]
        key = (axis, int(angle)) if angle.isdigit() else None
        if key not in _MOUNT_TURNS:
            raise ValueError(
                f"bad mount token '{token}': use axis x/y/z + 90/180/270, e.g. 'x180' or 'x180,z90'")
        turn = _MOUNT_TURNS[key]
        matrix = tuple(
            tuple(sum(turn[i][k] * matrix[k][j] for k in range(3)) for j in range(3))
            for i in range(3))
    return matrix


class MPU9250:
    def __init__(self,
                 address=MPU_ADDRESS,
                 accel_range=AccelRange.RANGE_2_G,
                 gyro_range=GyroRange.RANGE_250_DPS,
                 rate=50,
                 bus=None,
                 mount=None):
        if bus is None:
            import smbus2  # imported lazily so the package works without hardware
            bus = smbus2.SMBus(1)

        self.__bus = bus
        self.__mpu_address = address
        self.__accel_range = accel_range
        self.__gyro_range = gyro_range
        self.__lpf = LPF(rate=rate)
        self.__mount = _mount_matrix(mount) if mount else None
        self.__QUEUE = queue.Queue(maxsize=0)

        self.mcal1 = None
        self.mcal2 = None
        self.mcal3 = None
        self.mpuDate = MPUData()
        self.mpuAvgDate = MPUData()
        self.mpuCalDate = MPUCalData()

    def __write_byte(self, address, byte):
        self.__bus.write_byte_data(i2c_addr=self.__mpu_address, register=address, value=byte)
        time.sleep(1e-3)

    def __read_byte(self, address):
        return self.__bus.read_byte_data(self.__mpu_address, address)

    def __remount(self, v1, v2, v3):
        # Chip body frame → vehicle frame, applied after all calibration
        if self.__mount is None:
            return v1, v2, v3
        m = self.__mount
        return (m[0][0] * v1 + m[0][1] * v2 + m[0][2] * v3,
                m[1][0] * v1 + m[1][1] * v2 + m[1][2] * v3,
                m[2][0] * v1 + m[2][1] * v2 + m[2][2] * v3)

    def __read_word(self, register):
        # SMBus words are little-endian, which matches the AK8963 output registers
        return _to_int16(self.__bus.read_word_data(self.__mpu_address, register))

    def __read_word_be(self, register):
        # MPU9250 gyro/accel/temp output registers are big-endian (high byte first)
        return _be_word_to_int16(self.__bus.read_word_data(self.__mpu_address, register))

    def __write_ak_byte(self, address, byte):
        # Direct AK8963 access; only valid while bypass mode is enabled
        self.__bus.write_byte_data(i2c_addr=AK8963_I2C_ADDR, register=address, value=byte)
        time.sleep(1e-3)

    def __read_ak_byte(self, address):
        return self.__bus.read_byte_data(AK8963_I2C_ADDR, address)

    def self_check(self):
        """
        Verifies that the chip on the bus is a genuine MPU-9250/9255 with a
        responding AK8963 magnetometer, and returns the WHO_AM_I value.

        Many boards sold as MPU-9250 carry relabeled MPU-6500 dies with no
        magnetometer; this catches them before any configuration is written.

        Raises HardwareMismatchError when either die does not identify itself.
        """
        whoami = self.__read_byte(MPUREG_WHOAMI)
        if whoami not in (MPU9250_ID, MPU9255_ID):
            hint = {
                MPU6500_ID: 'an MPU-6500 — no magnetometer; common in relabeled boards',
                MPU6050_ID: 'an MPU-6050',
            }.get(whoami, 'an unknown chip')
            raise HardwareMismatchError(
                f'WHO_AM_I returned 0x{whoami:02X}; expected MPU-9250 (0x{MPU9250_ID:02X}) '
                f'or MPU-9255 (0x{MPU9255_ID:02X}). This looks like {hint}.')

        # The AK8963 die only answers directly while bypass mode is enabled
        temp = self.__read_byte(MPUREG_USER_CTRL)
        self.__write_byte(address=MPUREG_USER_CTRL, byte=temp & ~BIT_AUX_IF_EN)
        time.sleep(3e-3)
        self.__write_byte(address=MPUREG_INT_PIN_CFG, byte=BIT_BYPASS_EN)
        time.sleep(3e-3)

        wia = self.__read_ak_byte(AK8963_WIA)

        self.__write_byte(address=MPUREG_USER_CTRL, byte=temp | BIT_AUX_IF_EN)
        time.sleep(3e-3)
        self.__write_byte(address=MPUREG_INT_PIN_CFG, byte=0x00)
        time.sleep(3e-3)

        if wia != AK8963_Device_ID:
            raise HardwareMismatchError(
                f'AK8963 WIA returned 0x{wia:02X}, expected 0x{AK8963_Device_ID:02X}: '
                f'the magnetometer is not responding (relabeled chip or dead die).')

        return whoami

    def set_gyro_range(self, gyro_range=GyroRange.RANGE_250_DPS):
        self.__gyro_range = gyro_range
        self.__write_byte(address=MPUREG_GYRO_CONFIG, byte=self.__gyro_range.get_bits())

    def set_accel_range(self, accel_range=AccelRange.RANGE_2_G):
        self.__accel_range = accel_range
        self.__write_byte(address=MPUREG_ACCEL_CONFIG, byte=self.__accel_range.get_bits())

    def __mag_setup(self):
        """
        Reads the factory sensitivity values from the AK8963 fuse ROM and
        leaves the magnetometer running in continuous mode (100 Hz, 16-bit)
        for the sampling loop to consume.
        """

        # Enable bypass mode so the AK8963 is directly addressable on the bus
        temp = self.__read_byte(MPUREG_USER_CTRL)
        self.__write_byte(address=MPUREG_USER_CTRL, byte=temp & ~BIT_AUX_IF_EN)
        time.sleep(3e-3)
        self.__write_byte(address=MPUREG_INT_PIN_CFG, byte=BIT_BYPASS_EN)
        time.sleep(3e-3)

        # Power down the AK8963
        self.__write_ak_byte(address=AK8963_CNTL1, byte=AKM_POWER_DOWN)
        time.sleep(1e-3)

        # Fuse AK8963 ROM access
        self.__write_ak_byte(address=AK8963_CNTL1, byte=AKM_FUSE_ROM_ACCESS)
        time.sleep(1e-3)

        # Get sensitivity data from AK8963 fuse ROM
        __mcal1 = self.__read_ak_byte(address=AK8963_ASAX)
        __mcal2 = self.__read_ak_byte(address=AK8963_ASAY)
        __mcal3 = self.__read_ak_byte(address=AK8963_ASAZ)

        scale_mag = np.float64(9830) / np.float64(65536)
        self.mcal1 = np.float64(np.int16(__mcal1) + 128) / 256 * scale_mag
        self.mcal2 = np.float64(np.int16(__mcal2) + 128) / 256 * scale_mag
        self.mcal3 = np.float64(np.int16(__mcal3) + 128) / 256 * scale_mag

        # Power down the AK8963 again to leave fuse ROM access mode
        self.__write_ak_byte(address=AK8963_CNTL1, byte=AKM_POWER_DOWN)
        time.sleep(1e-3)

        # Start continuous measurements; 16-bit matches the 0.15 uT/LSB scale in mcal
        self.__write_ak_byte(address=AK8963_CNTL1, byte=AKM_16BIT | AKM_CONTINUOUS_100HZ)
        time.sleep(1e-3)

        # Disable bypass mode now that we're done getting sensitivity data
        temp = self.__read_byte(MPUREG_USER_CTRL)
        self.__write_byte(address=MPUREG_USER_CTRL, byte=temp | BIT_AUX_IF_EN)
        time.sleep(3e-3)
        self.__write_byte(address=MPUREG_INT_PIN_CFG, byte=0x00)
        time.sleep(3e-3)

    def _read_mag_sample(self):
        """
        Reads one magnetometer sample from the EXT_SENS_DATA registers, where
        the aux I2C master copies ST1..ST2 on every internal sample.

        Returns (m1, m2, m3) with the factory sensitivity applied and remapped
        into the accel/gyro frame, or None when the AK8963 has no fresh data
        (ST1 DRDY clear) or the sensor overflowed (ST2 HOFL set).
        """
        st1 = self.__read_byte(MPUREG_EXT_SENS_DATA_00)
        if (st1 & AKM_DATA_READY) == 0x00:
            return None

        hx = self.__read_word(MPUREG_EXT_SENS_DATA_01)
        hy = self.__read_word(MPUREG_EXT_SENS_DATA_03)
        hz = self.__read_word(MPUREG_EXT_SENS_DATA_05)
        st2 = self.__read_byte(MPUREG_EXT_SENS_DATA_07)

        if (st2 & AKM_HOFL) != 0x00:
            return None

        # The AK8963 axes are rotated relative to the accel/gyro frame:
        # body X = mag Y, body Y = mag X, body Z = -mag Z
        return (np.float64(hy) * self.mcal2,
                np.float64(hx) * self.mcal1,
                -np.float64(hz) * self.mcal3)

    def __enable_gyro_bias_cal(self, enable=False):
        """
        Enables or disables motion bias compensation for the gyro.
        -> For flying we generally do not want this!
        """
        enable_regs = [0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35, 0x5d]
        disable_regs = [0xb8, 0xaa, 0xaa, 0xaa, 0xb0, 0x88, 0xc3, 0xc5, 0xc7]

        self.__mem_write(address=CFG_MOTION_BIAS, data=enable_regs if enable else disable_regs)

    def __mem_write(self, address=CFG_MOTION_BIAS, data=None):
        if data is None:
            data = []

        temp = [(address >> 8) & 0xFF, address & 0xFF]

        # Check memory bank boundaries
        if temp[1] + len(data) > MPU_BANK_SIZE:
            raise Exception('Bad address: writing outside of memory bank boundaries')

        self.__bus.write_i2c_block_data(self.__mpu_address, MPUREG_BANK_SEL, temp)
        self.__bus.write_i2c_block_data(self.__mpu_address, MPUREG_MEM_R_W, data)

    # Initialization of MPU
    def initialize(self, check_hardware=True):
        # Reset device.
        self.__write_byte(address=MPUREG_PWR_MGMT_1, byte=BIT_H_RESET)

        # Wake up chip.
        time.sleep(1e-1)
        self.__write_byte(address=MPUREG_PWR_MGMT_1, byte=0x00)

        # Refuse to configure hardware that isn't what this driver expects
        if check_hardware:
            self.self_check()

        # Don't let FIFO overwrite DMP data
        self.__write_byte(address=MPUREG_ACCEL_CONFIG_2, byte=BIT_FIFO_SIZE_1024 | 0x8)

        # Set accelerometer and gyroscope range
        self.set_accel_range(accel_range=self.__accel_range)
        self.set_gyro_range(gyro_range=self.__gyro_range)

        # Default: Set Gyro LPF to half of sample rate
        self.__write_byte(address=MPUREG_CONFIG, byte=self.__lpf.get_gyro_bits())

        # Default: Set Accel LPF to half of sample rate
        self.__write_byte(address=MPUREG_ACCEL_CONFIG_2, byte=self.__lpf.get_accel_bits())

        # Changes the sampling rate of the MPU.
        self.__write_byte(address=MPUREG_SMPLRT_DIV, byte=self.__lpf.get_simple_rate_byte())

        # Turn off FIFO buffer
        self.__write_byte(address=MPUREG_FIFO_EN, byte=0x00)

        # Turn off interrupts
        self.__write_byte(address=MPUREG_INT_ENABLE, byte=0x00)

        # --- Magnetometer: factory calibration + continuous mode --- #
        self.__mag_setup()

        # Set up AK8963 master mode, master clock and ES bit
        self.__write_byte(address=MPUREG_I2C_MST_CTRL, byte=0x40)

        # Slave 0 reads from AK8963
        self.__write_byte(address=MPUREG_I2C_SLV0_ADDR, byte=BIT_I2C_READ | AK8963_I2C_ADDR)

        # Compass reads start at this register
        self.__write_byte(address=MPUREG_I2C_SLV0_REG, byte=AK8963_ST1)

        # Enable 8-byte reads on slave 0
        self.__write_byte(address=MPUREG_I2C_SLV0_CTRL, byte=BIT_SLAVE_EN | 8)

        # Triggers slave 0 reads at each sample
        self.__write_byte(address=MPUREG_I2C_MST_DELAY_CTRL, byte=0x01)

        # Not so sure of this one--I2C Slave 4??!
        if self.__lpf.get_simple_rate() < AK8963_MAX_SAMPLE_RATE:
            self.__write_byte(address=MPUREG_I2C_SLV4_CTRL, byte=0x00)
        else:
            self.__write_byte(address=MPUREG_I2C_SLV4_CTRL,
                              byte=np.byte(self.__lpf.get_simple_rate() // AK8963_MAX_SAMPLE_RATE - 1))

        time.sleep(1e-1)

        # Set clock source to PLL
        self.__write_byte(address=MPUREG_PWR_MGMT_1, byte=INV_CLK_PLL)

        # Turn off all sensors -- Not sure if necessary, but it's in the InvenSense DMP driver
        self.__write_byte(address=MPUREG_PWR_MGMT_2, byte=0x63)
        time.sleep(1e-1)

        # Turn on all gyro, all accel
        self.__write_byte(address=MPUREG_PWR_MGMT_2, byte=0x00)

        # Usually we don't want the automatic gyro bias compensation - it pollutes the gyro in a non-inertial frame.
        self.__enable_gyro_bias_cal(enable=False)

        # Give the IMU time to fully initialize and then clear out any bad values from the averages.
        time.sleep(5e-1)  # Make sure it's ready

        h = threading.Thread(target=self.__read_data)
        h.daemon = True
        h.start()

    def __read_data(self):
        m1 = m2 = m3 = np.float64(0)
        avg1 = avg2 = avg3 = ava1 = ava2 = ava3 = avtmp = np.float64(0)
        avm1 = avm2 = avm3 = np.float64(0)
        n = nm = 0
        t = tm = t0 = t0m = datetime.now()

        float_rate = np.float32(self.__lpf.get_rate())
        period = np.float32(int(1000.0 / float_rate + 0.5)) / 1000.0

        float_rate_mag = np.float32(100 if self.__lpf.get_rate() > 100 else self.__lpf.get_rate())
        period_mag = np.float32(int(1000.0 / float_rate_mag + 0.5)) / 1000.0

        clock = TickerThread(period=period, q=queue.Queue(maxsize=0))
        clock_mag = TickerThread(period=period_mag, q=queue.Queue(maxsize=0))

        clock.start()
        clock_mag.start()

        combined = queue.Queue(maxsize=0)

        def listen_and_forward(q):
            while True:
                combined.put((q, q.get()))

        laf = threading.Thread(target=listen_and_forward, args=(clock.get_q(),))
        laf.daemon = True
        laf.start()
        laf = threading.Thread(target=listen_and_forward, args=(clock_mag.get_q(),))
        laf.daemon = True
        laf.start()

        laf = threading.Thread(target=listen_and_forward, args=(self.__QUEUE,))
        laf.daemon = True
        laf.start()

        t0 = datetime.now()
        t0m = datetime.now()

        while True:
            which, message = combined.get()
            if which is clock.get_q():
                t = datetime.now()
                g1 = self.__read_word_be(MPUREG_GYRO_XOUT_H)
                g2 = self.__read_word_be(MPUREG_GYRO_YOUT_H)
                g3 = self.__read_word_be(MPUREG_GYRO_ZOUT_H)
                a1 = self.__read_word_be(MPUREG_ACCEL_XOUT_H)
                a2 = self.__read_word_be(MPUREG_ACCEL_YOUT_H)
                a3 = self.__read_word_be(MPUREG_ACCEL_ZOUT_H)
                tmp = self.__read_word_be(MPUREG_TEMP_OUT_H)

                mm1 = m1 - self.mpuCalDate.M01
                mm2 = m2 - self.mpuCalDate.M02
                mm3 = m3 - self.mpuCalDate.M03

                gs = self.__gyro_range.get_scale()
                acs = self.__accel_range.get_scale()
                gv = self.__remount((np.float64(g1) - self.mpuCalDate.G01) * gs,
                                    (np.float64(g2) - self.mpuCalDate.G02) * gs,
                                    (np.float64(g3) - self.mpuCalDate.G03) * gs)
                av = self.__remount((np.float64(a1) - self.mpuCalDate.A01) * acs,
                                    (np.float64(a2) - self.mpuCalDate.A02) * acs,
                                    (np.float64(a3) - self.mpuCalDate.A03) * acs)
                mv = self.__remount(
                    self.mpuCalDate.Ms11 * mm1 + self.mpuCalDate.Ms12 * mm2 + self.mpuCalDate.Ms13 * mm3,
                    self.mpuCalDate.Ms21 * mm1 + self.mpuCalDate.Ms22 * mm2 + self.mpuCalDate.Ms23 * mm3,
                    self.mpuCalDate.Ms31 * mm1 + self.mpuCalDate.Ms32 * mm2 + self.mpuCalDate.Ms33 * mm3)

                self.mpuDate = MPUData(
                    g1=gv[0], g2=gv[1], g3=gv[2],
                    a1=av[0], a2=av[1], a3=av[2],
                    m1=mv[0], m2=mv[1], m3=mv[2],
                    temp=np.float64(tmp) * MPU9250T_85degC + 21, t=t, tm=tm, n=n, nm=nm
                )

                avg1 += np.float64(g1)
                avg2 += np.float64(g2)
                avg3 += np.float64(g3)
                ava1 += np.float64(a1)
                ava2 += np.float64(a2)
                ava3 += np.float64(a3)
                avtmp += np.float64(tmp)
                n += 1
            elif which is clock_mag.get_q():
                sample = self._read_mag_sample()
                if sample is None:
                    continue  # data not ready, or magnetic overflow

                tm = datetime.now()
                m1, m2, m3 = sample

                avm1 += m1
                avm2 += m2
                avm3 += m3
                nm += 1
            elif which is self.__QUEUE:
                self.mpuAvgDate = self.__make_avg_mpu_data(
                    avg1=avg1, avg2=avg2, avg3=avg3,
                    ava1=ava1, ava2=ava2, ava3=ava3,
                    avm1=avm1, avm2=avm2, avm3=avm3,
                    avtmp=avtmp, n=n, nm=nm, t=t, tm=tm, t0=t0, t0m=t0m
                )
                message.put(self.mpuAvgDate)

                m1 = m2 = m3 = np.float64(0)
                avg1 = avg2 = avg3 = ava1 = ava2 = ava3 = avtmp = np.float64(0)
                avm1 = avm2 = avm3 = np.float64(0)
                n = nm = 0
                t0 = t
                t0m = tm

    def __make_avg_mpu_data(self, avg1, avg2, avg3, ava1, ava2, ava3, avm1, avm2, avm3, avtmp, n, nm, t, tm, t0, t0m):
        d = MPUData()

        if n > 0.5:
            gs = self.__gyro_range.get_scale()
            acs = self.__accel_range.get_scale()
            d.G1, d.G2, d.G3 = self.__remount((avg1 / n - self.mpuCalDate.G01) * gs,
                                              (avg2 / n - self.mpuCalDate.G02) * gs,
                                              (avg3 / n - self.mpuCalDate.G03) * gs)
            d.A1, d.A2, d.A3 = self.__remount((ava1 / n - self.mpuCalDate.A01) * acs,
                                              (ava2 / n - self.mpuCalDate.A02) * acs,
                                              (ava3 / n - self.mpuCalDate.A03) * acs)
            d.Temp = (np.float64(avtmp) / np.float64(n)) * MPU9250T_85degC + 21
            d.N = int(n + 0.5)
            d.T = t
            timedelta = t - t0
            d.DT = timedelta.total_seconds() * 1000  # ms
        else:
            d.MsgError = 'MPU9250 Error: No new accel/gyro values'

        if nm > 0:
            mm1 = avm1 / nm - self.mpuCalDate.M01
            mm2 = avm2 / nm - self.mpuCalDate.M02
            mm3 = avm3 / nm - self.mpuCalDate.M03

            d.M1, d.M2, d.M3 = self.__remount(
                self.mpuCalDate.Ms11 * mm1 + self.mpuCalDate.Ms12 * mm2 + self.mpuCalDate.Ms13 * mm3,
                self.mpuCalDate.Ms21 * mm1 + self.mpuCalDate.Ms22 * mm2 + self.mpuCalDate.Ms23 * mm3,
                self.mpuCalDate.Ms31 * mm1 + self.mpuCalDate.Ms32 * mm2 + self.mpuCalDate.Ms33 * mm3)
            d.NM = int(nm + 0.5)
            d.TM = tm
            timedeltam = tm - t0m
            d.DTM = timedeltam.total_seconds() * 1000  # ms
        else:
            d.MsgError = 'MPU9250 Error: No new magnetometer values'

        return d

    def get_avg(self):
        reply = queue.Queue(maxsize=1)
        self.__QUEUE.put(reply)
        return reply.get()

    def calibrate_gyro(self, duration=2.0):
        """
        Measures the gyro bias over ``duration`` seconds — the device must sit
        still — and folds it into ``mpuCalDate`` so subsequent readings center
        on zero. Call after ``initialize()``. Returns the measured bias in °/s.

        MEMS gyros always show a constant offset per axis (typically around
        ±1 °/s, drifting with temperature); without this the attitude filter
        fights a phantom rotation forever.
        """
        deadline = time.monotonic() + duration
        sums = [0.0, 0.0, 0.0]
        n = 0
        while time.monotonic() < deadline:
            d = self.mpuDate
            sums[0] += float(d.G1)
            sums[1] += float(d.G2)
            sums[2] += float(d.G3)
            n += 1
            time.sleep(1.0 / self.__lpf.get_rate())
        if n == 0:
            return (0.0, 0.0, 0.0)

        bias = (sums[0] / n, sums[1] / n, sums[2] / n)

        # Readings are vehicle-frame; the stored bias lives in the chip frame,
        # so un-rotate with the mount transpose (= inverse for a rotation)
        chip = bias
        if self.__mount is not None:
            m = self.__mount
            chip = (m[0][0] * bias[0] + m[1][0] * bias[1] + m[2][0] * bias[2],
                    m[0][1] * bias[0] + m[1][1] * bias[1] + m[2][1] * bias[2],
                    m[0][2] * bias[0] + m[1][2] * bias[1] + m[2][2] * bias[2])

        scale = self.__gyro_range.get_scale()
        self.mpuCalDate.G01 += chip[0] / scale
        self.mpuCalDate.G02 += chip[1] / scale
        self.mpuCalDate.G03 += chip[2] / scale
        return bias
