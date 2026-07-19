"""Driver for the ST LSM9DS1 iNEMO 9-axis IMU (Ozzmaker BerryGPS-IMU v2 /
BerryIMUv2 boards).

Presents the same reading surface as ``MPU9250`` — ``mpuDate``, ``get_avg()``,
``self_check()``, ``calibrate_gyro()``, ``mount=`` — so the whole pipeline
(recorder, streamer, viewer, reports) runs unchanged on either chip.

The LSM9DS1 is two I2C devices in one package: accel/gyro (0x6A/0x6B) and
magnetometer (0x1C/0x1E). Its registers are little-endian throughout, and the
magnetometer X axis is mirrored relative to the accel/gyro frame — this driver
un-mirrors it, so the mag is reported in the accel/gyro frame like the MPU.
"""

import queue
import threading
import time
from datetime import datetime

import numpy as np

from .data import MPUCalData, MPUData
from .driver import HardwareMismatchError, _mount_matrix, _to_int16
from .ticker import TickerThread

# ---------- Addresses and identification ----------
LSM9DS1_AG_ADDRESS = 0x6A   # accel/gyro; 0x6B with SDO_A/G pulled high
LSM9DS1_MAG_ADDRESS = 0x1C  # magnetometer; 0x1E with SDO_M pulled high
LSM9DS1_AG_ID = 0x68        # WHO_AM_I — same value the MPU-6050 uses, other address
LSM9DS1_MAG_ID = 0x3D       # WHO_AM_I_M

# ---------- Accel/gyro registers ----------
REG_WHO_AM_I = 0x0F
REG_CTRL_REG1_G = 0x10
REG_OUT_TEMP_L = 0x15
REG_OUT_X_G = 0x18
REG_CTRL_REG4 = 0x1E
REG_CTRL_REG5_XL = 0x1F
REG_CTRL_REG6_XL = 0x20
REG_CTRL_REG8 = 0x22
REG_OUT_X_XL = 0x28

# ---------- Magnetometer registers ----------
REG_CTRL_REG1_M = 0x20
REG_CTRL_REG2_M = 0x21
REG_CTRL_REG3_M = 0x22
REG_CTRL_REG4_M = 0x23
REG_OUT_X_L_M = 0x28
MAG_AUTO_INCREMENT = 0x80  # MSB of the sub-address enables multi-byte reads

# ---------- Fixed configuration and scales ----------
CTRL_REG8_BDU_INC = 0x44     # block data update + register auto-increment
CTRL_REG1_G_119HZ_245DPS = 0x60
CTRL_REG6_XL_119HZ_2G = 0x60
CTRL_REG1_M_80HZ_UHP = 0x7C  # ultra-high performance XY, 80 Hz
CTRL_REG2_M_4GAUSS = 0x00
CTRL_REG3_M_CONTINUOUS = 0x00
CTRL_REG4_M_UHP_Z = 0x0C     # ultra-high performance Z

GYRO_DPS_PER_LSB = 8.75e-3   # 245 dps full scale
ACCEL_G_PER_LSB = 0.061e-3   # ±2 g full scale
MAG_UT_PER_LSB = 0.014       # ±4 gauss full scale (0.14 mgauss/LSB)
TEMP_LSB_PER_DEGC = 16.0
TEMP_OFFSET_DEGC = 25.0
MAG_MAX_SAMPLE_RATE = 80


class LSM9DS1:
    def __init__(self,
                 ag_address=LSM9DS1_AG_ADDRESS,
                 mag_address=LSM9DS1_MAG_ADDRESS,
                 rate=50,
                 bus=None,
                 mount=None):
        if bus is None:
            import smbus2  # imported lazily so the package works without hardware
            bus = smbus2.SMBus(1)

        self.__bus = bus
        self.__ag = ag_address
        self.__mag = mag_address
        self.__rate = rate
        self.__mount = _mount_matrix(mount) if mount else None
        self.__QUEUE = queue.Queue(maxsize=0)

        # Parity with MPU9250: the LSM9DS1 has no fuse-ROM sensitivity, so the
        # per-axis factory factors are identity
        self.mcal1 = self.mcal2 = self.mcal3 = 1.0
        self.mpuDate = MPUData()
        self.mpuAvgDate = MPUData()
        self.mpuCalDate = MPUCalData()

    def __remount(self, v1, v2, v3):
        # Chip body frame → vehicle frame, applied after all calibration
        if self.__mount is None:
            return v1, v2, v3
        m = self.__mount
        return (m[0][0] * v1 + m[0][1] * v2 + m[0][2] * v3,
                m[1][0] * v1 + m[1][1] * v2 + m[1][2] * v3,
                m[2][0] * v1 + m[2][1] * v2 + m[2][2] * v3)

    def self_check(self):
        """
        Verifies both LSM9DS1 dies answer with their WHO_AM_I ids and returns
        the accel/gyro id. Raises HardwareMismatchError otherwise.
        """
        try:
            ag_id = self.__bus.read_byte_data(self.__ag, REG_WHO_AM_I)
        except OSError:
            ag_id = None
        if ag_id != LSM9DS1_AG_ID:
            got = 'no answer' if ag_id is None else f'0x{ag_id:02X}'
            raise HardwareMismatchError(
                f'LSM9DS1 accel/gyro not found at 0x{self.__ag:02X} '
                f'(WHO_AM_I {got}, expected 0x{LSM9DS1_AG_ID:02X})')

        try:
            mag_id = self.__bus.read_byte_data(self.__mag, REG_WHO_AM_I)
        except OSError:
            mag_id = None
        if mag_id != LSM9DS1_MAG_ID:
            got = 'no answer' if mag_id is None else f'0x{mag_id:02X}'
            raise HardwareMismatchError(
                f'LSM9DS1 magnetometer not found at 0x{self.__mag:02X} '
                f'(WHO_AM_I_M {got}, expected 0x{LSM9DS1_MAG_ID:02X})')

        return ag_id

    def __configure(self):
        ag, mag, w = self.__ag, self.__mag, self.__bus.write_byte_data
        w(ag, REG_CTRL_REG8, CTRL_REG8_BDU_INC)
        w(ag, REG_CTRL_REG1_G, CTRL_REG1_G_119HZ_245DPS)
        w(ag, REG_CTRL_REG4, 0x38)      # gyro XYZ enabled
        w(ag, REG_CTRL_REG5_XL, 0x38)   # accel XYZ enabled
        w(ag, REG_CTRL_REG6_XL, CTRL_REG6_XL_119HZ_2G)

        w(mag, REG_CTRL_REG1_M, CTRL_REG1_M_80HZ_UHP)
        w(mag, REG_CTRL_REG2_M, CTRL_REG2_M_4GAUSS)
        w(mag, REG_CTRL_REG4_M, CTRL_REG4_M_UHP_Z)
        w(mag, REG_CTRL_REG3_M, CTRL_REG3_M_CONTINUOUS)

    def initialize(self, check_hardware=True):
        if check_hardware:
            self.self_check()

        self.__configure()
        time.sleep(0.05)  # first conversions

        h = threading.Thread(target=self.__read_data)
        h.daemon = True
        h.start()

    # ---------- raw readers (chip frame, unscaled counts) ----------

    def _read_imu_raw(self):
        """Returns (g1, g2, g3, a1, a2, a3, temp_raw) as int16 counts."""
        g = self.__bus.read_i2c_block_data(self.__ag, REG_OUT_X_G, 6)
        a = self.__bus.read_i2c_block_data(self.__ag, REG_OUT_X_XL, 6)
        t = self.__bus.read_i2c_block_data(self.__ag, REG_OUT_TEMP_L, 2)
        word = lambda d, i: _to_int16(d[i] | (d[i + 1] << 8))
        return (word(g, 0), word(g, 2), word(g, 4),
                word(a, 0), word(a, 2), word(a, 4),
                word(t, 0))

    def _read_mag_sample(self):
        """
        Returns (m1, m2, m3) in µT, already in the accel/gyro frame: the
        LSM9DS1 magnetometer X axis is mirrored, so it is negated here.
        """
        d = self.__bus.read_i2c_block_data(self.__mag, MAG_AUTO_INCREMENT | REG_OUT_X_L_M, 6)
        word = lambda i: _to_int16(d[i] | (d[i + 1] << 8))
        return (-word(0) * MAG_UT_PER_LSB * self.mcal1,
                word(2) * MAG_UT_PER_LSB * self.mcal2,
                word(4) * MAG_UT_PER_LSB * self.mcal3)

    # ---------- sampling loop (same shape as the MPU9250 driver) ----------

    def __read_data(self):
        m1 = m2 = m3 = np.float64(0)
        avg1 = avg2 = avg3 = ava1 = ava2 = ava3 = avtmp = np.float64(0)
        avm1 = avm2 = avm3 = np.float64(0)
        n = nm = 0
        t = tm = t0 = t0m = datetime.now()

        period = 1.0 / self.__rate
        period_mag = 1.0 / min(self.__rate, MAG_MAX_SAMPLE_RATE)

        clock = TickerThread(period=period, q=queue.Queue(maxsize=0))
        clock_mag = TickerThread(period=period_mag, q=queue.Queue(maxsize=0))
        clock.start()
        clock_mag.start()

        combined = queue.Queue(maxsize=0)

        def listen_and_forward(q):
            while True:
                combined.put((q, q.get()))

        for q in (clock.get_q(), clock_mag.get_q(), self.__QUEUE):
            laf = threading.Thread(target=listen_and_forward, args=(q,))
            laf.daemon = True
            laf.start()

        t0 = datetime.now()
        t0m = datetime.now()

        while True:
            which, message = combined.get()
            if which is clock.get_q():
                t = datetime.now()
                g1, g2, g3, a1, a2, a3, tmp = self._read_imu_raw()

                mm1 = m1 - self.mpuCalDate.M01
                mm2 = m2 - self.mpuCalDate.M02
                mm3 = m3 - self.mpuCalDate.M03

                gv = self.__remount((np.float64(g1) - self.mpuCalDate.G01) * GYRO_DPS_PER_LSB,
                                    (np.float64(g2) - self.mpuCalDate.G02) * GYRO_DPS_PER_LSB,
                                    (np.float64(g3) - self.mpuCalDate.G03) * GYRO_DPS_PER_LSB)
                av = self.__remount((np.float64(a1) - self.mpuCalDate.A01) * ACCEL_G_PER_LSB,
                                    (np.float64(a2) - self.mpuCalDate.A02) * ACCEL_G_PER_LSB,
                                    (np.float64(a3) - self.mpuCalDate.A03) * ACCEL_G_PER_LSB)
                mv = self.__remount(
                    self.mpuCalDate.Ms11 * mm1 + self.mpuCalDate.Ms12 * mm2 + self.mpuCalDate.Ms13 * mm3,
                    self.mpuCalDate.Ms21 * mm1 + self.mpuCalDate.Ms22 * mm2 + self.mpuCalDate.Ms23 * mm3,
                    self.mpuCalDate.Ms31 * mm1 + self.mpuCalDate.Ms32 * mm2 + self.mpuCalDate.Ms33 * mm3)

                self.mpuDate = MPUData(
                    g1=gv[0], g2=gv[1], g3=gv[2],
                    a1=av[0], a2=av[1], a3=av[2],
                    m1=mv[0], m2=mv[1], m3=mv[2],
                    temp=np.float64(tmp) / TEMP_LSB_PER_DEGC + TEMP_OFFSET_DEGC,
                    t=t, tm=tm, n=n, nm=nm,
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
                try:
                    sample = self._read_mag_sample()
                except OSError:
                    continue
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
                    avtmp=avtmp, n=n, nm=nm, t=t, tm=tm, t0=t0, t0m=t0m,
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
            d.G1, d.G2, d.G3 = self.__remount((avg1 / n - self.mpuCalDate.G01) * GYRO_DPS_PER_LSB,
                                              (avg2 / n - self.mpuCalDate.G02) * GYRO_DPS_PER_LSB,
                                              (avg3 / n - self.mpuCalDate.G03) * GYRO_DPS_PER_LSB)
            d.A1, d.A2, d.A3 = self.__remount((ava1 / n - self.mpuCalDate.A01) * ACCEL_G_PER_LSB,
                                              (ava2 / n - self.mpuCalDate.A02) * ACCEL_G_PER_LSB,
                                              (ava3 / n - self.mpuCalDate.A03) * ACCEL_G_PER_LSB)
            d.Temp = (np.float64(avtmp) / np.float64(n)) / TEMP_LSB_PER_DEGC + TEMP_OFFSET_DEGC
            d.N = int(n + 0.5)
            d.T = t
            timedelta = t - t0
            d.DT = timedelta.total_seconds() * 1000  # ms
        else:
            d.MsgError = 'LSM9DS1 Error: No new accel/gyro values'

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
            d.MsgError = 'LSM9DS1 Error: No new magnetometer values'

        return d

    def get_avg(self):
        reply = queue.Queue(maxsize=1)
        self.__QUEUE.put(reply)
        return reply.get()

    def calibrate_gyro(self, duration=2.0):
        """
        Measures the gyro bias over ``duration`` seconds — the device must sit
        still — and folds it into ``mpuCalDate``. Returns the bias in °/s.
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
            time.sleep(1.0 / self.__rate)
        if n == 0:
            return (0.0, 0.0, 0.0)

        bias = (sums[0] / n, sums[1] / n, sums[2] / n)

        chip = bias
        if self.__mount is not None:
            m = self.__mount
            chip = (m[0][0] * bias[0] + m[1][0] * bias[1] + m[2][0] * bias[2],
                    m[0][1] * bias[0] + m[1][1] * bias[1] + m[2][1] * bias[2],
                    m[0][2] * bias[0] + m[1][2] * bias[1] + m[2][2] * bias[2])

        self.mpuCalDate.G01 += chip[0] / GYRO_DPS_PER_LSB
        self.mpuCalDate.G02 += chip[1] / GYRO_DPS_PER_LSB
        self.mpuCalDate.G03 += chip[2] / GYRO_DPS_PER_LSB
        return bias
