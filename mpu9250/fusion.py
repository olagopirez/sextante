"""Attitude estimation: Mahony AHRS filter and quaternion helpers.

The filter fuses gyro (rad/s), accelerometer (any consistent unit) and
optionally magnetometer readings, all in the driver's body frame (A3 = +1 g
at rest), into an orientation quaternion (w, x, y, z) rotating body to world.
"""

import math


def q_multiply(a, b):
    """Returns the Hamilton product a ⊗ b of two (w, x, y, z) quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def q_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def q_rotate(q, v):
    """Rotates vector v by quaternion q (body → world for an attitude q)."""
    p = q_multiply(q_multiply(q, (0.0, v[0], v[1], v[2])), q_conjugate(q))
    return (p[1], p[2], p[3])


def q_from_euler(roll, pitch, yaw):
    """Builds a quaternion from aerospace ZYX euler angles, in radians."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        cy * sp * cr + sy * cp * sr,
        sy * cp * cr - cy * sp * sr,
    )


def q_to_euler(q):
    """Returns aerospace ZYX (roll, pitch, yaw) in radians."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return (roll, pitch, yaw)


def q_angle(a, b):
    """Returns the rotation angle in radians between two unit quaternions."""
    dot = abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3])
    return 2 * math.acos(min(1.0, dot))


class MahonyAHRS:
    """
    Mahony's complementary attitude filter (gyro + accel, optionally + mag).

    Proportional feedback (kp) pulls the attitude toward the gravity and
    magnetic-field references; integral feedback (ki) absorbs slow gyro bias.
    """

    def __init__(self, kp=1.0, ki=0.005):
        self.kp = kp
        self.ki = ki
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.__ix = self.__iy = self.__iz = 0.0

    def update(self, gx, gy, gz, ax, ay, az, mx=None, my=None, mz=None, dt=0.02):
        """
        Advances the filter by dt seconds. Gyro in rad/s; accel and mag in any
        consistent units (they are normalized). Pass mx/my/mz as None (or all
        zeros) to run the 6-axis IMU update without a magnetometer.
        """
        q0, q1, q2, q3 = self.q

        use_mag = not (mx is None or my is None or mz is None or (mx == 0 and my == 0 and mz == 0))
        norm_a = math.sqrt(ax * ax + ay * ay + az * az)

        halfex = halfey = halfez = 0.0
        if norm_a > 0:
            ax, ay, az = ax / norm_a, ay / norm_a, az / norm_a

            # Estimated gravity direction in the body frame (halved)
            halfvx = q1 * q3 - q0 * q2
            halfvy = q0 * q1 + q2 * q3
            halfvz = q0 * q0 - 0.5 + q3 * q3

            halfex = ay * halfvz - az * halfvy
            halfey = az * halfvx - ax * halfvz
            halfez = ax * halfvy - ay * halfvx

            if use_mag:
                norm_m = math.sqrt(mx * mx + my * my + mz * mz)
                mx, my, mz = mx / norm_m, my / norm_m, mz / norm_m

                # Reference direction of Earth's field: rotate m into the
                # world frame and force it into the xz plane (bx, 0, bz)
                hx, hy, hz = q_rotate((q0, q1, q2, q3), (mx, my, mz))
                bx = math.sqrt(hx * hx + hy * hy)
                bz = hz

                # Estimated field direction back in the body frame (halved)
                halfwx = bx * (0.5 - q2 * q2 - q3 * q3) + bz * (q1 * q3 - q0 * q2)
                halfwy = bx * (q1 * q2 - q0 * q3) + bz * (q0 * q1 + q2 * q3)
                halfwz = bx * (q0 * q2 + q1 * q3) + bz * (0.5 - q1 * q1 - q2 * q2)

                halfex += my * halfwz - mz * halfwy
                halfey += mz * halfwx - mx * halfwz
                halfez += mx * halfwy - my * halfwx

            if self.ki > 0:
                self.__ix += 2 * self.ki * halfex * dt
                self.__iy += 2 * self.ki * halfey * dt
                self.__iz += 2 * self.ki * halfez * dt
                gx += self.__ix
                gy += self.__iy
                gz += self.__iz

            gx += 2 * self.kp * halfex
            gy += 2 * self.kp * halfey
            gz += 2 * self.kp * halfez

        # Integrate the quaternion rate: q̇ = ½ q ⊗ (0, ω)
        gx *= 0.5 * dt
        gy *= 0.5 * dt
        gz *= 0.5 * dt
        qa, qb, qc = q0, q1, q2
        q0 += -qb * gx - qc * gy - q3 * gz
        q1 += qa * gx + qc * gz - q3 * gy
        q2 += qa * gy - qb * gz + q3 * gx
        q3 += qa * gz + qb * gy - qc * gx

        norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        self.q = (q0 / norm, q1 / norm, q2 / norm, q3 / norm)
        return self.q

    def euler(self):
        """Returns (roll, pitch, yaw) of the current attitude, in radians."""
        return q_to_euler(self.q)


def reconstruct_az(ax, ay, sign=1.0):
    """
    Rebuilds the vertical accelerometer component from the 1 g constraint when
    the Z axis is dead (stuck MEMS proof mass): az = sign·√(1 − ax² − ay²).

    Valid for static and gently dynamic motion with tilts below 90°; beyond
    that the sign of az cannot be observed from ax/ay alone.
    """
    mag2 = ax * ax + ay * ay
    return math.copysign(math.sqrt(max(0.0, 1.0 - min(mag2, 1.0))), sign)
