import math

import pytest

from mpu9250.demo import DemoMPU
from mpu9250.fusion import (
    MahonyAHRS,
    q_angle,
    q_from_euler,
    q_multiply,
    q_rotate,
    q_to_euler,
)

DEG = math.pi / 180


class TestQuaternionHelpers:
    def test_identity_multiplication(self):
        q = q_from_euler(0.3, -0.2, 1.1)
        assert q_multiply((1, 0, 0, 0), q) == pytest.approx(q)

    def test_rotation_of_a_vector(self):
        # 90° about z sends x to y
        q = q_from_euler(0, 0, math.pi / 2)
        v = q_rotate(q, (1, 0, 0))
        assert v == pytest.approx((0, 1, 0), abs=1e-9)

    def test_euler_round_trip(self):
        angles = (0.4, -0.3, 1.2)
        assert q_to_euler(q_from_euler(*angles)) == pytest.approx(angles)

    def test_angle_between_quaternions(self):
        a = q_from_euler(0, 0, 0)
        b = q_from_euler(0, 0, math.pi / 2)
        assert q_angle(a, b) == pytest.approx(math.pi / 2)


class TestMahonyAHRS:
    def test_stays_at_identity_when_flat(self):
        ahrs = MahonyAHRS()
        for _ in range(200):
            ahrs.update(0, 0, 0, 0, 0, 1, 22, 0, -31, dt=0.02)
        assert q_angle(ahrs.q, (1, 0, 0, 0)) < 1 * DEG

    def test_converges_to_a_tilted_gravity(self):
        # device rolled 30°: gravity appears at (0, sin30, cos30) in the body
        ahrs = MahonyAHRS(kp=2.0)
        for _ in range(1500):
            ahrs.update(0, 0, 0, 0.0, math.sin(30 * DEG), math.cos(30 * DEG), dt=0.02)
        g_est = q_rotate((ahrs.q[0], -ahrs.q[1], -ahrs.q[2], -ahrs.q[3]), (0, 0, 1))
        dot = g_est[1] * math.sin(30 * DEG) + g_est[2] * math.cos(30 * DEG)
        assert dot > 0.999

    def test_integrates_pure_gyro_rotation(self):
        # 90°/s about body z for 1 s with no correction references
        ahrs = MahonyAHRS()
        for _ in range(100):
            ahrs.update(0, 0, 90 * DEG, 0, 0, 0, dt=0.01)
        _, _, yaw = ahrs.euler()
        assert yaw == pytest.approx(90 * DEG, abs=2 * DEG)

    def test_tracks_the_demo_motion(self):
        # closed loop: the demo generates consistent gyro/accel/mag from a
        # known attitude; the filter must track it within a few degrees
        mpu = DemoMPU(noisy=False)
        ahrs = MahonyAHRS(kp=2.0)
        dt = 0.02
        t = 0.0
        for _ in range(int(30 / dt)):
            gx, gy, gz, ax, ay, az, mx, my, mz, _ = mpu._DemoMPU__sample(t)
            ahrs.update(gx * DEG, gy * DEG, gz * DEG, ax, ay, az, mx, my, mz, dt=dt)
            t += dt
        assert q_angle(ahrs.q, mpu.attitude(t)) < 8 * DEG


class TestReconstructAz:
    def test_level_gives_full_gravity(self):
        from mpu9250.fusion import reconstruct_az
        assert reconstruct_az(0.0, 0.0, 1.0) == pytest.approx(1.0)
        assert reconstruct_az(0.0, 0.0, -1.0) == pytest.approx(-1.0)

    def test_tilt_follows_the_1g_sphere(self):
        from mpu9250.fusion import reconstruct_az
        assert reconstruct_az(math.sin(30 * DEG), 0.0, 1.0) == pytest.approx(math.cos(30 * DEG))

    def test_clamps_beyond_1g(self):
        from mpu9250.fusion import reconstruct_az
        assert reconstruct_az(1.5, 0.0, 1.0) == 0.0
