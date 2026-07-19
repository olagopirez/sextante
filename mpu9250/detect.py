"""IMU auto-detection: which supported chip is answering on the bus."""

from .constants import MPU9250_ID, MPU9255_ID, MPU_ADDRESS, MPUREG_WHOAMI
from .driver import HardwareMismatchError
from .lsm9ds1 import LSM9DS1_AG_ADDRESS, LSM9DS1_AG_ID, REG_WHO_AM_I


def detect_imu(bus):
    """
    Probes the bus for a supported IMU and returns ``'mpu9250'`` or
    ``'lsm9ds1'``. Raises HardwareMismatchError with everything it saw when
    no supported chip answers.
    """
    seen = []

    try:
        chip_id = bus.read_byte_data(MPU_ADDRESS, MPUREG_WHOAMI)
        if chip_id in (MPU9250_ID, MPU9255_ID):
            return 'mpu9250'
        seen.append(f'0x{MPU_ADDRESS:02X} answered id 0x{chip_id:02X}')
    except OSError:
        pass

    try:
        chip_id = bus.read_byte_data(LSM9DS1_AG_ADDRESS, REG_WHO_AM_I)
        if chip_id == LSM9DS1_AG_ID:
            return 'lsm9ds1'
        seen.append(f'0x{LSM9DS1_AG_ADDRESS:02X} answered id 0x{chip_id:02X}')
    except OSError:
        pass

    detail = '; '.join(seen) or 'no device answered at 0x68 or 0x6A'
    raise HardwareMismatchError(f'no supported IMU found ({detail})')
