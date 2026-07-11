import pytest

from mpu9250 import MPU9250


class FakeSMBus:
    """In-memory stand-in for smbus2.SMBus: records writes, serves canned reads."""

    def __init__(self):
        self.byte_writes = []   # (i2c_addr, register, value)
        self.block_writes = []  # (i2c_addr, register, [data])
        self.byte_regs = {}     # (i2c_addr, register) -> byte
        self.word_regs = {}     # (i2c_addr, register) -> raw SMBus word (little-endian)

    def write_byte_data(self, i2c_addr, register, value):
        self.byte_writes.append((i2c_addr, register, value))
        self.byte_regs[(i2c_addr, register)] = value

    def read_byte_data(self, i2c_addr, register):
        return self.byte_regs.get((i2c_addr, register), 0)

    def read_word_data(self, i2c_addr, register):
        return self.word_regs.get((i2c_addr, register), 0)

    def write_i2c_block_data(self, i2c_addr, register, data):
        self.block_writes.append((i2c_addr, register, list(data)))


@pytest.fixture
def fake_bus():
    return FakeSMBus()


@pytest.fixture
def mpu(fake_bus):
    return MPU9250(bus=fake_bus)
