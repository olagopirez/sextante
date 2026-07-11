#!/usr/bin/env python3
"""Prints averaged MPU-9250 readings once per second."""

import time

from mpu9250 import MPU9250


def main():
    mpu = MPU9250()
    mpu.initialize()

    while True:
        time.sleep(1)
        print(mpu.get_avg().get_json())


if __name__ == '__main__':
    main()
