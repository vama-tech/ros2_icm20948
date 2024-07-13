from __future__ import print_function
import qwiic_icm20948
from qwiic_icm20948 import QwiicIcm20948
import time
import sys


if __name__ == "__main__":

    IMU = QwiicIcm20948(0x69)
    connect = IMU.connected(0x69)
    print(connect)
    # IMU.setBank(2)
    # IMU._i2c.writeByte(IMU.address, IMU.AGB2_REG_ACCEL_CONFIG_1, 0x23)	
    # register = IMU._i2c.readByte(IMU.address, IMU.AGB2_REG_ACCEL_CONFIG_1)
    # print(register)