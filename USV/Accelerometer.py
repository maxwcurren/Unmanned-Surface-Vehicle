import logging
import math
import time
import smbus
from time import sleep

class MPU6050:
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    INT_ENABLE = 0x38
    ACCEL_XOUT_H = 0x3B
    ACCEL_YOUT_H = 0x3D
    ACCEL_ZOUT_H = 0x3F
    GYRO_XOUT_H = 0x43
    GYRO_YOUT_H = 0x45
    GYRO_ZOUT_H = 0x47

    def __init__(self, bus_num=1, device_address=0x68):
        self.bus = smbus.SMBus(bus_num)
        self.Device_Address = device_address
        self.MPU_Init()

    def MPU_Init(self):
        self.bus.write_byte_data(self.Device_Address, self.SMPLRT_DIV, 7)
        self.bus.write_byte_data(self.Device_Address, self.PWR_MGMT_1, 1)
        self.bus.write_byte_data(self.Device_Address, self.CONFIG, 0)
        self.bus.write_byte_data(self.Device_Address, self.GYRO_CONFIG, 24)
        self.bus.write_byte_data(self.Device_Address, self.INT_ENABLE, 1)

    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.Device_Address, addr)
        low = self.bus.read_byte_data(self.Device_Address, addr + 1)
        value = (high << 8) | low
        if value > 32768:
            value -= 65536
        return value

    def read_accelerometer_data(self):
        acc_x = self.read_raw_data(self.ACCEL_XOUT_H)
        acc_y = self.read_raw_data(self.ACCEL_YOUT_H)
        acc_z = self.read_raw_data(self.ACCEL_ZOUT_H)
        Ax = acc_x / 16384.0
        Ay = acc_y / 16384.0
        Az = acc_z / 16384.0
        return Ax, Ay, Az

    def read_gyroscope_data(self):
        gyro_x = self.read_raw_data(self.GYRO_XOUT_H)
        gyro_y = self.read_raw_data(self.GYRO_YOUT_H)
        gyro_z = self.read_raw_data(self.GYRO_ZOUT_H)
        Gx = gyro_x / 131.0
        Gy = gyro_y / 131.0
        Gz = gyro_z / 131.0
        return Gx, Gy, Gz



def getAccel():
    Ax, Ay, Az = mpu6050.read_accelerometer_data()
return Ax, Ay, Az

def getGyro():
    Gx, Gy, Gz = mpu6050.read_gyroscope_data()
    return Gx, Gy, Gz

#if __name__ == "__main__":
#    mpu6050 = MPU6050()
#     
#    while True:
#        Ax, Ay, Az = mpu6050.read_accelerometer_data()
#        Gx, Gy, Gz = mpu6050.read_gyroscope_data()
#        print("Gx={:.2f} °/s \tGy={:.2f} °/s \tGz={:.2f} °/s \tAx={:.2f} g \tAy={:.2f} g \tAz={:.2f} g".format(Gx, Gy, Gz, Ax, Ay, Az))
#        sleep(1)


