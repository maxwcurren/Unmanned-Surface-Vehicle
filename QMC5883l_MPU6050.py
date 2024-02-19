import logging
import math
import time
import smbus
from time import sleep

#QMC5883L addresses
DFLT_BUS = 1
DFLT_ADDRESS = 0x0d

REG_XOUT_LSB = 0x00     # Output Data Registers for magnetic sensor.
REG_XOUT_MSB = 0x01
REG_YOUT_LSB = 0x02
REG_YOUT_MSB = 0x03
REG_ZOUT_LSB = 0x04
REG_ZOUT_MSB = 0x05
REG_STATUS_1 = 0x06     # Status Register.
REG_TOUT_LSB = 0x07     # Output Data Registers for temperature.
REG_TOUT_MSB = 0x08
REG_CONTROL_1 = 0x09    # Control Register #1.
REG_CONTROL_2 = 0x0a    # Control Register #2.
REG_RST_PERIOD = 0x0b   # SET/RESET Period Register.
REG_CHIP_ID = 0x0d      # Chip ID register.

# Flags for Status Register #1.
STAT_DRDY = 0b00000001  # Data Ready.
STAT_OVL = 0b00000010   # Overflow flag.
STAT_DOR = 0b00000100   # Data skipped for reading.

# Flags for Status Register #2.
INT_ENB = 0b00000001    # Interrupt Pin Enabling.
POL_PNT = 0b01000000    # Pointer Roll-over.
SOFT_RST = 0b10000000   # Soft Reset.

# Flags for Control Register 1.
MODE_STBY = 0b00000000  # Standby mode.
MODE_CONT = 0b00000001  # Continuous read mode.
ODR_10HZ = 0b00000000   # Output Data Rate Hz.
ODR_50HZ = 0b00000100
ODR_100HZ = 0b00001000
ODR_200HZ = 0b00001100
RNG_2G = 0b00000000     # Range 2 Gauss: for magnetic-clean environments.
RNG_8G = 0b00010000     # Range 8 Gauss: for strong magnetic fields.
OSR_512 = 0b00000000    # Over Sample Rate 512: less noise, more power.
OSR_256 = 0b01000000
OSR_128 = 0b10000000
OSR_64 = 0b11000000     # Over Sample Rate 64: more noise, less power.

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


class QMC5883L(object):
    def __init__(self,
                 i2c_bus=DFLT_BUS,
                 address=DFLT_ADDRESS,
                 output_data_rate=ODR_10HZ,
                 output_range=RNG_2G,
                 oversampling_rate=OSR_512):
        self.address = address
        self.bus = smbus.SMBus(i2c_bus)
        self.output_range = output_range
        self._declination = 0.0
        self._calibration = [[1.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0],
                             [0.0, 0.0, 1.0]]
        chip_id = self._read_byte(REG_CHIP_ID)
        if chip_id != 0xff:
            msg = "wrong chip?"
            logging.warning(msg, chip_id)
        self.mode_cont = (MODE_CONT | output_data_rate | output_range
                          | oversampling_rate)
        self.mode_stby = (MODE_STBY | ODR_10HZ | RNG_2G | OSR_64)
        self.mode_continuous()

    def __del__(self):
        """Once finished switch to standby mode."""
        self.mode_standby()

    def mode_continuous(self):
        "continuous read mode."
        self._write_byte(REG_CONTROL_2, SOFT_RST)  # Soft reset.
        self._write_byte(REG_CONTROL_2, INT_ENB)  # Disable interrupt.
        self._write_byte(REG_RST_PERIOD, 0x01)  # Define SET/RESET period.
        self._write_byte(REG_CONTROL_1, self.mode_cont)  # Set operation mode.

    def mode_standby(self):
        "Set the device in standby mode."
        self._write_byte(REG_CONTROL_2, SOFT_RST)
        self._write_byte(REG_CONTROL_2, INT_ENB)
        self._write_byte(REG_RST_PERIOD, 0x01)
        self._write_byte(REG_CONTROL_1, self.mode_stby)  # Set operation mode.

    def _write_byte(self, registry, value):
        self.bus.write_byte_data(self.address, registry, value)
        time.sleep(0.01)

    def _read_byte(self, registry):
        return self.bus.read_byte_data(self.address, registry)

    def _read_word(self, registry):
        "Read a two bytes value stored as LSB and MSB."
        low = self.bus.read_byte_data(self.address, registry)
        high = self.bus.read_byte_data(self.address, registry + 1)
        val = (high << 8) + low
        return val

    def _read_word_2c(self, registry):
        "Gets 2's complement of a two bytes value."
        val = self._read_word(registry)
        if val >= 0x8000:  # 32768
            return val - 0x10000  # 65536
        else:
            return val

    def get_data(self):
        "Read data from magnetic and temperature data registers."
        i = 0
        [x, y, z, t] = [None, None, None, None]
        while i < 20:  # Timeout after about 0.20 seconds.
            status = self._read_byte(REG_STATUS_1)
            if status & STAT_OVL:
                # Some values have reached an overflow.
                msg = ("Overflow.")
                if self.output_range == RNG_2G:
                    msg += " Change RNG_8G output range."
                logging.warning(msg)
            if status & STAT_DOR:
                # Previous measure was read partially, sensor in Data Lock.
                x = self._read_word_2c(REG_XOUT_LSB)
                y = self._read_word_2c(REG_YOUT_LSB)
                z = self._read_word_2c(REG_ZOUT_LSB)
                continue
            if status & STAT_DRDY:
                # Data is ready to read.
                x = self._read_word_2c(REG_XOUT_LSB)
                y = self._read_word_2c(REG_YOUT_LSB)
                z = self._read_word_2c(REG_ZOUT_LSB)
                t = self._read_word_2c(REG_TOUT_LSB)
                break
            else:
                # Waiting for DRDY.
                time.sleep(0.01)
                i += 1
        return [x, y, z, t]

    def get_magnet_raw(self):
        "Get the 3 axis values."
        [x, y, z, t] = self.get_data()
        return [x, y, z]

    def get_magnet(self):
        "Return the horizontal magnetic sensor vector with (x, y) calibration applied."
        [x, y, z] = self.get_magnet_raw()
        if x is None or y is None:
            [x1, y1] = [x, y]
        else:
            c = self._calibration
            x1 = x * c[0][0] + y * c[0][1] + c[0][2]
            y1 = x * c[1][0] + y * c[1][1] + c[1][2]
        return [x1, y1]

    def get_bearing_raw(self):
        "Horizontal bearing (in degrees) from magnetic value X and Y."
        [x, y, z] = self.get_magnet_raw()
        if x is None or y is None:
            return None
        else:
            b = math.degrees(math.atan2(y, x))
            if b < 0:
                b += 360.0
            return b

    def get_bearing(self):
        "Horizontal bearing, adjusted by calibration and declination."
        [x, y] = self.get_magnet()
        if x is None or y is None:
            return None
        else:
            b = math.degrees(math.atan2(y, x))
            if b < 0:
                b += 360.0
            b += self._declination
            if b < 0.0:
                b += 360.0
            elif b >= 360.0:
                b -= 360.0
        return b

    def set_declination(self, value):
        "Set the magnetic declination, in degrees."
        try:
            d = float(value)
            if d < -180.0 or d > 180.0:
                logging.error(u'Declination must be >= -180 and <= 180.')
            else:
                self._declination = d
        except:
            logging.error(u'Declination must be a float value.')

    def get_declination(self):
        "Return the current set value of magnetic declination."
        return self._declination

    def set_calibration(self, value):
        "Set the 3x3 matrix for horizontal (x, y) magnetic vector calibration."
        c = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        try:
            for i in range(0, 3):
                for j in range(0, 3):
                    c[i][j] = float(value[i][j])
            self._calibration = c
        except:
            logging.error(u'Calibration must be a 3x3 float matrix.')

    def get_calibration(self):
        "Return the current set value of the calibration matrix."
        return self._calibration

    declination = property(fget=get_declination,
                           fset=set_declination,
                           doc=u'Magnetic declination to adjust bearing.')

    calibration = property(fget=get_calibration,
                           fset=set_calibration,
                           doc=u'Transformation matrix to adjust (x, y) magnetic vector.')

def getAccel():
    Ax, Ay, Az = mpu6050.read_accelerometer_data()
return Ax, Ay, Az

def getGyro():
    Gx, Gy, Gz = mpu6050.read_gyroscope_data()
    return Gx, Gy, Gz
def getMagno():
    bearing = QMC.get_bearing()
return bearing

#if __name__ == "__main__":
#    QMC = QMC5883L()
#    mpu6050 = MPU6050()
#     
#    while True:
#        bearing = QMC.get_bearing()
#        Ax, Ay, Az = mpu6050.read_accelerometer_data()
#        Gx, Gy, Gz = mpu6050.read_gyroscope_data()
#        print("Gx={:.2f} °/s \tGy={:.2f} °/s \tGz={:.2f} °/s \tAx={:.2f} g \tAy={:.2f} g \tAz={:.2f} g \tBearing: {:.2f} degrees".format(Gx, Gy, Gz, Ax, Ay, Az, bearing))
#        sleep(1)


