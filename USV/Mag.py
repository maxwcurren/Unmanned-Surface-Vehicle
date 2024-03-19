from array import array
from time import sleep
import math
import smbus
import time

# Scaling and offsetting factors for x and y
SCALE_X = 1
SCALE_Y = 1
OFFSET_X = 0
OFFSET_Y = 0

# Variables to track minimum and maximum values of x and y
X_MIN = 0
X_MAX = 0
Y_MIN = 0
Y_MAX = 0

# I2C bus configuration
I2C_BUS = 1

# Initialize the I2C bus
i2c = smbus.SMBus(I2C_BUS)
data = array('B', [0] * 6)

# Configure the sensor
i2c.write_byte_data(0x0D, 0xB, 0x01)
i2c.write_byte_data(0x0D, 0x9, 0x011101)

# Function to calculate heading in degrees
def calculate_heading(x, y):
    heading_rad = math.atan2(y, x)

    # reverse heading
    while heading_rad < 0:
        heading_rad += 2 * math.pi

    # wrapping
    while heading_rad > 2 * math.pi:
        heading_rad -= 2 * math.pi

    heading_deg = heading_rad * 180 / math.pi
    degrees = math.floor(heading_deg)
    return degrees

def get_bearing():
    global SCALE_X, SCALE_Y, OFFSET_X, OFFSET_Y, X_MIN, X_MAX, Y_MIN, Y_MAX
    
    try:
        sleep(0.2)
        for i in range(6):
            data[i] = i2c.read_byte_data(0x0D, 0x00 + i)

        x = (data[1] << 8) | data[0]
        y = (data[3] << 8) | data[2]
        z = (data[5] << 8) | data[4]

        x = x - (1 << 16) if x & (1 << 15) else x
        y = y - (1 << 16) if y & (1 << 15) else y
        z = z - (1 << 16) if z & (1 << 15) else z

        # Track minimum and maximum values of x and y
        X_MIN = min(x, X_MIN)
        X_MAX = max(x, X_MAX)
        Y_MIN = min(y, Y_MIN)
        Y_MAX = max(y, Y_MAX)

    except KeyboardInterrupt:
        print("Exited")

    x = x * SCALE_X + OFFSET_X
    y = y * SCALE_Y + OFFSET_Y

    # Calculate heading in degrees
    return calculate_heading(x, y)

if __name__ == "__main__":
    while True:
        start = time.time()
        heading_degrees = get_bearing()

        # Print the heading
        print('Heading: {}°'.format(heading_degrees))
        #print(f"elapsed time: {time.time()-start}")
        sleep(1)
