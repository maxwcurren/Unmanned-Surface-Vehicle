import matplotlib.pyplot as plt
import numpy as np
from math import cos, sin, pi, floor
from adafruit_rplidar import RPLidar

# Setup the RPLidar
PORT_NAME = "/dev/ttyUSB0"
lidar = RPLidar(None, PORT_NAME, timeout=3)

# Create an initial empty plot
plt.figure()
plt.gca().set_aspect('equal', adjustable='box')
plt.xlim(-2000, 2000)
plt.ylim(-2000, 2000)
scatter = plt.scatter([], [], s=1, c='b', marker='.')
scatter2 = plt.scatter([], [], s=1, c='r', marker='.')

plt.show(block=False)  # Show the plot without blocking the code execution

def process_data(data):
    points = []
    for angle, distance in enumerate(data):
        #and (angle>= 0 and angle <= 30)
        if distance > 0 and distance < 1001:
        #if distance < 0 and (angle>= 330 and angle <= 360) or (angle>= 0 and angle <= 30):
            radians = angle * pi / 180.0
            x = distance * cos(radians + pi/2)
            y = distance * sin(radians + pi/2)
            point = (x, y)
            points.append(point)
    points = np.array(points)
    scatter2.set_offsets(points)
    plt.pause(0.001)  # Pause to allow the plot to update

scan_data = [0] * 360
count1 = 0
count2 = 0

try:
    for scan in lidar.iter_scans():
        for quality, angle, distance in scan:
            scan_data[min([359, floor(angle)])] = distance
        process_data(scan_data)

except:
    print("Stopping.")
    pass


