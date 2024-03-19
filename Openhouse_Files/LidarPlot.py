import matplotlib.pyplot as plt
import numpy as np
from adafruit_rplidar import RPLidar
import time
from math import pi

# Setup the RPLidar
lidar = RPLidar(None, '/dev/ttyUSB0', timeout=3)

# Create an initial empty plot
plt.figure()
scatter = plt.scatter([], [], s=1, c='b', marker='.')
plt.scatter(0, 0, s=20, c='r', marker='o')  # Add a red dot at the center
plt.gca().set_aspect('equal', adjustable='box')
plt.xlim(-2000, 2000)
plt.ylim(-2000, 2000)
plt.show(block=False)  # Show the plot without blocking the code execution

def scan_lidar(duration, max_distance=500):
    lidar.connect()
    time.sleep(0.1)
    AngleArr = []
    DistanceArr = []
    start_time = time.time()

    try:
        for scan in lidar.iter_scans():
            for _, angle, distance in scan:
                if time.time() - start_time > duration:
                    raise StopIteration  # Stop the iteration after the specified duration

                if distance <= max_distance:
                    AngleArr.append(angle)
                    DistanceArr.append(distance)

    except StopIteration:
        pass  # This exception is caught to exit the loop when the specified duration is reached
    finally:
        lidar.stop()
        lidar.disconnect()

    return DistanceArr, AngleArr

if __name__ == "__main__":
    while True:
        try:
            scan_duration = 3  # Set the duration in seconds
            max_allowed_distance = 1500  # Set the maximum allowed distance in mm
            distances, angles = scan_lidar(scan_duration, max_allowed_distance)
            print(len(distances))
            x = distances * np.cos(np.radians(angles) + pi/2)
            y = distances * np.sin(np.radians(angles)+ pi/2)
            scatter.set_offsets(np.column_stack((x, y)))
            plt.pause(0.001)  # Pause to allow the plot to update

        except Exception as e:
            print(f"Error: {e}")
