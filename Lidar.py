from math import floor
from adafruit_rplidar import RPLidar
import time

# Setup the RPLidar
PORT_NAME = "/dev/ttyUSB1"
lidar = RPLidar(None, '/dev/ttyUSB1', timeout=3)

def scan_lidar(duration):
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
                AngleArr.append(angle)
                DistanceArr.append(distance)

    except StopIteration:
        pass  # This exception is caught to exit the loop when the specified duration is reached
    finally:
        lidar.stop()
        lidar.disconnect()

    return DistanceArr, AngleArr

if __name__ == "__main__":
    scan_duration = 2  # Set the duration in seconds
    distances, angles = scan_lidar(scan_duration)

    print(len(distances))
    print(len(angles))
    print(f"distances: {distances}")
    print(f"angles: {angles}")
