from adafruit_rplidar import RPLidar
import time

# Setup the RPLidar
lidar = RPLidar(None, '/dev/ttyUSB1', timeout=3)

def scan_lidar(duration, max_distance=2000):
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
            scan_duration = 5  # Set the duration in seconds
            max_allowed_distance = 2000  # Set the maximum allowed distance in mm
            distances, angles = scan_lidar(scan_duration, max_allowed_distance)

            #print(len(distances))
            #print(len(angles))
            print(f"distances: {distances}")
            print(f"angles: {angles}")
        except:
            print("error")
