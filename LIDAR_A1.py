import matplotlib.pyplot as plt
import numpy as np
from math import cos, sin, pi, floor
from adafruit_rplidar import RPLidar

# Setup the RPLidar
PORT_NAME = "/dev/ttyUSB0"
lidar = RPLidar(None, PORT_NAME, timeout=3)

# used to scale data to fit on the screen

def scan_lidar():
    AngleArr = []
    DistanceArr = []
    scan_data = [0] * 360
    scan_counter = 0
    try:
        for scan in lidar.iter_scans():
            for _, angle, distance in scan:
                scan_data[min([359, floor(angle)])] = distance
                DistanceArr.append(distance)
                AngleArr.append(angle)    
            scan_counter += 1
            
            print("Distance array =", DistanceArr)
            print("Angle Array = ", AngleArr)
            AngleArr = []
            DistanceArr = []
            
    except KeyboardInterrupt:
        print("Stopping.")
        lidar.stop()
        lidar.disconnect()
    
        restart = input("waiting  for enter")
        if restart == "":
            lidar.connect()
            scan_lidar()
scan_lidar()

