import math
import numpy as np
from scipy.integrate import odeint
import math
import serial
import re

def pid_controller(Kp_surge, Ki_surge, Kd_surge, Kp_yaw, Ki_yaw, Kd_yaw, setpoint_surge, setpoint_yaw, integral_surge, integral_yaw, prev_e_surge, prev_e_yaw, current_value_surge, current_value_yaw, dt):
    e_surge = setpoint_surge - current_value_surge
    integral_surge += e_surge * dt
    derivative_surge = (e_surge - prev_e_surge) / dt
    
    output_surge = Kp_surge * e_surge + Ki_surge * integral_surge + Kd_surge * derivative_surge
    
    e_yaw = setpoint_yaw - current_value_yaw
    integral_yaw += e_yaw * dt
    derivative_yaw = (e_yaw - prev_e_yaw) / dt
    
    output_yaw = Kp_yaw * e_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw
    
    return output_surge, output_yaw, integral_surge, integral_yaw

def desired_heading(current_lon, current_lat, current_yaw, waypoint_lon, waypoint_lat):
    # Calculate the differences in x and y coordinates
    delta_lon = waypoint_lon - current_lon
    delta_lat = waypoint_lat - current_lat

    # Calculate the yaw angle using arctangent
    target_yaw = math.atan2(delta_lon, delta_lat)

    # Calculate the setpoint_yaw by adding the current_yaw
    desired_heading = target_yaw + current_yaw

    return desired_heading

# SENSOR DATA AQUISITION
def parse_gpgll(sentence):
    match = re.match(r'\$GPGLL,(\d+\.\d+),([NS]),(\d+\.\d+),([EW]),.*', sentence)
    if match:
        latitude = float(match.group(1))
        if match.group(2) == 'S':
            latitude = -latitude
        longitude = float(match.group(3))
        if match.group(4) == 'W':
            longitude = -longitude
        return latitude / 100, longitude / 100
    else:
        return None

def getGPS():
    ser = serial.Serial(port='/dev/ttyS0', baudrate=9600, timeout=1)

    data = ser.readline().decode('utf-8').strip()
    if data.startswith("$GPGLL"):
        latitude, longitude = parse_gpgll(data)
        if latitude is not None and longitude is not None:
            print(f"Latitude: {latitude}, Longitude: {longitude}")
    return longitude, latitude
def getMagno():
    # NEED TO GET CODE FROM ALYAN
    pass
def getAccel():
    # NEED TO GET CODE FROM ALYAN
    pass

def mapSurge(surge):
    # MAPS SURGE FROM 0 TO 1023 ALL ODD NUMBERS
    pass
def mapYaw(yaw):
    # MAPS YAW FROM 0 TO 1022 ALL EVEN NUMBERS
    pass

def sendControls(surge,yaw):
    # NEED TO IMPLEMENT SEND TO ARDUINO
    throttle = mapSurge(surge)
    steering = mapYaw(yaw)
    # send to arduino


# Create Array of Waypoints
# Choose number of waypoints
num_waypoints = int(input("Enter the amount of waypoints you will be using: "))

# Create array of waypoints
waypoints_lon = []
waypoints_lat = []
for i in range(num_waypoints):
    # Get longitude and latitude input from the user
    # Will replace with better system once this code works
    lon = float(input("Enter longitude for waypoint " + str(i + 1) + ": "))
    lat = float(input("Enter latitude for waypoint " + str(i + 1) + ": ")) 
        
    # Append the coordinates as a list to the 2D array
    waypoints_lon.append(lon)
    waypoints_lat.append(lat)

# Combine the lon and lat lists into a 2D array
waypoints = list(zip(waypoints_lon, waypoints_lat))

# Print the waypoints array
print("Longitude\tLatitude")
for row in range(num_waypoints):
    print(f"{waypoints[row][0]}\t\t{waypoints[row][1]}")

# PID parameters
Kp_surge, Ki_surge, Kd_surge = 11.59, 2.86, 1.88
Kp_yaw, Ki_yaw, Kd_yaw = 11.59, 2.86, 1.88
dt = 0.1    # Rate of sensor updates
# Initialize errors
prev_e_surge = 0
prev_e_yaw = 0

while True:
    for i in range(num_waypoints - 1):
        # Read GPS Module for coordinates
        gps_lon, gps_lat = getGPS()
        # Read magnometer
        current_yaw = getMagno()
        # Read accelerometer
        current_surge = getAccel()
        setpoint_surge = 1.0  # Surge setpoint (m/s)
        output_surge, output_yaw, integral_surge, integral_yaw = pid_controller(Kp_surge, Ki_surge, Kd_surge, Kp_yaw, Ki_yaw, Kd_yaw, setpoint_surge, setpoint_yaw, integral_surge, integral_yaw, prev_e_surge, prev_e_yaw, current_surge, current_yaw, dt)
        
        # MAP setpoints to 0 to 1022 for yaw and 0 to 1023 for surge and send to arduino for motor control
        sendControls(output_surge,output_yaw)

        setpoint_yaw = desired_heading(gps_lon, gps_lat, current_yaw, waypoints_lon[i], waypoints_lat[i])
        prev_e_surge = setpoint_surge - current_surge
        prev_e_yaw = setpoint_yaw - current_yaw
        
