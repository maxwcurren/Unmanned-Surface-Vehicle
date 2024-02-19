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

def mapSurge(surge):
    # MAPS SURGE FROM 0 TO 1023 ALL ODD NUMBERS
    pass
def mapYaw(yaw):
    # MAPS YAW FROM 0 TO 1022 ALL EVEN NUMBERS
    pass

def sendPIDControls(surge,yaw):
    # NEED TO IMPLEMENT SEND TO ARDUINO
    throttle = mapSurge(surge)
    steering = mapYaw(yaw)
    pass
    # send to arduino

#def createWaypoints()
#    # Choose number of waypoints
#    num_waypoints = int(input("Enter the amount of waypoints you will be using: "))
#    
#    # Create array of waypoints
#    waypoints_lon = []
#    waypoints_lat = []
#    for i in range(num_waypoints):
#        # Get longitude and latitude input from the user
#       # Will replace with better system once this code works
#        lon = float(input("Enter longitude for waypoint " + str(i + 1) + ": "))
#        lat = float(input("Enter latitude for waypoint " + str(i + 1) + ": ")) 
#            
#        # Append the coordinates as a list to the 2D array
#        waypoints_lon.append(lon)
#        waypoints_lat.append(lat)
#    
#    # Combine the lon and lat lists into a 2D array
#    waypoints = list(zip(waypoints_lon, waypoints_lat))
#    
#    # Print the waypoints array
#    print("Longitude\tLatitude")
#    for row in range(num_waypoints):
#        print(f"{waypoints[row][0]}\t\t{waypoints[row][1]}")
#    return waypoints_lon, waypoints_lat
