#!/usr/bin/env python
# THIS IS THE MAIN CODE FOR THE USV

# PID libraries
import math
import numpy as np
from scipy.integrate import odeint

# GUI libraries
import tkinter as tk
from PIL import Image, ImageTk

# Sensor libraries
import serial
import time
import struct
import re

# Import Scripts
import Accel_Mag
import GPS
import PID
from Waypoints import getWaypoints

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # set up serial output for Arduino
lora = serial.Serial('/dev/ttyS0', baudrate=115200, timeout=0)  # set up LoRa

QMC = Accel_mag.QMC5883L()
mpu6050 = Accel_mag.MPU6050()

def parse(sentence):
    # The format is "+RCV=ADDRESS,MESSAGE LENGTH,*request&mode^throttle%steering+"
    match = re.match(r'\+RCV=(\d+),(\d+),\*(\d+)&(\d+)\^(\d+)\%(\d+)\+', sentence)
    if match:
        req = int(match.group(3))
        mode = int(match.group(4))
        throttle = int(match.group(5))
        steering = int(match.group(6))

        if throttle > 1023:
            throttle = 1023
        if steering > 1022:
            steering = 1022

        if steering not in [1022, 510, 0]:
            if steering == 102:
                steering = 1022
            else:
                steering = 510
        return req, mode, throttle, steering
    else:
        return None

def transmit_Lora(data, max_retries=15):
    retry_count = 0

    while retry_count < max_retries:
        lora.write(('AT+SEND=99,' + str(len(data)) + ',' + data + ',\r\n').encode('utf-8'))
        time.sleep(0.1)
        ack_response = lora.readline().decode('utf-8').strip()

        if 'OK' in ack_response:
            print(f"Received acknowledgment: {ack_response}")
            print("Transmission successful.")
            break  # Successful acknowledgment, exit the loop
        else:
            retry_count += 1
            print(f"Retrying... (Retry {retry_count}/{max_retries})")
            time.sleep(0.1)

    if retry_count == max_retries:
        print(f"Failed to receive a successful acknowledgment after {max_retries} retries. Transmission unsuccessful.")

def receive_Lora():
    lora.write(('+RCV\r\n').encode('utf-8'))
    response = lora.readline().decode('utf-8').strip()
    return response

def send_arduino(throttle, steering):
    ser.write(struct.pack('<h', int(throttle)))
    ser.flush()
    ser.write(struct.pack('<h', int(steering)))
    ser.flush()

mode = 0
req = 0
throttle = 511
steering = 511

# Get waypoints:

image_path = 'map.png'
final_longitude, final_latitude = getWaypoints(image_path)
# Set PID Parameters:
Kp_surge, Ki_surge, Kd_surge = 11.59, 2.86, 1.88
Kp_yaw, Ki_yaw, Kd_yaw = 11.59, 2.86, 1.88
dt = 0.1    # Rate of sensor updates
# Initialize errors
prev_e_surge = 0
prev_e_yaw = 0

while True:
    # RECEIVE MANUAL CONTROLS OVER LORA
    time.sleep(0.1)
    response = receive_Lora()

    # Filter response
    if 'ERR' not in response:
        print(f"Unfiltered response: {response}")
        if '%' in response and '^' in response and '&' in response and '+' in response and '*' in response:
            # If data successfully sent over LoRa
            #print(response)
            result = parse(response)
            if result is not None:
                req, mode, throttle, steering = result
                print("mode is", mode)
                if mode == 1:
                    # IN MANUAL MODE
                    # Send motor controls to Arduino
                    print("throttle is", throttle)
                    print("steering is", steering)
                    send_arduino(throttle, steering)
                elif mode == 0:
                    # IN AUTO MODE
                    for i in range(num_waypoints - 1):
                        # Read GPS Module for coordinates
                        gps_lon, gps_lat = GPS.getGPS()
                        gps_lon1 = gps_lon / 10000
                        gps_lat1 = gps_lat / 10000
                        # Read magnometer
                        current_yaw = Accel_Mag.getMagno()
                        # Read accelerometer
                        current_surge = Accel_Mag.getAccel()
                        setpoint_surge = 1.0  # Surge setpoint (m/s)
                        output_surge, output_yaw, integral_surge, integral_yaw = PID.pid_controller(Kp_surge, Ki_surge, Kd_surge, Kp_yaw, Ki_yaw, Kd_yaw, setpoint_surge, setpoint_yaw, integral_surge, integral_yaw, prev_e_surge, prev_e_yaw, current_surge, current_yaw, dt)
                        
                        # MAP setpoints to 0 to 1022 for yaw and 0 to 1023 for surge and send to arduino for motor control
                        #NOT YET IMPLEMENTED
                        print("Function sendPIDControls not yet implemented")
                        PID.sendPIDControls(output_surge,output_yaw)

                        setpoint_yaw = PID.desired_heading(gps_lon1, gps_lat1, current_yaw, waypoints_lon[i], waypoints_lat[i])
                        prev_e_surge = setpoint_surge - current_surge
                        prev_e_yaw = setpoint_yaw - current_yaw
                        
                        # Send data if requested
                        if req == 1:
                            print("Data Request")
                            # Grab sensor values
                            # use dummy values:
                            longitude, latitude = GPS.getGPS()
                            orientation = math.floor(current_yaw)
                            data = '!' + str(gps_lon) + '&' + str(gps_lat) + '^' + orientation + '+'
                            time.sleep(0.1)
                        
                        # Wait for ACK before moving on
                        transmit_Lora(data)
                    else:
                        print("No Data request")
