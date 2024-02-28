import time
from Accel_Mag import QMC5883L, MPU6050
import GPS
import serial
import Lidar
import math
import struct
import re

QMC = QMC5883L()
mpu6050 = MPU6050()
ser = serial.Serial('/dev/ttyACM0', 1200, timeout=1)  # set up serial output for Arduino
lora = serial.Serial('/dev/ttyS0', baudrate=115200, timeout=0)  # set up LoRa

# Get starting position of USV
starting_lon, starting_lat = GPS.getGPS()
print(f"Starting Longitude: {starting_lon/10000.0}, Starting Latitude: {starting_lat/10000.0}")

# return_method will determine how the USV returns to the base station. It will be determined on creation of waypoint arrays.
# If return_method is zero then the USV will return to the starting position by iterating through the waypoints in reverse.
# If return_method is one then the USV will directly calculate the heading towards the starting position and stop when it arrives.

# Define globals
waypoint_count = 0
mode = 1
req = 0
throttle = 511
steering = 511
distances_prev = []
distances = []
angles_prev = []
angles = []

# Define PID variables:
Kp_yaw, Kd_yaw = 2, 1.88
prev_error = 0.0
cumulative_error = 0.0

# PATHING FUNCTIONS
def detectObject(current_heading, distances, angles):
    print(f"Current Heading: {current_heading}")
    
    # Find the difference between the current_heading and each angle in angles
    angle_diff = [abs(current_heading - angle) for angle in angles]
    
    # Find the index of the minimum difference
    min_diff_index = angle_diff.index(min(angle_diff))
    
    # Get the nearest angle and corresponding distance
    nearest_angle = angles[min_diff_index]
    nearest_distance = distances[min_diff_index]

    if current_heading - 15 <= nearest_angle <= current_heading + 15:
        # If there is an object detected in the current path of the USV
        object_detected = 1
        if nearest_angle <= current_heading:
            # If the object is to the right of the threshold window or directly in front, then adjust heading of the USV to the left.
            new_heading = (current_heading + 30) % 360
        else:
            # If the object is to the left of the threshold window adjust the heading of the USV to the right.
            new_heading = (current_heading - 30) % 360
    else:
        object_detected = 0
        new_heading = current_heading

    print(f"Object Detected: {object_detected}")
    return object_detected, new_heading

def getNextHeading(current_heading, distances, angles, waypoint_lon, waypoint_lat, current_lon, current_lat, return_method):
    global waypoint_count, waypoint_num, starting_lon, starting_lat
    # Calculate distance to waypoint
    delta_lon = waypoint_lon - current_lon
    delta_lat = waypoint_lat - current_lat
    distance_to_waypoint = math.sqrt((delta_lon)**2 + (delta_lat)**2)

    if distance_to_waypoint < 10.0:
        waypoint_count += 1

    if waypoint_count <= waypoint_num:
        object_detected, pot_heading = detectObject(current_heading, distances, angles)

        if object_detected == 1:
            new_heading = pot_heading
        else:
            pid_output = pid_controller(current_heading, waypoint_lon, waypoint_lat, current_lon, current_lat)
            # Add saturation limits to prevent sudden adjustments
            # pid_output = max(min(pid_output, max_output), min_output)
            new_heading = pid_output

        return new_heading

    # Check if it's time to return
    elif waypoint_count == waypoint_num + 1:
        if return_method == 0:  # Iterate through waypoints in reverse
            waypoint_count -= 1
            object_detected, pot_heading = detectObject(current_heading, distances, angles)

            if object_detected == 1:
                new_heading = pot_heading
            else:
                pid_output = pid_controller(current_heading, starting_lon, starting_lat, current_lon, current_lat)
                new_heading = current_heading + pid_output

            return new_heading

        elif return_method == 1:  # Directly calculate heading towards starting position
            delta_starting_lon = starting_lon - current_lon
            delta_starting_lat = starting_lat - current_lat
            return_heading = math.atan2(delta_starting_lon, delta_starting_lat)
            return return_heading

def pid_controller(current_heading, waypoint_lon, waypoint_lat, current_lon, current_lat):
    global prev_error, Kp_yaw, Kd_yaw
    
    # Calculate the differences in x and y coordinates
    delta_lon, delta_lat = waypoint_lon - current_lon, waypoint_lat - current_lat
    
    # Calculate the yaw angle using arctangent
    target_heading = math.atan2(delta_lon, delta_lat)
    
    # Normalize the target_heading to the range [0, 360)
    target_heading_degrees = (math.degrees(target_heading) + 360) % 360
    print(f"Ideal Heading: {target_heading_degrees}")
    # Calculate the error
    error = target_heading_degrees - current_heading
    
    # Calculate the PD components
    proportional = Kp_yaw * error
    derivative = Kd_yaw * (error - prev_error)
    print(f"Proportional: {proportional}")
    print(f"Derivative: {derivative}")
    
    # Calculate the PD output
    pid_output = proportional + derivative
    # Update previous error
    prev_error = error
    
    return pid_output

def mapYaw(yaw):
    # MAPS YAW FROM 0 TO 1022 ALL EVEN NUMBERS
    print(f"yaw: {yaw}")
    mapped_value = int(yaw * (1022 / 360))
    mapped_value = mapped_value - (mapped_value % 2)
    print(f"Mapped Steering: {mapped_value}")
    return mapped_value

# LORA FUNCTIONS
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

        if steering not in [1022, 510, 2]:
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
            #print(f"Retrying... (Retry {retry_count}/{max_retries})")
            time.sleep(0.1)

    if retry_count == max_retries:
        print(f"Failed to receive a successful acknowledgment after {max_retries} retries. Transmission unsuccessful.")

def receive_Lora():
    lora.write(('+RCV\r\n').encode('utf-8'))
    response = lora.readline().decode('utf-8').strip()
    return response

def receive_Way_Ret():
    lon = []
    lat = []
    ret = 2
    print("Waiting for waypoints")
    while not (lon and lat and ret != 2):
        response = receive_Lora()
        print(f"lon: {lon}")
        print(f"lat: {lat}")
        print(f"ret: {ret}")
        if 'ERR' not in response:
            if '!' in response:
                if '_' in response:
                    result = parse_lon(response)
                    if result is not None:
                        lon, ret = result
                elif '&' in response:
                    result = parse_lat(response)
                    if result is not None:
                        lat, ret = result
        time.sleep(0.1)
    return lon, lat, ret

def parse_lon(response):
    # Remove the prefix
    response = response.split("_")[1]

    # Split the response string based on "?"
    parts = response.split("?")
    
    # Initialize lon as an empty list
    lon = []

    # Remove everything after "!" in parts and store in lon
    for part in parts:
        lon.append(part.split("!")[0])

    # lon_values is an array of integers where each element is an integer in lon that is separated by commas
    lon_values = [int(value) / 10000 for value in lon]
    
    # Remove everything before "!" in parts and store in r
    r = [part.split("!")[1] for part in parts if "!" in part]
    r0 = r[0]
    
    # Extract the value after "!"
    ret = int(r0[0]) if r0 else None

    return lon_values, ret
    
def parse_lat(response):
    # Remove the prefix
    response = response.split("&")[1]

    # Split the response string based on "?"
    parts = response.split("?")
    
    # Initialize lon as an empty list
    lat = []

    # Remove everything after "!" in parts and store in lon
    for part in parts:
        lat.append(part.split("!")[0])

    # lon_values is an array of integers where each element is an integer in lon that is separated by commas
    lat_values = [int(value) / 10000 for value in lat]
    
    # Remove everything before "!" in parts and store in r
    r = [part.split("!")[1] for part in parts if "!" in part]
    r0 = r[0]
    
    # Extract the value after "!"
    ret = int(r0[0]) if r0 else None

    return lat_values, ret
    
def send_arduino(throttle, steering):
    ser.write(struct.pack('<h', int(throttle)))
    ser.flush()
    print(f"Steering Sent to Arduino: {steering}")
    ser.write(struct.pack('<h', int(steering)))
    ser.flush()

def manual():
    global throttle, steering
    # IN MANUAL MODE
    send_arduino(throttle, steering)
def auto():
    global throttle, steering, req, distances, distances_prev, angles, angles_prev
    # IN AUTO MODE
    throttle = 711
    # Read GPS Module for coordinates
    time_before = time.time()
    gps_lon, gps_lat = GPS.getGPS()
    gps_lon1 = gps_lon / 10000
    gps_lat1 = gps_lat / 10000
    # Read magnetometer
    current_yaw = QMC.get_bearing()

    # Check for objects
    distances, angles = Lidar.scan_lidar()
    time_after = time.time()
    sensor_time = time_after - time_before

    if len(distances) == 0:
        distances = distances_prev.copy()
        angles = angles_prev.copy() 
    else:
        distances_prev = distances.copy() 
        angles_prev = angles.copy() 

    # Get target yaw using PID or object detected function
    target_yaw = getNextHeading(current_yaw, distances, angles, waypoint_lon[waypoint_count], waypoint_lat[waypoint_count], gps_lon1, gps_lat1, return_method)
    # MAP setpoints to 0 to 1022 for yaw and send to Arduino for motor control
    steering = mapYaw(target_yaw)

    # Send motor controls to Arduino
    send_arduino(throttle, steering)

    # Send data if requested
    if req == 1:
        print("Data Request")
        data = '*' + str(gps_lon) + '&' + str(gps_lat) + '^' + str(math.floor(current_yaw)) + '+'            
        # Wait for ACK before moving on
        transmit_Lora(data)
        req = 0
    else:
        print("No Data request")

# Get waypoints:
waypoint_lon, waypoint_lat, return_method = receive_Way_Ret()
print("Received Waypoints")
waypoint_num = len(waypoint_lon)

while True:
    # RECEIVE MANUAL CONTROLS OVER LORA
    time.sleep(0.1)
    response = receive_Lora()
    # Filter response
    if 'ERR' not in response:
        if '%' in response and '^' in response and '&' in response and '+' in response and '*' in response:
            # If data successfully sent over LoRa
            result = parse(response)
            if result is not None:
                req, mode, throttle, steering = result
                if mode == 0:
                    print("Auto Mode")
                else:
                    print("Manual Mode")
                print("throttle is", throttle)
                print("steering is", steering)
                if mode == 1:
                    manual()
                elif mode == 0:
                    auto()

                        
