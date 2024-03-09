import time
from Accel_Mag import QMC5883L, MPU6050
import GPS
import serial
import Lidar
import math
import struct
import re
import threading
import Ultrasonic
from adafruit_rplidar import RPLidar

QMC = QMC5883L()
mpu6050 = MPU6050()
ard_port = '/dev/ttyACM0'
ser = serial.Serial(ard_port, 1200, timeout=1)  # set up serial output for Arduino
lora = serial.Serial('/dev/ttyS0', baudrate=115200, timeout=0)  # set up LoRa

# Get starting position of USV
starting_lon, starting_lat = GPS.getGPS()
print(f"Starting Longitude: {starting_lon}, Starting Latitude: {starting_lat}")

# return_method will determine how the USV returns to the base station. It will be determined on creation of waypoint arrays.
# If return_method is zero then the USV will return to the starting position by iterating through the waypoints in reverse.
# If return_method is one then the USV will directly calculate the heading towards the starting position and stop when it arrives.

# Define globals
waypoint_count = 0
mode = 1
req = 0
throttle = 511
steering = 512
earth_radius = 6371000  # Earth's radius in meters

# Global variables for shared data
distances = []
angles = []
scan_lock = threading.Lock()
distances_prev = []
angles_prev = []

# Define PID variables:
Kp_yaw, Kd_yaw = 2, 1.88
prev_error = 0.0
cumulative_error = 0.0

def lidar_scan_thread(duration):
    while True:
        try:
            global distances, angles
            distances, angles = Lidar.scan_lidar(duration)
            #print(f"distances: {distances}")
            #print(f"angles: {angles}")
        except Exception as e:
            print(f"Lidar thread encountered an exception: {e}")
            
def ultrasonic_scan_thread():
    while True:
        # Check for objects with ultrasonics
        try:
            global ultrasonic_obstacles
            ultrasonic_obstacles, _, _, _, _ = Ultrasonic.detObj()
        except:
            #print(f"error retrieving ultrasonic data")
            ultrasonic_obstacles = [0,0,0,0]
        #print(f"ultrasonics: {ultrasonic_obstacles}")
        time.sleep(3)

# PATHING FUNCTIONS
def detectObject(current_heading, distances, angles, ultrasonic_ave):
    #print(f"Current Heading in detect Object: {current_heading}")
    
    # WITH ULTRASONICS
    #if len(angles) != 0 or ultrasonic_ave[1]:
    #    # Lidar or ultrasonics detected an object
    #    if any(330 <= angle <= 360 or 0 <= angle <= 30 for angle in angles) or ultrasonic_ave[1]:
    #        # If there is an object detected in the current path of the USV
    #        object_detected = 1
    #        print("Object in front of USV")
    #    else:
    #        object_detected = 0
    
    # WITHOUT ULTRASONICS
    if len(angles) != 0:
        # Lidar or ultrasonics detected an object
        if any(330 <= angle <= 360 or 0 <= angle <= 30 for angle in angles):
            # If there is an object detected in the current path of the USV
            object_detected = 1
            print("Object in front of USV")
        else:
            object_detected = 0
    else:
        object_detected = 0
                
    if ultrasonic_ave[0]:
        #print("Object to the left of USV")
        pass
    if ultrasonic_ave[2]:
        #print("Object to the right of USV")
        pass
    if ultrasonic_ave[3]:
        # Depth ultrasonic detects too shallow
        #print("Water too shallow")
        #print("Not yet implemented")
        pass
        
    #print(f"Object Detected: {object_detected}")
    return object_detected

def deg2rad(deg):
    return deg * (math.pi / 180)

def getNextHeading(current_heading, distances, angles, waypoint_lon, waypoint_lat, current_lon, current_lat, return_method, ultrasonic_ave):
    global waypoint_count, waypoint_num, starting_lon, starting_lat, earth_radius
    # Calculate distance to waypoint
    #print("getting next heading")

    # Waypoint and current coordinates
    waypoint_lon = deg2rad(waypoint_lon)
    waypoint_lat = deg2rad(waypoint_lat)
    current_lon = deg2rad(current_lon)
    current_lat = deg2rad(current_lat)

    # Differences in latitude and longitude in radians
    delta_lon = waypoint_lon - current_lon
    delta_lat = waypoint_lat - current_lat

    # Haversine formula to calculate distance
    a = math.sin(delta_lat / 2) * math.sin(delta_lat / 2) + math.cos(current_lat) * math.cos(waypoint_lat) * math.sin(delta_lon / 2) * math.sin(delta_lon / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance_to_waypoint = earth_radius * c
    print(f"distance to waypoint: {distance_to_waypoint}")

    if distance_to_waypoint < 10.0:
        waypoint_count += 1

    if waypoint_count <= waypoint_num:
        object_detected = detectObject(current_heading, distances, angles, ultrasonic_ave)
                
        # OBJECT DETECTION
        if object_detected == 1:
            new_heading = (current_heading + 120) % 360
            print(f"Desired heading: {new_heading}")
            return map_to_servo(current_heading, new_heading)

        # PID
        elif object_detected == 0:
            new_heading = pid_controller(current_heading, waypoint_lon, waypoint_lat, current_lon, current_lat)
            return new_heading

    # Check if it's time to return
    elif waypoint_count == waypoint_num:
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
            deta_starting_lon = starting_lon - current_lon
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
    
    # Calculate the PD output
    pid_output = proportional + derivative
    # Update previous error
    prev_error = error
    
    # Calculate next heading
    next_heading = current_heading + pid_output
    # Normalize the next_heading to the range [0, 360)
    next_heading = (next_heading + 360) % 360
    print(f"Desired heading: {next_heading}")
    # Calculate the proportional servo value based on the PID output
    proportional_servo = pid_output * (1022 - 2) / 360
    
    #print(f"Desired heading: {pid_output}")
    
    # Calculate the final servo value
    final_servo_value = int(round(510 + proportional_servo))
    final_servo_value -= final_servo_value % 2  # Adjust to the nearest even number

    # Ensure the servo value stays within the valid range
    #print(f"Mapped Steering: {final_servo_value}")
    return max(min(final_servo_value, 1022), 2)

def map_to_servo(current_heading, desired_heading):
    # Define the range of servo values and the middle value
    servo_min = 2
    servo_max = 1022
    servo_middle = 510  # Middle value
    
    # Calculate the difference between the current heading and the desired heading
    heading_difference = abs(current_heading - desired_heading)
    
    # Calculate a factor to adjust the proportional servo value
    adjustment_factor = heading_difference / 180  # Normalize to the range [0, 1]
    
    # Calculate the proportional servo value based on the desired heading
    proportional_servo = (desired_heading / 360) * (servo_max - servo_min)
    
    # Adjust the proportional servo value based on the adjustment factor
    adjusted_proportional_servo = servo_middle + (proportional_servo - servo_middle) * adjustment_factor
    
    # Round to the nearest even integer
    final_servo_value = int(round(adjusted_proportional_servo))
    final_servo_value -= final_servo_value % 2  # Ensure even number
    
    # Ensure the servo value stays within the valid range
    return max(min(final_servo_value, servo_max), servo_min)

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
            throttle = 511
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
            #print(f"Received acknowledgment: {ack_response}")
            #print("Transmission successful.")
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
        #print(response)
        #print(f"lon: {lon}")
        #print(f"lat: {lat}")
        #print(f"ret: {ret}")
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
    


def send_arduino(throttle, steering, timeout=3):
    try:
        #print("Sending throttle")
        ser.write(struct.pack('<h', int(throttle)))
        time.sleep(0.1)  # Add a small delay
        #print("Sending steering")
        ser.write(struct.pack('<h', int(steering)))
        time.sleep(0.1)  # Add another small delay
        #print("Finish sending motor controls")
    except Exception as e:
        print(f"Error writing to serial port: {e}")


def is_serial_port_available(port_name):
    try:
        ser = serial.Serial(port_name)
        ser.close()
        return True
    except serial.SerialException:
        return False

def manual():
    global throttle, steering
    # IN MANUAL MODE
    print("In Manual")
    print(f"throttle: {throttle}")
    print(f"steering: {steering}")
    time.sleep(0.5)
    send_arduino(throttle, steering)
def auto():
    global throttle, steering, req, distances, distances_prev, angles, angles_prev
    #t1 = time.time()
    # IN AUTO MODE
    #print("In Auto")
    throttle = 611
    #throttle = 511
    # Read GPS Module for coordinates
    gps_lon, gps_lat = GPS.getGPS()
    #print(f"gps_lon: {gps_lon}")
    #print(f"gps_lat: {gps_lat}")
    gps_lon1 = gps_lon * 10000
    gps_lat1 = gps_lat * 10000
    # Read magnetometer
    current_yaw = QMC.get_bearing()
    print(f"current_yaw: {current_yaw}")
    
    # Get target yaw using PID or object detected function
    #print(f"waypoint_lon[waypoint_count] : {waypoint_lon[waypoint_count]}")
    #print(f"waypoint_lat[waypoint_count] : {waypoint_lat[waypoint_count]}")
    target_yaw = getNextHeading(current_yaw, distances, angles, waypoint_lon[waypoint_count], waypoint_lat[waypoint_count], gps_lon, gps_lat, return_method, ultrasonic_obstacles)
    #print(f"Taget Heading: {target_yaw}")
    steering = target_yaw
    
    time.sleep(0.5)
    # Send motor controls to Arduino
    send_arduino(throttle, steering)

    # Send data if requested
    if req == 1:
        #print("Data Request")
        data = '*' + str(math.floor(gps_lon1)) + '&' + str(math.floor(gps_lat1)) + '^' + str(math.floor(current_yaw)) + '+'            
        # Wait for ACK before moving on
        print(f"Sent: {data}")
        transmit_Lora(data)
        req = 0

# Get waypoints:
#waypoint_lon, waypoint_lat, return_method = receive_Way_Ret()
#print("Received Waypoints")
#waypoint_num = len(waypoint_lon)

# Lake Perris Test Waypoints
#waypoint_lon = [-117.195808167, -117.19659367]
#waypoint_lat = [33.86186967, 33.8615805]
#return_method = 0
#waypoint_num = len(waypoint_lon)

# Home Test Waypoints:
waypoint_lon = [-117.47080895979879]
waypoint_lat = [33.8885588260905]
return_method = 0
waypoint_num = len(waypoint_lon)

scan_duration = 2  # Set the duration in seconds
lidar_thread = threading.Thread(target=lidar_scan_thread, args=(scan_duration,))
lidar_thread.start()

ultrasonic_thread = threading.Thread(target=ultrasonic_scan_thread)
ultrasonic_thread.start()

while True:
    # RECEIVE MANUAL CONTROLS OVER LORA
    #time.sleep(0.1)
    response = receive_Lora()
    # Filter response
    #print(f"response: {response}")
    #print("in while loop")
    #print(f"waypoint_count: {waypoint_count}")
    if 'ERR' not in response:
        #print(f"response: {response}")
        if '%' in response and '^' in response and '&' in response and '+' in response and '*' in response:
            # If data successfully sent over LoRa
            result = parse(response)
            if result is not None:
                req, mode, throttle, steering = result
        if mode == 1:
            #print("going to manual")
            manual()
        elif mode == 0:
            #print("going to auto")
            auto()
