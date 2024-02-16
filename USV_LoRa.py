#!/usr/bin/env python

# THIS IS THE LORA CODE FOR THE USV MODULE WRITTEN FOR THE REYAX RYLR896 Lora Module

import serial
import time
import struct  # Import the struct module
import re

serial_port = '/dev/ttyS0'
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # set up serial output for Arduino

lora = serial.Serial(serial_port, baudrate=115200, timeout=0)

# TEST COMMAND
lora.write(('AT\r\n').encode('utf-8'))
print(f'Response: ' + (lora.readline().decode('utf-8').strip()))

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

def transmit(data):
    lora.write(('AT+SEND=99,' + str(len(data)) + ',' + data + ',\r\n').encode('utf-8'))
    
def receive():
    lora.write(('+RCV\r\n').encode('utf-8'))
    response = lora.readline().decode('utf-8').strip()
    return response

mode = 0
req = 0
throttle = 511
steering = 511

while True:
    # RECEIVE MANUAL CONTROLS OVER LORA
    
    time.sleep(0.1)
    response = receive()

    # Filter response
    if 'ERR' not in response:
        if '%' in response and '^' in response and '&' in response and '+' in response and '*' in response:
            # If data successfully sent over LoRa
            print(response)              
            result = parse(response)
            if result is not None:
                req, mode, throttle, steering = result
                print("mode is", mode)
                if mode == 1:
                    # IN MANUAL MODE
                    # Send motor controls to arduino
                    print("throttle is", throttle)
                    print("steering is", steering)
                    ser.write(struct.pack('<h', int(throttle)))
                    ser.flush()
                    ser.write(struct.pack('<h', int(steering)))
                    ser.flush()
                elif mode == 0: 
                    # IN AUTO MODE
                    # Send data if requested
                    if req == 1:
                        print("Data Request")
                        # Grab sensor values
                        # use dummy values:
                        longitude = "524"
                        latitude = "-12"
                        orientation = "N"
                        data = '*' + longitude + '&' + latitude + '^' + orientation + '+'
                        time.sleep(0.1)
                        transmit(data)
                        print("Sent: ",data)
                    else:
                        print("No Data request")
                        
