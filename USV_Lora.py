#!/usr/bin/env python
# THIS IS THE LORA CODE FOR THE USV MODULE WRITTEN FOR THE REYAX RYLR896 Lora Module

import serial
import time
import struct
import re

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # set up serial output for Arduino
lora = serial.Serial('/dev/ttyS0', baudrate=115200, timeout=0)  # set up LoRa

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
            # You might want to add a delay before retransmitting to avoid overwhelming the channel
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

while True:
    # RECEIVE MANUAL CONTROLS OVER LORA
    time.sleep(0.1)
    response = receive_Lora()

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
                    # Send motor controls to Arduino
                    print("throttle is", throttle)
                    print("steering is", steering)
                    send_arduino(throttle, steering)
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
                        
                        # Wait for ACK before moving on
                        transmit_Lora(data)
                    else:
                        print("No Data request")
