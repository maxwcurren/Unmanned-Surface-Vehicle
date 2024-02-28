#!/usr/bin/env python

import serial
import time
serial_port = '/dev/ttyS0'

lora = serial.Serial(serial_port, baudrate=115200, timeout=1)

#TEST COMMAND
lora.write(('AT\r\n').encode('utf-8'))
print(f'Response: ' + (lora.readline().decode('utf-8').strip()))

while True:
	lora.write(('+RCV\r\n').encode('utf-8'))
	time.sleep(0.2)
	response = lora.readline().decode('utf-8').strip()
	print(f'Response: {response}')
	if 'ERR' not in response:
		print(f'Response: {response}')

