#!/usr/bin/env python

import serial
import time
serial_port = '/dev/ttyS0'

ser = serial.Serial(serial_port, baudrate=115200, timeout=0.5)

test_data = 'TEST DATA'

while True:
	start_time = time.time()
	ser.write(('AT+SEND=99,'+ str(len(test_data)) + ',' +test_data + ',\r\n').encode('utf-8'))
	
	response = ser.readline().decode('utf-8').strip()
	print(f'Response: {response}')
