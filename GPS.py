import serial
import re

def parse_gpgll(sentence):
    match = re.match(r'\$GPGLL,(\d+\.\d+),([NS]),(\d+\.\d+),([EW]),.*', sentence)
    
    if match:
        latitude = float(match.group(1))
        if match.group(2) == 'S':
            # If South, make latitude negative
            latitude = -latitude  

        longitude = float(match.group(3))
        if match.group(4) == 'W':
            # If West, make longitude negative
            longitude = -longitude  

        return latitude/100, longitude/100
    else:
        return None

try:
    ser = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)
    
    while True:
        data = ser.readline().decode('utf-8').strip()
        if data.startswith("$GPGLL"):
            result = parse_gpgll(data)
            if result is not None:
                latitude, longitude = result
                print(f"Latitude: {latitude}, Longitude: {longitude}")

except Exception as e:
    print(f"Error: {e}")
finally:
    ser.close()
