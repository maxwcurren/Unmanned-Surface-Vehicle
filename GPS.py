import serial
import re

gps = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)

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

        return int(latitude* 100), int(longitude* 100)
    else:
        return None
        
def getGPS():
    latitude = 0
    longitude = 0

    while latitude == 0 and longitude == 0:
        print("Retrieving coordinates")
        # Read data from the GPS module
        data = gps.readline().decode('utf-8').strip()

        if data.startswith("$GPGLL"):
            # Parse the GPS data
            result = parse_gpgll(data)

            if result is not None:
                latitude, longitude = result

    return longitude, latitude

