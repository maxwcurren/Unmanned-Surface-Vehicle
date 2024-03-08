import serial
import re

gps = serial.Serial(port='COM9', baudrate=9600, timeout=1)  # Update the port for Raspberry Pi

def parse_gpgll(sentence):
    match = re.match(r'\$GPGLL,(\d+\.?\d*),([NS]),(\d+\.?\d*),([EW]),(\d+\.\d+),A.*', sentence)
    
    if match:
        latitude_degrees = float(match.group(1)[:2])
        latitude_minutes = float(match.group(1)[2:])
        if match.group(2) == 'N':
            #print(f"latitude_degrees north: {latitude_degrees}")
            #print(f"latitude_minutes: {latitude_minutes}")
            latitude = latitude_degrees + latitude_minutes / 60.0
        if match.group(2) == 'S':
            # If South, make latitude negative
            #print(f"South")
            latitude_degrees = -latitude_degrees
            #print(f"latitude_degrees south: {latitude_degrees}")
            #print(f"latitude_minutes: {latitude_minutes}")
            latitude = -(latitude_degrees + latitude_minutes / 60.0)
        
        #print(f"latitude: {latitude}")

        longitude_degrees = float(match.group(3)[:3])
        longitude_minutes = float(match.group(3)[3:])

        if match.group(4) == 'E':
            #print(f"longitude_degrees east: {longitude_degrees}")
            #print(f"longitude_minutes: {longitude_minutes}")
            longitude = longitude_degrees + longitude_minutes / 60.0
        if match.group(4) == 'W':
            # If West, make longitude negative
            longitude = -(longitude_degrees + longitude_minutes / 60.0)
            #print(f"longitude_degrees west: {longitude_degrees}")
            #print(f"longitude_minutes: {longitude_minutes}")
        
        #print(f"longitude: {longitude}")
        return latitude, longitude
    else:
        return None
        
def getGPS():
    latitude = 0
    longitude = 0

    while latitude == 0 and longitude == 0:
        # Read data from the GPS module
        data = gps.readline().decode('utf-8').strip()

        if data.startswith("$GPGLL"):
            # Parse the GPS data
            result = parse_gpgll(data)

            if result is not None:
                latitude, longitude = result

    return longitude, latitude

if __name__ == "__main__":
    while True:
        lon, lat = getGPS()
        print(f"lon: {lon}")
        print(f"lat: {lat}")
