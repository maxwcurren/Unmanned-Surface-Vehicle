
from evdev import InputDevice, categorize, ecodes
import re
import serial
import struct
import time
import board
import digitalio
import math
import threading

from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306

import GUI

maxJs= 65536
value=0
mode=1
req=0
throttle=511
steering=511

controller = InputDevice( '/dev/input/event2') # set up input from game controller using evdev library to decode gamepad input
serial_port = '/dev/ttyS0'
test_data="this"
ser = serial.Serial(serial_port, baudrate=115200,timeout=1)


# Define the Reset Pin
oled_reset = digitalio.DigitalInOut(board.D4)

# Display Parameters
WIDTH = 128
HEIGHT = 64
BORDER = 5

# Display Refresh
LOOPTIME = 1.0

# Use for I2C.
i2c = board.I2C()
oled = adafruit_ssd1306.SSD1306_I2C(WIDTH, HEIGHT, i2c, addr=0x3C, reset=oled_reset)

# Clear display.
oled.fill(0)
oled.show()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
image = Image.new("1", (oled.width, oled.height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw a white background
draw.rectangle((0, 0, oled.width, oled.height), outline=255, fill=255)

#font = ImageFont.truetype('PixelOperator.ttf', 16)
font = ImageFont.load_default()
print("starting")

def transmit_Lora(data):
    ser.write(('AT+SEND=100,' + str(len(data)) + ',' + data + ',\r\n').encode('utf-8'))
    

def receive_Lora(max_retries=15):
    time.sleep(4)
    retry_count = 0

    while retry_count < max_retries:
        ser.write(('+RCV\r\n').encode('utf-8'))
        response = ser.readline().decode('utf-8').strip()

        if response:
            if 'ERR' in response or 'OK' in response:
                #print(f"Received error: {response}")
                pass
            else:
                #print(f"Received: {response}")
                error = 0
                return response, error
        retry_count += 1
        #print(f"Retrying... {retry_count}/{max_retries}") 
        time.sleep(0.1)

    #print(f"Exceeded maximum retries ({max_retries})")
    print("Error receiving sensor data")
    response = ""
    error = 1
    return response, error

def request():
    #write to request data from USV
    global req
    start_time = time.time()
    req=1
    print(f"Request is: {req}")
    test_data = '*' + str(req) + '&' + str(mode) + '^' + str(throttle) + '%' + str(steering) + '+\n' 
    print(f"Transmitted string: {test_data}")
    req=0
    transmit_Lora(test_data)
    #read mode
    time.sleep(1)
    info, error = receive_Lora()
    if error != 1:
        #print(f"Sensor Data: {info}")
        response = parse(info)
        if response is not None:
            req=0
            lon, lat, Mag = response
            print("Coordinates are: ", lon, lat) 
            print(f"Orientation is {Mag} degrees")
            draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)
            draw.text((0, 0), "In AUTO mode", font=font, fill=255)
            draw.text((0, 16), "Lat: " + str(float(lat)/10000), font=font, fill=255)
            draw.text((0, 32), "Long: " + str(float(lon)/10000), font=font, fill=255)
            draw.text((0, 48), "Mag: " + str(Mag), font=font, fill=255)
            oled.image(image)
            oled.show()
    print(f"req time: {time.time() - start_time}")
            
def sendWaypoints(lon, lat, ret, max_retries=1):
    retry_count = 0
    while retry_count < max_retries:
        for i in range(1,4):
            lon_data = '_' + '?'.join(map(str, map(lambda x: math.floor(x*10000), lon))) + '!' + str(ret)
            print(f"lon data: {lon_data}")
            transmit_Lora(lon_data)
            time.sleep(2)
        for i in range(1,4):        
            lat_data = '&' + '?'.join(map(str, map(lambda x: math.floor(x*10000), lat))) + '!' + str(ret)
            print(f"lat data: {lat_data}")
            transmit_Lora(lat_data)
            retry_count += 1
            time.sleep(2)
        
def parse(info):
    #print("info is: ", info)
    match = re.match(r'\+RCV=(\d+),(\d+),\*(-?\d+)\&(-?\d+)\^(\d+)\+,(-?\d+),(-?\d+)', info)

    if match:
        lon = match.group(3)
        lat = match.group(4)
        Mag = match.group(5)
        #print("lon: ", lon)
        #print("lat: ", lat)
        #print("mag: ", Mag)
        return lon, lat, Mag
    else:
        print("error in parse")
        return None
        
def dPrntA():
        draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)
        draw.text((0, 0), "In AUTO mode", font=font, fill=255)
        draw.text((0, 16), "Press B to Exit", font=font, fill=255)
        draw.text((0, 32), "Press A for Info", font=font, fill=255)
        oled.image(image)
        oled.show()
        
debounce_delay = 0.2  # Adjust this value based on your requirements

def Auto():
    global mode
    global Manual
    global req
    global controller
    global dPrntA

    dPrntA()   

    last_a_state = 0  # Initialize the last A button state (0 for up, 1 for down)
    last_a_press_time = 0  # Initialize the last A press time

    while mode == 0:   # make void function for auto mode
        for event in controller.read_loop():
            if event.type == ecodes.EV_KEY:
                if categorize(event).keycode[0] == "BTN_B":  # press B to exit
                    mode = 1
                    Manual()

                #if categorize(event).keycode[0] == "BTN_A":  # press A to get data
                #    current_time = time.time()
                #
                    # Check if the button state has changed to down
                #    if event.value == 1 and last_a_state == 0:
                #        last_a_state = 1
                #        last_a_press_time = current_time

                    # Check if the button state has changed to up and debounce
                #    elif event.value == 0 and last_a_state == 1:
                #        if (current_time - last_a_press_time) > debounce_delay:
                #            request()
                #        last_a_state = 0
                
                request()
            else:
                break
           
    
def Manual():
    global mode
    global req
    global steering
    global throttle

    axis = {
        ecodes.ABS_X: 'ls_x' ,   # 0-65,535 input
        ecodes.ABS_Y: 'ls_y' ,
        ecodes.ABS_RX: 'rs_x' ,
        ecodes.ABS_RY: 'rs_y' ,
        
        ecodes.ABS_RZ: 'rt',      #0-1023 trigger input
        ecodes.ABS_Z: 'lt',
        
        ecodes.ABS_HAT0X: 'dpad_x', #-1 0 1 input
        ecodes.ABS_HAT0Y: 'dpad_y'
    }

    center =  {
            'ls_x': maxJs/2,
            'ls_y': maxJs/2,
            'rs_x': maxJs/2,
            'rs_y': maxJs/2
        }

    last =  {
            'ls_x': maxJs/2,
            'ls_y': maxJs/2,
            'rs_x': maxJs/2,
            'rs_y': maxJs/2
        }

    val1=str(0)
    val2=str(0)
    draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)
    draw.text((0, 0), "In Manual Mode", font=font, fill=255)
    draw.text((0, 16), "Press Y to Exit", font=font, fill=255)
    oled.image(image)
    oled.show()
   
    
    
    while mode==1:
        current_time = time.time()
        #print("request is", request)
        for event in controller.read_loop() :
        
            
            if event.type == ecodes.EV_KEY:  
                
                if categorize(event).keycode[0] =="BTN_WEST":                  # press y button to exit manual
                    mode=0
                    throttle=511
                    steering=511
                    test_data = '*' + str(req) + '&' + str(mode) + '^' + str(throttle) + '%' + str(steering) + '+\n'        
                    transmit_Lora(test_data)
                    Auto()   
                    
            if event.type == ecodes.EV_ABS:
                if event.code in axis and axis[ event.code ] == 'dpad_y':       #read if throttle forward input
                    value = event.value 
                                                                                # make sent value between 0-->1024
                    if value==-1:
                        #throttle=651
                        throttle=585
                    elif value==1:
                        throttle=401
                    else:
                        throttle=511 
                    
                    if throttle % 2 == 0:                                       # make throttle always odd to distinguish in arduino
                            throttle = throttle + 1
                    
                    print("throttle is: " , throttle)
                    test_data = '*' + str(req) + '&' + str(mode) + '^' + str(throttle) + '%' + str(steering) + '+\n'        
                    transmit_Lora(test_data)
                    print(test_data)
                    #print("request is", request)
                    

                    
                         
                if event.code in axis and axis[ event.code ] == 'dpad_x':       #Read if steering input
                    
                    value = event.value
                    
                    if value==1:
                        steering=1022
                    elif value==-1:
                        steering=2
                    else:
                        steering=510
                    
                    if steering % 2 == 1:                                      # make steering always even to distinguish in arduino
                        steering = steering + 1
                   
                    print("steering is:" , steering)
                    test_data = '*' + str(req) + '&' + str(mode) + '^' + str(throttle) + '%' + str(steering) + '+\n'        
                    transmit_Lora(test_data)
                    print(test_data)
                    #print("request is", request)

def IRQ():
    global mode
    while True:
        if mode == 0:
            print("calling reqest")
            request()
            time.sleep(6)
        
app = GUI.start_GUI()
waypoint_lon, waypoint_lat = app.get_final_coordinates()
return_method = app.get_return_method()
print(f"Waypoint Lon: {waypoint_lon}")
print(f"Waypoint Lat: {waypoint_lat}")
print(f"Return Method: {return_method}")
draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)
draw.text((0, 0), "Sending Waypoints", font=font, fill=255)
draw.text((0, 16), "//////////" , font=font, fill=255)
draw.text((0, 32), "///////" , font=font, fill=255)
draw.text((0, 48), "///// "  , font=font, fill=255)
draw.text((0, 48), "// "  , font=font, fill=255)
oled.image(image)
oled.show()
sendWaypoints(waypoint_lon, waypoint_lat, return_method)

IRQ_thread = threading.Thread(target=IRQ)
IRQ_thread.start()

Manual()
