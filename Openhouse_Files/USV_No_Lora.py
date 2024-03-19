
from evdev import InputDevice, categorize, ecodes
import serial
import struct
import time
import board
import busio
import digitalio

from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306

import subprocess

controller = InputDevice( '/dev/input/event8') # set up input from game controller using evdev library to decode gamepad input
ser = serial.Serial('/dev/ttyACM0', 1200,timeout=1)      #  set up serial ouput for arduino
#print(controller)
maxJs= 65536

Mode=1

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





def Auto(Mode):
    
    while Mode==0:   # make void funtion for auto mode
        
        draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)
        draw.text((0, 0), "In AUTO Mode", font=font, fill=255)
        draw.text((0, 16), "Press B to Exit", font=font, fill=255)
        oled.image(image)
        oled.show()
        
        for event in controller.read_loop():
            if event.type == ecodes.EV_KEY:
                
                if categorize(event).keycode[0] =="BTN_B": # press b button to enter manual
                    Mode=1
                    time.sleep(1)
                    Manual(Mode)


def Manual(Mode):
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
    
    while Mode==1:
        
        for event in controller.read_loop() :
            
            if event.type == ecodes.EV_KEY:  
                
                if categorize(event).keycode[0] =="BTN_WEST": # press y button to exit manual
                    print("button")
                    Mode=0
                    print(Mode)
                    time.sleep(1)
                    Auto(Mode)
                
            if event.type == ecodes.EV_ABS:
                #print("Input detected")
                if event.code in axis and axis[ event.code ] == 'dpad_y':      #read if throttle forward input
                    value = event.value 
                    print(value)
                                                  # make sent value between 0-->1024
                    #
                    if value==-1:
                        throttle=651
                    elif value==1:
                        throttle=401
                    else:
                        throttle=511 
                    #
                    
                    if throttle % 2 == 0:                                     # make throttle always odd to distinguish in arduino
                            throttle = throttle + 1
                    
                    print("throttle is: " , throttle)
                    ser.write(struct.pack('<h',throttle))                    #send to arduino
                    ser.flush()
                    
                         
                if event.code in axis and axis[ event.code ] == 'dpad_x':       #Read if steering input
                    
                    value = event.value
                    
                    if value==1:
                        steering=1022
                    elif value==-1:
                        steering=2
                    else:
                        steering=510
                    #
                    if steering % 2 == 1:                                     # make steering always even to distinguish in arduino
                        steering = steering + 1
                   
                    print("steering is:" , steering)
                    ser.write(struct.pack('<h',steering))
                    ser.flush()
                                  
             #read input sent to arduino for testing
#                     responce_value = None
#                     
#                     responce=ser.read(2)
#                     ser.flush()
#                     if responce:
#                          responce_value = struct.unpack('<h',responce)
#                          print("sent this value ", responce_value)
#                     val2=str(steering)
#                     draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)
#                     draw.text((0, 0), "In Manual Mode", font=font, fill=255)
#                     draw.text((0, 16), "Press Y to Exit", font=font, fill=255)
#                     draw.text((0, 32), "Throttle: " + val1, font=font, fill=255)
#                     draw.text((0, 48), "Steering: " + val2, font=font, fill=255)
#                     oled.image(image)
#                     oled.show()
#                     

Manual(Mode)                        #always starts manual 
