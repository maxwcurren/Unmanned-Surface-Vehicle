from gpiozero import DistanceSensor

import time
import board
import digitalio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
#print(controller)
maxJs= 65536
tol=600
value=0
mode=1
throttle=511
steering=511
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

range = DistanceSensor(echo=17, trigger=4, max_distance=20)
while True:
        dist = (range.distance)
        #convert to meter, decimeter, centimeter
        Meter = int(dist)
        dm = (int(dist*10)-(Meter*10))
        cm = (int(dist*100))-((Meter*100)+(dm*10))
        M=str(Meter)
        DM=str(dm)
        CM=str(cm)
        # draw to sceen
        draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)
        draw.text((0, 0), "Distance: " + M + "." + DM + CM, font=font, fill=255)
        oled.image(image)
        oled.show()

        print(dist)
        