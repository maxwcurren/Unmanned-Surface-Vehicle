import pigpio
import time

# Define GPIO pins for the sensors
trigger1, echo1 = 17, 18
trigger2, echo2 = 22, 27
trigger3, echo3 = 23, 24
trigger4, echo4 = 8, 25

def detObj():
    global trigger1, echo1, trigger2, echo2, trigger3, echo3, trigger4, echo4
    # Initialize pigpio
    pi = pigpio.pi()

    # Set trigger pins as output and echo pins as input for all sensors
    pi.set_mode(trigger1, pigpio.OUTPUT)
    pi.set_mode(echo1, pigpio.INPUT)
    pi.set_mode(trigger2, pigpio.OUTPUT)
    pi.set_mode(echo2, pigpio.INPUT)
    pi.set_mode(trigger3, pigpio.OUTPUT)
    pi.set_mode(echo3, pigpio.INPUT)
    pi.set_mode(trigger4, pigpio.OUTPUT)
    pi.set_mode(echo4, pigpio.INPUT)

    try:
        # Send a short pulse to trigger the first sensor
        pi.write(trigger1, 1)
        time.sleep(0.00001)
        pi.write(trigger1, 0)

        # Wait for the echo pin of the first sensor to go high
        while pi.read(echo1) == 0:
            pulse_start1 = time.time()

        # Wait for the echo pin of the first sensor to go low again
        while pi.read(echo1) == 1:
            pulse_end1 = time.time()

        # Calculate the pulse duration for the first sensor (in seconds)
        pulse_duration1 = pulse_end1 - pulse_start1

        # Calculate the distance for the first sensor (in cm)
        distance_meters1 = pulse_duration1 * 34300 / 2  # Speed of sound is approximately 343 m/s

        # Convert distance to inches for the first sensor
        distance_inches1 = distance_meters1 * 0.3937008

        # Send a short pulse to trigger the second sensor
        pi.write(trigger2, 1)
        time.sleep(0.00001)
        pi.write(trigger2, 0)

        # Wait for the echo pin of the second sensor to go high
        while pi.read(echo2) == 0:
            pulse_start2 = time.time()

        # Wait for the echo pin of the second sensor to go low again
        while pi.read(echo2) == 1:
            pulse_end2 = time.time()

        # Calculate the pulse duration for the second sensor (in seconds)
        pulse_duration2 = pulse_end2 - pulse_start2

        # Calculate the distance for the second sensor (in cm)
        distance_meters2 = pulse_duration2 * 34300 / 2  # Speed of sound is approximately 343 m/s

        # Convert distance to inches for the second sensor
        distance_inches2 = distance_meters2 * 0.3937008

        # Send a short pulse to trigger the third sensor
        pi.write(trigger3, 1)
        time.sleep(0.00001)
        pi.write(trigger3, 0)

        # Wait for the echo pin of the third sensor to go high
        while pi.read(echo3) == 0:
            pulse_start3 = time.time()

        # Wait for the echo pin of the third sensor to go low again
        while pi.read(echo3) == 1:
            pulse_end3 = time.time()

        # Calculate the pulse duration for the third sensor (in cm)
        pulse_duration3 = pulse_end3 - pulse_start3

        # Calculate the distance for the third sensor (in meters)
        distance_meters3 = pulse_duration3 * 34300 / 2  # Speed of sound is approximately 343 m/s

        # Convert distance to inches for the third sensor
        distance_inches3 = distance_meters3 * 0.3937008

        # Send a short pulse to trigger the fourth sensor
        pi.write(trigger4, 1)
        time.sleep(0.00001)
        pi.write(trigger4, 0)

        # Wait for the echo pin of the fourth sensor to go high
        while pi.read(echo4) == 0:
            pulse_start4 = time.time()

        # Wait for the echo pin of the fourth sensor to go low again
        while pi.read(echo4) == 1:
            pulse_end4 = time.time()

        # Calculate the pulse duration for the fourth sensor (in seconds)
        pulse_duration4 = pulse_end4 - pulse_start4

        # Calculate the distance for the fourth sensor (in cm)
        distance_meters4 = pulse_duration4 * 34300 / 2  # Speed of sound is approximately 343 m/s

        # Convert distance to inches for the fourth sensor
        distance_inches4 = distance_meters4 * 0.3937008

        # Clean up GPIO
        pi.stop()
        
        obj_array = filter_values(distance_inches1, distance_inches2, distance_inches3, distance_inches4)
        #return distance_inches1, distance_inches2, distance_inches3, distance_inches4
        return obj_array, distance_inches1, distance_inches2, distance_inches3, distance_inches4

    except KeyboardInterrupt:
        print("Stopping the sensor reading.")
        pi.stop()

def filter_values(left, front, right, bottom):
    object_arr = [0,0,0,0]
    # If object over 2 meters away, ignore.
    object_arr[0] = 0 if left > 78.7402 else 1
    object_arr[1] = 0 if front > 78.7402 else 1
    object_arr[2] = 0 if right > 78.7402 else 1
    object_arr[3] = 0 if bottom > 30 else 1
    return object_arr

# Call the function to read data from all four sensors
if __name__ == "__main__":
    while True:
        obj, l, f, r, b = detObj()
        print(f"left: {l}, front: {f}, right: {r}, bottom: {b}")
        print(f"obj array: {obj}")
        time.sleep(1)


