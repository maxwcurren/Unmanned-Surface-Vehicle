import pigpio
import time
import numpy as np

def DetectAve():
    AvgObstacleInterference = [0, 0, 0, 0]
    
    sensor1_data, sensor2_data, sensor3_data, sensor4_data = read_JSR_sensor_data(trigger1, echo1, trigger2, echo2, trigger3, echo3, trigger4, echo4)
    
    ObstacleArr_data = np.array(check_obstacle(sensor1_data, sensor2_data, sensor3_data, sensor4_data))
    time.sleep(0.75)
    
    ObstacleArr_data2 = np.array(check_obstacle(sensor1_data, sensor2_data, sensor3_data, sensor4_data))
    time.sleep(0.75)
    
    ObstacleArr_data3 = np.array(check_obstacle(sensor1_data, sensor2_data, sensor3_data, sensor4_data))
    time.sleep(0.75)
    
    sum_ObstacleArr =  ObstacleArr_data + ObstacleArr_data2 + ObstacleArr_data3
    
    for i in range(4):
        AvgObstacleInterference[i] = sum_ObstacleArr[i] // 3
    return AvgObstacleInterference

def check_obstacle(sensor1_data, sensor2_data, sensor3_data, sensor4_data):
    ObstacleVal = []
    if 35 <= sensor1_data <= 90:
        ObstacleVal.append(1)
    else:
        ObstacleVal.append(0)
    if 35 <= sensor2_data <= 90:
        ObstacleVal.append(1)
    else:
        ObstacleVal.append(0)
    if 35 <= sensor3_data <= 90:
        ObstacleVal.append(1)
    else:
        ObstacleVal.append(0)
    if 10 <= sensor4_data <= 35:
        ObstacleVal.append(1)
    else:
        ObstacleVal.append(0)
    return ObstacleVal

def read_JSR_sensor_data(trigger1, echo1, trigger2, echo2, trigger3, echo3, trigger4, echo4):
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

        return distance_inches1, distance_inches2, distance_inches3, distance_inches4

    except KeyboardInterrupt:
        print("Stopping the sensor reading.")
        pi.stop()


# Define GPIO pins for the sensors
trigger1, echo1 = 17, 18
trigger2, echo2 = 22, 27
trigger3, echo3 = 23, 24
trigger4, echo4 = 8, 25

# Call the function to read data from all four sensors
if __name__ == "__main__":
    while True:
        sensor1_data, sensor2_data, sensor3_data, sensor4_data = read_JSR_sensor_data(trigger1, echo1, trigger2, echo2, trigger3, echo3, trigger4, echo4)
        #ObstacleArr_data = np.array(check_obstacle(sensor1_data, sensor2_data, sensor3_data, sensor4_data))
        OvertimeObstacleInterference_data = DetectAve()
        print(sensor1_data, sensor2_data, sensor3_data, sensor4_data)
        #print("Obstacle Check:", ObstacleArr_data)
        print("AvgScan: ", OvertimeObstacleInterference_data)

