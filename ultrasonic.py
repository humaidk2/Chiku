# import libraries
import RPi.GPIO as GPIO
import time
# set mode to BCM
GPIO.setmode(GPIO.BCM)
# select pins for ultrasonic
TRIG=18
ECHO=23

print ("Distance Measurement in Progress")
# set input/output pins to trig/echo
GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)
# get distance using ultrasonic
def getDistance():
    # set trig to off
    GPIO.output(TRIG, False)

    print("Waiting for sensor to settle")
    # turn on ultrasonic sensor
    # time.sleep(2)
    GPIO.output(TRIG,True)
    time.sleep(0.00001)
    GPIO.output(TRIG,False)
    # start ultrasonic
    while GPIO.input(ECHO)==0:
        pulse_start = time.time()
    # stop ultrasonic if signal is detected
    while GPIO.input(ECHO)==1:
        pulse_end=time.time()
    # measure length pulse by duration of signal  
    pulse_duration = pulse_end - pulse_start
    # scale distance to cm
    distance = pulse_duration *17150
    # round distance to 2 decimal places
    distance = round(distance,2)
    # print distance
    print("Distance: ", distance, "cm")
    # return distance
    return distance
prevDistance = getDistance()
