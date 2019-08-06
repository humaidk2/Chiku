from adafruit_servokit import ServoKit
import time
import RPi.GPIO as GPIO
import time
import numpy as np
from random import randrange
import random
from threading import Thread, Lock

cur_angle_mutex = Lock()
i2c_mutex = Lock()

GPIO.setmode(GPIO.BCM)

TRIG=18
ECHO=23

print ("Distance Measurement in Progress")

GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)
kit = ServoKit(channels = 16)



#kit.servo[0].angle = 40
#kit.servo[1].angle = 0
#kit.servo[2].angle = 100
#kit.servo[3].angle = 40
#kit.servo[4].angle = 0
#kit.servo[5].angle = 100
#kit.servo[6].angle = 160
#kit.servo[7].angle = 0
#kit.servo[8].angle = 120
#kit.servo[9].angle = 90
#kit.servo[10].angle = 0
#kit.servo[11].angle s= 120
startAngles = [70, 100, 140, 90, 80, 150, 90, 110, 140, 150, 130, 130]


def getDistance():
    GPIO.output(TRIG, False)

    print("Waiting for sensor to settle")

    # time.sleep(2)
    GPIO.output(TRIG,True)
    time.sleep(0.00001)
    GPIO.output(TRIG,False)

    while GPIO.input(ECHO)==0:
        pulse_start = time.time()
        
    while GPIO.input(ECHO)==1:
        pulse_end=time.time()
        
    pulse_duration = pulse_end - pulse_start

    distance = pulse_duration *17150
    distance = round(distance,2)

    print("Distance: ", distance, "cm")
    return distance
prevDistance = getDistance()
def getReward():
    global prevDistance
    currDistance = getDistance()
    reward = currDistance - prevDistance
    prevDistance = currDistance
    return reward - 1
vals = []
nums = [-0.5, 0, 0.5]
for i in nums:
    for j in nums:
        for k in nums:
            for l in nums:
                for m in nums:
                    for n in nums:
                        for o in nums:
                            for p in nums:
                                vals.append([i, j, 0, k, l, 0, m, n, 0, o, p, 0])

actions = []
action = [-0.5, 0, 0.5]
for i in action:
    for j in action:
        for k in action:
            for l in action:
                for m in action:
                    for n in action:
                        for o in action:
                            for p in action:
                                actions.append([i, j, startAngles[2], k, l, startAngles[5], m, n, startAngles[8], o, p, startAngles[11]])
                                if(i == 0 and j == 0 and k == 0 and l == 0 and m == 0 and n == 0 and o == 0 and p == 0):
                                    actionIndex = len(actions) - 1

numOfStates = len(vals)
numOfActions = len(actions)
print(numOfStates)
print(numOfActions)
print(getReward())
print(prevDistance)
#qmatrix = [[0.0] * numOfStates] * numOfActions
qmatrix = np.load("test.npy")
print("qmatrix loaded")
state = [0] * 12
stateIndex = 0

def setToInitialState():
    kit.servo[10].set_pulse_width_range(500, 3000)
    kit.servo[8].set_pulse_width_range(500, 3000)
    kit.servo[5].set_pulse_width_range(500, 3000)
    kit.servo[2].set_pulse_width_range(500, 3000)
    for angle in startAngles:
        i2c_mutex.acquire()
        kit.servo[11].angle = angle
        i2c_mutex.release()

def setState():
    for idx, elem in enumerate(vals):
        if(state[0] == elem[0] and state[1] == elem[1] and state[2] == elem[2] and state[3] == elem[3] and state[4] == elem[4] and state[5] == elem[5] and state[6] == elem[6]and state[7] == elem[7]and state[8] == elem[8] and state[9] == elem[9]and state[10] == elem[10] and state[11] == elem[11]):
            stateIndex = idx
            break
def performAction(actionIndex):
    for servoIndex, move in enumerate(actions[actionIndex]):
        state[servoIndex] += actions[actionIndex][servoIndex]
        if state[servoIndex] < -0.5:
            state[servoIndex] = -0.5
        elif state[servoIndex] > 0.5:
            state[servoIndex] = 0.5
        if((startAngles[servoIndex] + (state[servoIndex] * 70)) < 0):
            i2c_mutex.acquire()
            kit.servo[servoIndex].angle = 0
            i2c_mutex.release()
        elif((startAngles[servoIndex] + (state[servoIndex] * 70)) > 180):
            i2c_mutex.acquire()
            kit.servo[servoIndex].angle = 180
            i2c_mutex.release()
        else:
            i2c_mutex.acquire()
            kit.servo[servoIndex].angle = round(startAngles[servoIndex] + (state[servoIndex] * 70))
            i2c_mutex.release()
    setState()   
reward = 0
t = 0
actionValue = actionIndex
for j in range(15):
    stateIndex = 0
    setToInitialState()
    for i in range(300):
        if random.uniform(0, 1) > 0.3:
            qmax = -20000.0
            for q in range(numOfStates):
                if qmatrix[stateIndex][q] > qmax:
                    qmax = qmatrix[stateIndex][q]
                    actionValue = q
        else:
            actionValue = randrange(numOfStates)
        performAction(actionValue)
        reward = getReward()
        qmax = -20000.0
        for q in range(numOfStates):
            if qmatrix[actionValue][q] > qmax:
                qmax = qmatrix[actionValue][q]
        qmatrix[stateIndex][actionValue] =  (reward + (0.85 * qmax))
        stateIndex = actionValue
        print(j, " ", i)
    np.save("test", qmatrix)
    print("saved qmaatrix")
for j in range(300):
    stateIndex = 0
    setToInitialState()
    actionVal = actionIndex
    qmax = -20000.0
    for q in range(numOfStates):
        if qmatrix[stateIndex][q] > qmax:
            qmax = qmatrix[stateIndex][q]
            actionValue = q
    performAction(actionValue)
    reward = getReward()
    qmax = -20000.0
    for q in range(numOfStates):
        if qmatrix[actionValue][q] > qmax:
            qmax = qmatrix[actionValue][q]
    qmatrix[stateIndex][actionValue] =  qmatrix[stateIndex][actionValue] + 0.2 *((reward + (0.85 * qmax)) - qmatrix[stateIndex][actionValue])
    print(i)


