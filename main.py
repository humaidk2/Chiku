# Importing required libraries
from adafruit_servokit import ServoKit # library for running servos
import time # library for delays
import RPi.GPIO as GPIO # library for input/output on pins(ultrasonic)
import numpy as np # numpy library for saving/loading arrays
from random import randrange # library for getting random number
import random # library for getting random number
from threading import Thread, Lock # library for locking threads for servos

cur_angle_mutex = Lock()
i2c_mutex = Lock()

# initialize input mode to BCM 
GPIO.setmode(GPIO.BCM)

# select trig/echo pins for ultrasonic
TRIG=18
ECHO=23

print ("Distance Measurement in Progress")
# setup ultrasonic pins
GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)
# initialize motors
kit = ServoKit(channels = 16)


# select intiial angles for robot
startAngles = [70, 100, 140, 90, 80, 150, 90, 110, 140, 150, 130, 130]

# function to get distance using ultrasonic sensor
def getDistance():

    GPIO.output(TRIG, False)

    print("Waiting for sensor to settle")
    # start ultrasonic
    GPIO.output(TRIG,True)
    time.sleep(0.00001)
    GPIO.output(TRIG,False)
    # trhow a signal
    while GPIO.input(ECHO)==0:
        pulse_start = time.time()
    # wait for return of signal
    while GPIO.input(ECHO)==1:
        pulse_end=time.time()
    # measure duration  
    pulse_duration = pulse_end - pulse_start

    # scale distance to cm
    distance = pulse_duration *17150
    # round to two decimal places
    distance = round(distance,2)

    print("Distance: ", distance, "cm")
    # return distance
    return distance
prevDistance = getDistance()
# calculate reward using getDistance method
def getReward():
    # connect scopes to global prevDistance
    global prevDistance
    # get current distance from ultrasonic
    currDistance = getDistance()
    # calculate change in distance
    reward = currDistance - prevDistance
    # update distance
    prevDistance = currDistance
    # return change in distance - 1 as reward
    return reward - 1
# create initial q matrix
# used to store all combinations of states
vals = []
# possible states
nums = [-0.5, 0, 0.5]
# create combinations for 8 servos with each having 3 states
for i in nums:
    for j in nums:
        for k in nums:
            for l in nums:
                for m in nums:
                    for n in nums:
                        for o in nums:
                            for p in nums:
                                # add combination set 4 motors to state 0
                                vals.append([i, j, 0, k, l, 0, m, n, 0, o, p, 0])
# I've kept this separate in case i need to add more actions then states
# used to store all combinations of actions
actions = []
# possible actions
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
# get num of states
numOfStates = len(vals)
# get num of actions
numOfActions = len(actions)
# print current statistics
print(numOfStates)
print(numOfActions)
print(getReward())
print(prevDistance)
# create new qmatrix
# numOfstates represents how many rows
# num Of Actions represents how many columns
#qmatrix = [[0.0] * numOfStates] * numOfActions
# load qmatrix if test file is available
qmatrix = np.load("test.npy")
print("qmatrix loaded")
# initialize current state with all zeroes
state = [0] * 12
# initialize current state to state 0
stateIndex = 0
# intial sstate
def setToInitialState():
    # change ranges of pwm signal for 4 motors
    kit.servo[10].set_pulse_width_range(500, 3000)
    kit.servo[8].set_pulse_width_range(500, 3000)
    kit.servo[5].set_pulse_width_range(500, 3000)
    kit.servo[2].set_pulse_width_range(500, 3000)
    # set all motors to initial motor angle
    for angle in startAngles:
        i2c_mutex.acquire()
        kit.servo[11].angle = angle
        i2c_mutex.release()

# used to update stateIndex
# def setState():
#   # find index of current state   
#     for idx, elem in enumerate(vals):
#         if(state[0] == elem[0] and state[1] == elem[1] and state[2] == elem[2] and state[3] == elem[3] and state[4] == elem[4] and state[5] == elem[5] and state[6] == elem[6]and state[7] == elem[7]and state[8] == elem[8] and state[9] == elem[9]and state[10] == elem[10] and state[11] == elem[11]):
#             # update state index
#             nextState = idx
#             break

# perform action
def performAction(actionIndex):
    # update state for every val in state and action pair
    for servoIndex, move in enumerate(actions[actionIndex]):
        # update state based on action
        state[servoIndex] += actions[actionIndex][servoIndex]
        # make sure state only changes within a range of -0.5 to 0.5
        if state[servoIndex] < -0.5:
            state[servoIndex] = -0.5
        elif state[servoIndex] > 0.5:
            state[servoIndex] = 0.5
        # change the angle by 70 * 0.5 = 35 degrees
        # update servo angles based on state change
        # make sure angles of servos are between 0 and 180
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
    # update state
    #setState() 

# initialize reward  
reward = 0
t = 0
# initialize action
actionValue = actionIndex
# training qmatrix
# for 15 epochs
for j in range(15):
    # reinitialize state every epoch
    stateIndex = 0
    # initialize state of motors every epoch
    setToInitialState()
    # 300 iterations per epoch
    for i in range(300):
        # using episolon greedy method, select random action or best action(highest q value)
        if random.uniform(0, 1) > 0.3:
            # select highest q value from current state
            # set an initial max q
            qmax = -20000.0
            # update qmax value if its greater than max q value to get qmax for current state
            for q in range(numOfStates):
                if qmatrix[stateIndex][q] > qmax:
                    qmax = qmatrix[stateIndex][q]
                    actionValue = q
        else:
            # select random action
            actionValue = randrange(numOfStates)
        # perform selected action
        performAction(actionValue)
        # get reward for selected action
        reward = getReward()
        # find best q value for next state
        # set low initial qmax val
        qmax = -20000.0
        # update qmax if its greater than qmax in next state
        for q in range(numOfStates):
            if qmatrix[actionValue][q] > qmax:
                qmax = qmatrix[actionValue][q]
        # update q value for current state using qmax of next state
        qmatrix[stateIndex][actionValue] =  qmatrix[stateIndex][actionValue] + 0.2 *((reward + (0.85 * qmax)) - qmatrix[stateIndex][actionValue])
        # update current state to next state
        stateIndex = actionValue
        print(j, " ", i)
    # save q value every epoch
    np.save("test", qmatrix)
    print("saved qmaatrix")
    
# only for testing
# # only selects best q value for every state
# # intialize state
# stateIndex = 0
# setToInitialState()
# actionVal = actionIndex
# # for 300 moves
# for j in range(300):
# # select action with best q value
#     qmax = -20000.0
#     for q in range(numOfStates):
#         if qmatrix[stateIndex][q] > qmax:
#             qmax = qmatrix[stateIndex][q]
#             actionValue = q
#     performAction(actionValue)
#     stateIndex = actionValue



