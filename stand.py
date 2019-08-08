#import libraries
from adafruit_servokit import ServoKit

# intialize servokit
kit = ServoKit(channels = 16)

# change ranges for 4 motors to get 0-180 for all motors
kit.servo[10].set_pulse_width_range(500, 3000)
kit.servo[8].set_pulse_width_range(500, 3000)
kit.servo[5].set_pulse_width_range(500, 3000)
kit.servo[2].set_pulse_width_range(500, 3000)
# set all motors to standing position
kit.servo[0].angle = 70 # bottom right
kit.servo[1].angle = 100
kit.servo[2].angle = 140
kit.servo[3].angle = 90 # top right
kit.servo[4].angle = 80
kit.servo[5].angle = 150
kit.servo[6].angle = 90 # top left
kit.servo[7].angle = 110
kit.servo[8].angle = 140
kit.servo[9].angle = 150 # bottom left
kit.servo[10].angle = 130
kit.servo[11].angle = 130

