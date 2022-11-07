from controller import Robot, DistanceSensor, Motor
import numpy as np

"""

The answer and my code seems to do the same thing...



"""

# time in [ms] of a simulation step
TIME_STEP = 64
MAX_SPEED = 6.28
MAX_LIGHTSENSOR = 1000

# create the Robot instance.
robot = Robot()

# get the light sensors
lsNames = [
    'ls0', 'ls1', 'ls2', 'ls3',
    'ls4', 'ls5', 'ls6', 'ls7'
]

ls = []
for i in range(8):
    ls.append(robot.getDevice(lsNames[i]))
    ls[i].enable(TIME_STEP)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:

    # read sensors outputs
    lsValues = []
    for i in range(8):
        lsValues.append(ls[i].getValue())

    # TODO: get light values
    leftLight = np.mean(lsValues[4:8])
    rightLight = np.mean(lsValues[0:4])
    
    # TODO: calculate the left and right speed
    a = 0
    b = 10
    leftSpeed = a + b*rightLight/MAX_LIGHTSENSOR
    rightSpeed = a + b*leftLight/MAX_LIGHTSENSOR
 
    print("sensor [L-R]: %1.1f - %1.1f" % (leftLight, rightLight))
    print("speed  [L-R]: %1.1f - %1.1f" % (leftSpeed, rightSpeed))

 
    # Make sure the speeds do not exceed the maximum
    leftSpeed = np.clip(leftSpeed, -MAX_SPEED, MAX_SPEED)
    rightSpeed = np.clip(rightSpeed, -MAX_SPEED, MAX_SPEED)
 
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)