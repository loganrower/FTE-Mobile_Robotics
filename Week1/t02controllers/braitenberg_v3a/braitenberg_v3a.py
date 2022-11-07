"""Controller for Vehicle 3a -> LOVE """

from controller import Robot, DistanceSensor, Motor
import numpy as np

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
        S = ls[i].getValue()
        s_prime = S/MAX_LIGHTSENSOR
        lsValues.append(s_prime)

    # TODO: get light values
    #         for leftLight, use sensor 4,5,6,7    
    #         for rightLight, use sensor 0,1,2,3 
       
    rightLight =  np.mean(lsValues[:4])
    leftLight = np.mean(lsValues[4:]) 
    print(lsValues)
    print("left light",leftLight)
    print("right light", rightLight)
    # TODO: calculate the left and right speed
    ## FOR Vehicle 2a -> "FEAR"
    a = MAX_SPEED
    b = 6
    leftSpeed = a - (b*leftLight)
    rightSpeed = a - (b*rightLight)
 
    # Make sure the speeds do not exceed the maximum
    leftSpeed = np.clip(leftSpeed, -MAX_SPEED, MAX_SPEED)
    rightSpeed = np.clip(rightSpeed, -MAX_SPEED, MAX_SPEED)
 
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)