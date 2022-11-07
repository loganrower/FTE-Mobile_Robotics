"""Controller for Vehicle 3b -> Exploring """

"""
Added in the object detection stuff
"""

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
psNames = ['ps0','ps1','ps2','ps3','ps4','ps5','ps6','ps7']
ps = []
ls = []
for i in range(8):
    ls.append(robot.getDevice(lsNames[i]))
    ls[i].enable(TIME_STEP)
    ps.append(robot.getDevice(psNames[i]))  
    ps[i].enable(TIME_STEP)

     
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)


# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    
    # CHECK THE POSITON OF THE ROBOT AND ROTATE IF NEEDED
    psValues = []
 
    leftSpeed = .8*MAX_SPEED
    rightSpeed = .8*MAX_SPEED
    #so now will 

    #run the motors... 
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    # read sensors outputs
    lsValues = []
    for i in range(8):
        S = ls[i].getValue()
        s_prime = S/MAX_LIGHTSENSOR
        lsValues.append(s_prime)
        #append the value to the values list
        # ## used device ID to get the sensor reading
        psValues.append(ps[i].getValue())

    #detect obstacles
    
    right_obstacle = psValues[0] > 90.0 or psValues[1] > 90.0 or psValues[2] > 90.0
    left_obstacle = psValues[5] > 90.0 or psValues[6] > 90.0 or psValues[7] > 90.0
    
    if left_obstacle:
        #so if there is a left obstancle
        rightSpeed = .2*MAX_SPEED
        rightSpeed= rightSpeed*-1 # opposite direction
    
    #if there is a right obstance
    elif right_obstacle:
        leftSpeed = .2*MAX_SPEED
        leftSpeed = leftSpeed *-1 # left in opposite
    else: # No right or left obstacle...
        # TODO: get light values
        #         for leftLight, use sensor 4,5,6,7    
        #         for rightLight, use sensor 0,1,2,3 
           
        rightLight =  np.mean(lsValues[:4])
        leftLight = np.mean(lsValues[4:]) 
        # print(lsValues)
        # print("left light",leftLight)
        # print("right light", rightLight)
        # TODO: calculate the left and right speed
        ## FOR Vehicle 2a -> "FEAR"
        a = MAX_SPEED
        b = 6
        leftSpeed = a - (b*rightLight)
        rightSpeed = a - (b*leftLight)
     
        # Make sure the speeds do not exceed the maximum
        leftSpeed = np.clip(leftSpeed, -MAX_SPEED, MAX_SPEED)
        rightSpeed = np.clip(rightSpeed, -MAX_SPEED, MAX_SPEED)
     
        # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)