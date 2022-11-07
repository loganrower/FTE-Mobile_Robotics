"""read_dist controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
# robot = Robot()
# TIME_STEP = 64

# initialized max speed
#MAX_SPEED = 6.28
# list of the device names
# psNames = ['ps0','ps1','ps2','ps3','ps4','ps5','ps6','ps7']
# ps = []
# Enabled all of the sensors for the distance sensors
#for i in range(len(psNames)): # for each of the sensors get the device and enable for Time_Setp
    # get the device and then append the name to the list
    #ps.append(robot.getDevice(psNames[i]))
    # then with the actual device ID then enable the device 
    # ps[i].enable(TIME_STEP)


# leftMotor = robot.getDevice('left wheel motor')
# rightMotor = robot.getDevice('right wheel motor')
# leftMotor.setPosition(float('inf'))
# rightMotor.setPosition(float('inf'))
# leftMotor.setVelocity(0.0)
# rightMotor.setVelocity(0.0)

def obj_detect(psNames,ps,MAX_SPEED):
    psValues = []
    for i in range(len(psNames)):
        # append the value to the values list
        ## used device ID to get the sensor reading
        psValues.append(ps[i].getValue())
    # Now with the values for the sensors determined the obstances are detected
    ## This is done by comparing if the values returned by the stiance sensor are bigger
    ## than the threshold
    # detect obstacles
    
    right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0
    left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0
    
    leftspeed = .5*MAX_SPEED
    rightspeed = .5*MAX_SPEED
    
    # so now will 
    if left_obstacle:
        # so if there is a left obstancle
        rightspeed = .2*MAX_SPEED
        rightspeed= rightspeed*-1 # opposite direction
    
    # if there is a right obstance
    elif right_obstacle:
        leftspeed = .2*MAX_SPEED
        leftspeed = leftspeed *-1 # left in opposite
    # run the motors... 
    leftMotor.setVelocity(leftspeed)
    rightMotor.setVelocity(rightspeed)

#Main loop:
#- perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    psValues = []
    for i in range(len(psNames)):
        #append the value to the values list
        # ## used device ID to get the sensor reading
        ps.append(robot.getDevice(psNames[i]))
        ps[i].enable(TIME_STEP)
    #Now with the values for the sensors determined the obstances are detected
    # ## This is done by comparing if the values returned by the stiance sensor are bigger
    # ## than the threshold
   
    #detect obstacles
    right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0
    left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0
    
    leftspeed = .5*MAX_SPEED
    rightspeed = .5*MAX_SPEED
    
    
    #so now will 
    if left_obstacle:
        #so if there is a left obstancle
        rightspeed = .2*MAX_SPEED
        rightspeed= rightspeed*-1 # opposite direction
    
    #if there is a right obstance
    elif right_obstacle:
        leftspeed = .2*MAX_SPEED
        leftspeed = leftspeed *-1 # left in opposite
    #run the motors... 
    leftMotor.setVelocity(leftspeed)
    rightMotor.setVelocity(rightspeed)
    pass

# Enter here exit cleanup code.
