"""epuck_avoid_collision controller."""
"""
Distance sensors are scaled between 0 and 4096 where
4096 is a large amount of light and the object is close
and 0 is where no light is measured and thus no obstacle

Using the webots/distance_sensor.h file we can then use the functions 
and get values from distance sensors
"""
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

from controller import Robot, DistanceSensor, Motor

# define the duration of each physics step
## must be multiple of basicTimeStep
TIME_STEP = 64
# initialized max speed
MAX_SPEED = 6.28
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
# timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

## INITIALIZE DEVICES
### robot device is referenced by WbDeviceTag

ps = []
psNames = ['ps0','ps1','ps2','ps3','ps4','ps5','ps6','ps7']
for i in range(len(psNames)): # for each of the sensors get the device and enable for Time_Setp
    # get the device and then append the name to the list
    ps.append(robot.getDevice(psNames[i]))
    # then with the actual device ID then enable the device 
    ps[i].enable(TIME_STEP)
 
## INITIALIZE THE MOTORS
leftmotor = robot.getDevice('left wheel motor')
rightmotor = robot.getDevice('right wheel motor')
# set the motors to basically run the wheels for undefined number of rotations
leftmotor.setPosition(float('inf'))
rightmotor.setPosition(float('inf'))
# set the speed of the motors (0) initially zero because will move 
## when there is no object within a certain distance of the robot
leftmotor.setVelocity(0.0)
rightmotor.setVelocity(0.0)
 

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    # Read the distnace sensors:
    ## list of the values for the 8 sensors
    psValues = []
    for i in range(len(psNames)):
        # append the value to the values list
        ## used the actual device from the ps list
        psValues.append(ps[i].getValue())
    # Now with the values for the sensors determined the obstances are detected
    ## This is done by comparing if the values returned by the stiance sensor are bigger
    ## than the threshold
    # detect obstacles
    right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0
    left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0
    
    # Now actuate the wheels based on whether there is an obstance
    leftspeed = .5*MAX_SPEED
    rightspeed = .5*MAX_SPEED
    
    # So if there is a left obstancle
    ## need to redefine speeds...
    ### left is positive and right is negative in order to turn
    if left_obstacle:
        # so if there is a left obstancle
        rightspeed= rightspeed*-1 # opposite direction
    
    # if there is a right obstance
    elif right_obstacle:
        leftspeed = leftspeed *-1 # left in opposite
        
    leftmotor.setVelocity(leftspeed)
    rightmotor.setVelocity(rightspeed)
    #  val = ds.getValue()
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
