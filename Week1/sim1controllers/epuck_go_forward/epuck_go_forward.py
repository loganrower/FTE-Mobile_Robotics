"""epuck_go_forward controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot


# create the Robot instance.
robot = Robot()

# get the time step of the current world.
# timestep = int(robot.getBasicTimeStep())
timestep = 64
MAX_SPEED = 6.28
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
leftmotor = robot.getDevice('left wheel motor')
rightmotor = robot.getDevice('right wheel motor')

# set the target position of the motors
## The positions for both motors were set to be infinite
### tHis means that the motors will run with a certain speed
### and withrespect to the time of run... so this means that there
### will be no limitations with regard to the number of rotations
leftmotor.setPosition(float('inf')) # this means that the wheels will rotate 10 radians
rightmotor.setPosition(float('inf')) # 

# Set the motor speeds at 10% the MAX_SPEED
leftmotor.setVelocity(0.1*MAX_SPEED)
rightmotor.setVelocity(0.1*MAX_SPEED)

#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
