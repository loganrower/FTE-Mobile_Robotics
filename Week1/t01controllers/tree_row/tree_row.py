"""tree_row controller."""
"""
Logan's Working Version!
"""
import numpy as np
import math
from controller import Robot
import matplotlib.pyplot as plt
import time

# time in [ms] of a simulation step
TIME_STEP = 64
MAX_SPEED = 6.28

# create the robot instance.
robot = Robot()

# initialize motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# initialize sensors
lidar = robot.getDevice("lidar")
lidar.enable(TIME_STEP)
lidar.enablePointCloud()

# getObjectRanges(ranges, d_thr = 0.2)
# Function to detect all objects in the lidar range scan.
# @param ranges             An array of length 360 containing the range (distance) values
# @param d_thr              A distance threshold to detect the presence of a new object
# @return detected_objects  A list with per object the range and bearing (angle in rad)
def getObjectRanges(ranges, d_thr = 0.2):
    # Step 1: Parse the whole range signal and create objects for each set of 
    # neighboring and similar distance values
    objects = []
    regions = []
    for i in range(len(ranges)):
        diff_neighbor = np.abs( ranges[i] - ranges[(i-1)%len(ranges)])
#        if math.isinf(ranges[i]):
        if(diff_neighbor > d_thr): # A new object starts
            if regions: objects.append(regions)
            regions = []
        if(not math.isinf(ranges[i])):
            regions.append([i, ranges[i]])
             
    #Step 2: go through all objects and calculate the mean bearing and distance
    obj_rb = []
    for obj in objects:
        range_id_and_d = np.mean(obj,axis=0)
        angle_deg = -(range_id_and_d[0]-180)
        obj_rb.append([range_id_and_d[1], np.deg2rad(angle_deg)])
    
    return np.array(obj_rb)
    
# lidarToCartesian(ranges)
# A function to translate the lidar range scan from polar 
# coordinates to Cartesian coordinates
def lidarToCartesian(ranges, angles):
    x = np.cos(angles) * ranges
    y = np.sin(angles) * ranges    
    return(x,y)
    
# objFromPolarToCartesian(obj_rb)
# Function to translate the range,bearing representation of 
# the objects to Cartesian x,y representation
def objFromPolarToCartesian(obj_rb):
    obj_xy = []
    for obj in obj_rb:
        r, b = obj[0], obj[1]
        x = np.cos(b)*r
        y = np.sin(b)*r    
        obj_xy.append([x, y])
    return np.array(obj_xy)
    
step_id = 0
while robot.step(TIME_STEP) != -1:
    # Get the ranges and bearings (angles) from the lidar scanner
    ranges = lidar.getRangeImage()
    angles = np.linspace(np.pi,-np.pi,len(ranges))
        
    # Detect the objects in the lidar scan
    obj_rb = getObjectRanges(ranges)
    left_obj_xy = []
    right_vals = []
    
    
    obj_xy = objFromPolarToCartesian(obj_rb)
    for vals in obj_xy:
        y= vals[1] # get the y value
        if y < 0: # this means that is is the left trees...
            left_obj_xy.append(vals)
    left_obj_xy = np.array(left_obj_xy)
    # now wil get the slope of the line and that ios what the robot will have to be
    ## parallel to this slope to have robot run!
    
    # WE WANT ROBOT TO BE PARALLELT TO THESE TREES
    ## These trees are all in a line and it should all have a slope
    ### of zero essentially
    line_coef_left = np.polyfit(left_obj_xy[:,0], left_obj_xy[:,1],1) # first coefficient is slope, second is intercept
    ## assigned the predicted slope and intercept values 
    slope, intercept =  line_coef_left[0],line_coef_left[1]
    
    # Plot the LiDAR scan and detected objects
    if(step_id % 30 == 0):
     #this is so that there is a step of 30 so every 30 loops 
     # it will generate a plot and not everytimne
    
        # scan_x, scan_y = lidarToCartesian(ranges, angles)
        #scane y is basically all of the different angles
        # fig = plt.figure("lidar plot")
        # plt.clf()        
        # plt.plot(scan_x, scan_y, '.')
        # for obj in obj_xy:
            # plt.plot([0,obj[0]],[0,obj[1]])
        
        # plt.show(block=False)
        # plt.pause(0.01)
        
        fig = plt.figure("test polyfit")
        plt.scatter(left_obj_xy[:,0], left_obj_xy[:,1])
        #y = m*x+b
        y = line_coef_left[0]*left_obj_xy[:,0] + line_coef_left[1]
        plt.plot(left_obj_xy[:,0],y, color = 'tab:red')
        
        
        plt.show(block=False)
        plt.pause(0.01)
        print(slope)
    
    # Now need to dictate the movement after the scan
    ## so the slope of the left side should be zero
    ## and once the robot is able to get that and mainatin that slope
    ## then it will be in parallel with the trees... ESSENTIALLY ZERO IN THIS CASE .001  is good
    if slope > .001:
        # orientation of robot needs to rotate to the left 
        # Set left motor low and right high
        leftspeed = 1.5
        rightspeed = 2
    elif slope < -.001:
        # orientation of robot needs to rotate right
        # set right low and left high
        leftspeed = 2
        rightspeed = 1.5
    else:
        # if the robot is dead on at a slope of .001 which is assumed 
        ### slope of the tree line for both sides then both right and left
        ### motors can be set to full speed
        leftspeed = 2
        rightspeed = 2
   
    leftMotor.setVelocity(leftspeed)
    rightMotor.setVelocity(rightspeed)
    
    step_id += 1