"""spin_robot controller."""

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
leftMotor.setPosition(5)
rightMotor.setPosition(5)
speedl = MAX_SPEED*.8
speedr = MAX_SPEED*.5
leftMotor.setVelocity(speedl)
rightMotor.setVelocity(speedr)

# initialize sensors
# lidar = robot.getDevice("lidar")
# lidar.enable(TIME_STEP)
# lidar.enablePointCloud()

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

# def center_bot(step,left,right):
    # use from polar to cartesian to get the y val of coordinate
    # this what is used to see what what distance is across and then find
    ## The center which the robot will then use 
    # THE LEFT AND RIGTH HAVE ALREADY BEEN SEPARATED!
    
    # ROBOT HAS A 20.5mm diamter wheel so this means that with these
    ## Wheels we can calc the number of rotations needed to cover the distance.
    

# step_id = 0
# while robot.step(TIME_STEP) != -1:
    # Get the ranges and bearings (angles) from the lidar scanner
    # ranges = lidar.getRangeImage()
    # angles = np.linspace(np.pi,-np.pi,len(ranges))
        
    # Detect the objects in the lidar scan
    # obj_rb = getObjectRanges(ranges)
    # left_obj_xy = []
    # right_obj_xy = []
    
    
    # obj_xy = objFromPolarToCartesian(obj_rb)
    # for vals in obj_xy:
        # y= vals[1] # get the y value
        # if y < 0: # this means that is is the left trees...
            # left_obj_xy.append(vals)
        # else:
            # right_obj_xy.append(vals)
            
    # left_obj_xy = np.array(left_obj_xy)
    # right_obj_xy = np.array(right_obj_xy)
    # now wil get the slope of the line and that ios what the robot will have to be
    # ## parallel to this slope to have robot run!
    
    # len_left = len(left)
    # len_right = len(right)
    # val_left = np.random.randint(len_left,size=1)
    # val_right = np.random.randint(len_right,size=1)
    # yleft = left[val_left][1]
    # yright = right[val_right][1]
    # if step == 0:
    ## This is first time through
        # Add the y values together to get a result...
        # while abs(yleft-yright) > .01: 
        #so while there is a difference between thes that is greater than .01 
         #   So if these two values are not equal adjust this 
            # if yleft > yright:
                # dist = abs(yleft - yright)
          #      leftMotor.setVelocity(leftspeed)
           
           #     rightMotor.setVelocity(rightspeed)
            # elif yright > yleft:
    
    # step_id += 1
