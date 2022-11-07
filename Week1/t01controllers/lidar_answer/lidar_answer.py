import numpy as np
import math
from controller import Robot
import matplotlib.pyplot as plt
import time

"""

Logan's Working Version WORKS COMPARABLY TO THIS 
VERSION DEVELOIPED BY PROF

"""

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
    obj_xy = objFromPolarToCartesian(obj_rb)
    
    if(step_id % 30 == 0):
        # Plot the lidar scan in Cartesian coordinates
        # as well as the detected objects
        scan_x, scan_y = lidarToCartesian(ranges, angles)

        fig = plt.figure("lidar plot")
        plt.clf()        
        plt.plot(scan_x, scan_y, '.')
        for obj in obj_xy:
            plt.plot([0,obj[0]],[0,obj[1]])
        
        plt.show(block=False)
        plt.pause(0.01)
        
   
    # Implement the tree row following
    left_obj_xy = obj_xy[obj_xy[:,1] > 0]
    
    line_coef_left = np.polyfit(left_obj_xy[:,0], left_obj_xy[:,1],1)
    
    # Drive parallel to left row of trees    
    if(line_coef_left[0] > 0.001):
        print("Robot should move to the left")
        leftMotor.setVelocity(1.5)
        rightMotor.setVelocity(2)
    elif(line_coef_left[0] < -0.001):
        print("Robot should move to the right")
        leftMotor.setVelocity(2)
        rightMotor.setVelocity(1)
    else:
        print("Robot should move straight")
        leftMotor.setVelocity(2)
        rightMotor.setVelocity(2)
    

    step_id += 1

    