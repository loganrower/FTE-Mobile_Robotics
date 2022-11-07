"""red_camera controller."""

import numpy as np
import math
import cv2
from controller import Display, Robot

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
# enable camera
camera = robot.getDevice("camera1")
camera.enable(TIME_STEP)
# enable range
ranges = robot.getDevice("range-finder")
ranges.enable(TIME_STEP)

#detect blob based on lower_bound and upper_bound values and return x, y
def getBlobPositionInImage(image, lower_bound, upper_bound, min_size=1000, show_mask=False):
    #convert image to HSV
    color = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    #get the mask with ranges passed to this function
    mask = cv2.inRange(color, lower_bound, upper_bound)
    
    if(show_mask):
        cv2.imshow("mask", mask)
    
    #find condurs in the masked image
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    #if there is found an condur, find the center otherwise return x, y as zero
    x = -1
    y = -1
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        if(cv2.contourArea(largest_contour) > min_size):
            moments = cv2.moments(largest_contour)
            x = int(moments["m10"] / moments["m00"])
            y = int(moments["m01"] / moments["m00"])
            
    return x, y

# Function to caputer the camera image, to flip the axes and return it as
# a numoy array
def getCameraImage():
    image = np.asarray(camera.getImageArray(), dtype=np.uint8)
    image = np.transpose(image, (1,0,2))
    return(image)
    
def getDepthImage():
    depth = ranges.getRangeImage(data_type="buffer")
    depth = np.frombuffer(depth, dtype=np.float32)
    depth = depth.reshape(ranges.getHeight(), ranges.getWidth())
    return(depth)    

def showOpenCVImages(image_rgb, depth):
    # Translate to BGR image
    image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
    cv2.imshow("camera", image_bgr)
    cv2.imshow("depth", depth)
    cv2.waitKey(10)

def getRangeAndBearing(obj_x, obj_y, depth):
    obj_r = depth[obj_y, obj_x]
    half_width = camera.getWidth()/2
    half_fov = camera.getFov()/2
    obj_b = -half_fov*(obj_x - half_width)/half_width
    return obj_r, obj_b

### THE MAIN LOOP OF THE CONTROLLER
while robot.step(TIME_STEP) != -1:
    #get the image from the camera and the depth image from the range finder
    # as numpy array
    image = getCameraImage()
    depth = getDepthImage()
       
    # Display the images in two separate windows
    showOpenCVImages(image, depth)
    
    #detect blob by specifying the bounds of color in HSV values
    # Returns -1,-1 if no blob detected
    obj_x, obj_y = getBlobPositionInImage(
        image, 
        np.array([0,50,50]),  #Red object has been detected!
        np.array([20,255,255]),
        min_size = 1000,
        show_mask = True
    )
    
    if(obj_x>=0 and obj_y >=0): # Object is in the camera image
        print("Object detected")
        obj_r, obj_b = getRangeAndBearing(obj_x, obj_y, depth)
        
        # TODO: Add your code her to move towards the object and stop at 0.2m
        # Rotate on the spot by assigning opposite speeds to left and right wheel
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = -0.5 * MAX_SPEED
                
    else:  
        print("No Object, search for the object by rotating")  
        # Rotate on the spot by assigning opposite speeds to left and right wheel
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = -0.5 * MAX_SPEED
        
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)