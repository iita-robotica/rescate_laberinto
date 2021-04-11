from controller import Robot
import numpy as np
import cv2 as cv
import time


timeStep = 32            # Set the time step for the simulation
max_velocity = 6.28      # Set a maximum velocity time constant

# Make robot controller instance
robot = Robot()

'''
Every component on the robot is initialised through robot.getDevice("name") 
If the "name" does not register well, check the custom_robot.proto file in the /games/protos folder
There you will find the configuration for the robot including each component name
'''

# Define the wheels 
wheel1 = robot.getDevice("wheel1 motor")   # Create an object to control the left wheel
wheel2 = robot.getDevice("wheel2 motor") # Create an object to control the right wheel

# Set the wheels to have infinite rotation 
wheel1.setPosition(float("inf"))       
wheel2.setPosition(float("inf"))

lidar = robot.getDevice("lidar")
lidar.enable(timeStep)
lidar.enablePointCloud()

gps = robot.getDevice("gps")
gps.enable(timeStep)
#print(gps.getCoordinateSystem())

start = robot.getTime()
times = 0

while robot.step(timeStep) != -1:
    
    # pre-set each wheel velocity
    speed1 = 0
    speed2 = 0

    gpsVals = gps.getValues()
    #print(gpsVals)

    pos = [int(gpsVals[0] * 100), int(gpsVals[2] * 100)]
    array = np.zeros((400, 400), dtype=np.uint8)
    if times < 100:
        pointCloud = lidar.getPointCloud()
        detections = []
        for point in pointCloud:
            if point.x != float("inf") and point.x != float("-inf"):
                procesedPoint = [int(point.x * 100), int(point.y * 100), int(point.z * 100)]
                array[procesedPoint[0] + 200 + pos[0], procesedPoint[2] + 200 + pos[1]] = 255
                if procesedPoint not in detections:
                    detections.append(procesedPoint)
                    
        
        print(detections)
        #times = times + 1

    cv.imshow("grid", array)
    cv.waitKey(1)
    

    # Set the wheel velocity 
    wheel1.setVelocity(speed1)              
    wheel2.setVelocity(speed2)