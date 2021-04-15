from controller import Robot
import numpy as np
import cv2 as cv
import time
import math

# Corrects the given angle to be in a range from 0 to 360
def normalizeAngle(ang):
    ang = ang % 360
    if ang < 0:
        ang += 360
    if ang == 360:
        ang = 0
    return ang

# Converts a number from a range of value to another
def mapVals(val, in_min, in_max, out_min, out_max):
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Gets x, y coordinates from a given angle and distance
def getCoords(rad, distance):
    #rad = angle * math.pi / 180
    y = float(distance * math.cos(rad))
    x = float(distance * math.sin(rad))
    return [x, y]

# Gets the distance to given coordinates
def getDistance(position):
    return math.sqrt((position[0] ** 2) + (position[1] ** 2))

def isInRange(val, minVal, maxVal):
    return minVal < val < maxVal

# Tracks global rotation
class Gyroscope:
    def __init__(self, gyro, index, timeStep):
        self.sensor = gyro
        self.sensor.enable(timeStep)
        self.oldTime = 0.0
        self.index = index
    # Do on every timestep
    def update(self, time, currentRotation):
        print("Gyro Vals: " + str(self.sensor.getValues()))
        timeElapsed = time - self.oldTime  # Time passed in time step
        radsInTimestep = (self.sensor.getValues())[self.index] * timeElapsed
        degsInTimestep = radsInTimestep * 180 / math.pi
        finalRot = currentRotation + degsInTimestep
        finalRot = normalizeAngle(finalRot)
        self.oldTime = time
        return finalRot

timeStep = 32            # Set the time step for the simulation
max_velocity = 6.28      # Set a maximum velocity time constant

# Make robot controller instance
robot = Robot()

# Define the wheels 
wheel1 = robot.getDevice("wheel1 motor")   # Create an object to control the left wheel
wheel2 = robot.getDevice("wheel2 motor") # Create an object to control the right wheel

# Set the wheels to have infinite rotation 
wheel1.setPosition(float("inf"))       
wheel2.setPosition(float("inf"))

lidar = robot.getDevice("lidar")
lidar.enable(timeStep)
#lidar.enablePointCloud()

gps = robot.getDevice("gps")
gps.enable(timeStep)

gyro = Gyroscope(robot.getDevice("gyro"), 1, timeStep)


start = robot.getTime()
array = np.zeros((400, 400), dtype=np.uint8)
speed1 = -3
speed2 = 3
rot = 0
times = 0
fov = lidar.getFov()
numberOfPoints = lidar.getHorizontalResolution()
radPerDetection = fov / numberOfPoints
scale = 100

while robot.step(timeStep) != -1:

    gpsVals = gps.getValues()
    pos = [int(gpsVals[2] * scale * -1), int(gpsVals[0] * scale * -1)]

    rot = gyro.update(robot.getTime(), rot)
    print(rot)
    
    actualDetectionRot = (1.5708 * 1.5) + ((360 - rot) * math.pi / 180)
    rangeImage = lidar.getRangeImageArray()

    pointCloud = []

    for depthArray in rangeImage:
        
        for index, item in enumerate(depthArray):
            if item != float("inf") and item != float("inf") * -1 and 0 < index < 3:
                coords = getCoords(actualDetectionRot, item)
                procesedPoint = [int(coords[0] * scale), int(coords[1] * scale)]
                pointCloud.append(coords)
                x = (procesedPoint[0] + 200 + pos[0]) * 1
                y = (procesedPoint[1] + 200 + pos[1]) * 1
                if array[x, y] != 200:
                    array[x, y] += 1
                
        actualDetectionRot += radPerDetection
    
    array[pos[0] + 200, pos[1] + 200] = 255

    if times < 1:
        #print(rangeImage)
        print(pointCloud)
        times += 1

    # Set the wheel velocity 
    wheel1.setVelocity(speed1)              
    wheel2.setVelocity(speed2)


    cv.imshow("grid", cv.resize(array, (800, 800), interpolation=cv.INTER_AREA))
    cv.waitKey(1)