from controller import Robot
import numpy as np
import cv2 as cv
import time
import math

# Corrects the given angle in degrees to be in a range from 0 to 360
def normalizeDegs(ang):
    ang = ang % 360
    if ang < 0:
        ang += 360
    if ang == 360:
        ang = 0
    return ang

def normalizeRads(rad):
    ang = radsToDegs(rad)
    normAng = normalizeDegs(ang)
    return degsToRads(normAng)

def degsToRads(deg):
    return deg * math.pi / 180

def radsToDegs(rad):
    return rad * 180 / math.pi

# Converts a number from a range of value to another
def mapVals(val, in_min, in_max, out_min, out_max):
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Gets x, y coordinates from a given angle in radians and distance
def getCoordsFromRads(rad, distance):
    y = float(distance * math.cos(rad))
    x = float(distance * math.sin(rad))
    return [x, y]

# Gets x, y coordinates from a given angle in degrees and distance
def getCoordsFromDegs(deg, distance):
    rad = degsToRads(deg)
    y = float(distance * math.cos(rad))
    x = float(distance * math.sin(rad))
    return [x, y]


# Gets the distance to given coordinates
def getDistance(position):
    return math.sqrt((position[0] ** 2) + (position[1] ** 2))

def isInRange(val, minVal, maxVal):
    return minVal < val < maxVal



    

class Lidar():
    def __init__(self, device, timeStep):
        self.device = device
        self.device.enable(timeStep)
        self.x = 0
        self.y = 0
        self.z = 0
        self.rotation = 0
        self.fov = device.getFov()
        self.verticalFov = self.device.getVerticalFov()
        self.horizontalRes = self.device.getHorizontalResolution()
        self.radPerDetection = self.fov / self.horizontalRes
        self.detectRotOffset = math.pi * 0.75

    def getPointCloud(self, layers=range(3)):
        actualDetectionRot = self.detectRotOffset + (degsToRads(360 - radsToDegs(self.rotation)))
        rangeImage = self.device.getRangeImageArray()
        pointCloud = []
        for depthArray in rangeImage:
            for index, item in enumerate(depthArray):
                if item != float("inf") and item != float("inf") * -1 and index in layers:
                    coords = getCoordsFromRads(actualDetectionRot, item)
                    pointCloud.append(coords)
            actualDetectionRot += self.radPerDetection
        return pointCloud

    def setRotationRadians(self, rads):
        self.rotation = rads

    def setRotationDegrees(self, degs):
        self.rotation = degsToRads(degs)


# Tracks global rotation
class Gyroscope:
    def __init__(self, gyro, index, timeStep):
        self.sensor = gyro
        self.sensor.enable(timeStep)
        self.oldTime = 0.0
        self.index = index
        self.rotation = 0

    # Do on every timestep
    def update(self, time):
        #print("Gyro Vals: " + str(self.sensor.getValues()))
        timeElapsed = time - self.oldTime  # Time passed in time step
        radsInTimestep = (self.sensor.getValues())[self.index] * timeElapsed
        finalRot = self.rotation + radsInTimestep
        self.rotation = normalizeRads(finalRot)
        self.oldTime = time

    def getDegrees(self):
        return radsToDegs(self.rotation)

    def getRadians(self):
        return self.rotation

    def setRadians(self, rads):
        self.rotation = rads

    def setDegrees(self, degs):
        self.rotation = degsToRads(degs)
         

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

lidar = Lidar(robot.getDevice("lidar"), timeStep)

gps = robot.getDevice("gps")
gps.enable(timeStep)

gyro = Gyroscope(robot.getDevice("gyro"), 1, timeStep)


start = robot.getTime()
array = np.zeros((400, 400), dtype=np.uint8)
arrayOffset = 200
speed1 = 1
speed2 = -1
times = 0
scale = 150

while robot.step(timeStep) != -1:

    gpsVals = gps.getValues()
    pos = [gpsVals[2] * -1, gpsVals[0] * -1]

    

    gyro.update(robot.getTime())
    rot = gyro.getDegrees()
    print(rot)

    lidar.setRotationDegrees(rot)
    
    pointCloud = lidar.getPointCloud((1,2))

    if times < 1:
        print(pointCloud)
        times += 1


    for point in pointCloud:
        x = int((point[0] + pos[0]) * scale) + arrayOffset
        y = int((point[1] + pos[1]) * scale) + arrayOffset
        if array[x, y] != 200:
            array[x, y] += 1


    #array[int(pos[0] * scale) + arrayOffset, int(pos[1] * scale) + arrayOffset] = 255
    # Set the wheel velocity 
    wheel1.setVelocity(speed1)              
    wheel2.setVelocity(speed2)

    cv.imshow("grid", cv.resize(array, (800, 800), interpolation=cv.INTER_AREA))
    cv.waitKey(1)