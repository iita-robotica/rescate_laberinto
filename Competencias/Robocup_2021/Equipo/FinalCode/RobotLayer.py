from controller import Robot
import sys
import math
#REMEMBER TO COPY-PASTE THIS FUNCTIONS ON TO FINAL CODE
sys.path.append(r"C:\\Users\\ANA\\Desktop\\Webots - Erebus\\rescate_laberinto\\Competencias\\Robocup_2021\\Equipo\\FinalCode")
from UtilityFunctions import *


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

    # Returns the rotation on degrees
    def getDegrees(self):
        return radsToDegs(self.rotation)

    # Returns the rotation on radians
    def getRadians(self):
        return self.rotation

    # Sets the rotation in radians
    def setRadians(self, rads):
        self.rotation = rads

    # Sets the rotation in degrees
    def setDegrees(self, degs):
        self.rotation = degsToRads(degs)


# Tracks global position
class Gps:
    def __init__(self, gps,timeStep, coordsMultiplier=0):
        self.gps = gps
        self.gps.enable(timeStep)
        self.multiplier = coordsMultiplier
        self.__prevPosition = []
        self.position = []

    # updates gps, must run every timestep
    def update(self):
        self.__prevPosition = self.position
        self.position = self.getPosition()

    # Returns the global position
    def getPosition(self):
        vals = self.gps.getValues()
        return [vals[0] * self.multiplier, vals[2] * self.multiplier]

    # Returns the global rotation according to gps
    def getRotation(self):
        if self.__prevPosition != self.position:
            posDiff = [(self.position[0] - self.__prevPosition[0]), (self.position[1] - self.__prevPosition[1])]
            accuracy = getDistance(posDiff)
            #print("accuracy: " + str(accuracy))
            if accuracy > 0.1:
                degs = getDegsFromCoords(posDiff)
                return normalizeDegs(degs)
        return None


# Returns a point cloud of the detctions it makes
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

    # Does a detection pass and returns a point cloud with the results
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

    # Sets the rotation of the sensors in radians
    def setRotationRadians(self, rads):
        self.rotation = rads
    
    # Sets the rotation of the sensors in degrees
    def setRotationDegrees(self, degs):
        self.rotation = degsToRads(degs)


# Controlls a wheel
class Wheel:
    def __init__(self, wheel, maxVelocity):
        self.maxVelocity = maxVelocity
        self.wheel = wheel
        self.wheel.setPosition(float("inf"))

    # Moves the wheel at a ratio of the maximum speed (between 0 and 1)
    def move(self, ratio):
        if ratio > 1:
            ratio = 1
        elif ratio < -1:
            ratio = -1
        self.wheel.setVelocity(ratio * self.maxVelocity)


# Abstraction layer for robot
class RobotLayer:
    def __init__(self, timeStep):
        
        self.timeStep = timeStep
        self.robot = Robot()
        self.rotation = 0
        self.globalPosition = [0, 0]
        self.__useGyroForRoation = True
        self.time = 0
        self.gyroscope = Gyroscope(self.robot.getDevice("gyro"), 1, self.timeStep)
        self.gps = Gps(self.robot.getDevice("gps"), self.timeStep)
        self.lidar = Lidar(self.robot.getDevice("lidar"), self.timeStep)
        self.leftWheel = Wheel(self.robot.getDevice("wheel1 motor"), self.timeStep)
        self.rightWheel = Wheel(self.robot.getDevice("wheel2 motor"), self.timeStep) 

    # Decides if the rotation detection is carried out by the gps or gyro
    @property
    def rotationDetectionType(self):
        if self.__useGyroForRoation:
            return "gyroscope"
        else:
            return "gps"

    @rotationDetectionType.setter
    def rotationDetectionType(self, rotationType):
        if rotationType == "gyroscope":
            self.__useGyroForRoation = True
            self.gyroscope.setDegrees(self.rotation)
        elif rotationType == "gps":
            self.__useGyroForRoation = False
        else:
            raise ValueError("Invalid rotation detection type inputted")

    # Moves the wheels at the specified ratio
    def moveWheels(self, leftRatio, rightRatio):
        self.leftWheel.move(leftRatio)
        self.rightWheel.move(rightRatio)
    
    # Gets a point cloud with all the detections from lidar and distance sensorss
    def getDetectionPointCloud(self):
        return self.lidar.getPointCloud(layers=(1, 2))
    
    # Returns True if the simulation is running
    def doLoop(self):
        return self.robot.step(self.timeStep) != -1
    
    # Must run every TimeStep
    def update(self):
        # Updates the current time
        self.time = self.robot.getTime()
        # Updates the gps, gyroscope
        self.gps.update()
        self.gyroscope.update(self.time)

        # Gets global position
        self.globalPosition = self.gps.getPosition()

        # Gets global rotation
        if self.__useGyroForRoation:
            self.rotation = self.gyroscope.getDegrees()
        else:
            self.rotation = self.gps.getRotation()

        # Sets lidar rotation
        self.lidar.setRotationDegrees(self.rotation)

        




    
        


    

    

