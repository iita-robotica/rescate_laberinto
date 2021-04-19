from controller import Robot
import sys
sys.path.append(r"C:\\Users\\ANA\\Desktop\\Webots - Erebus\\rescate_laberinto\\Competencias\\Robocup_2021\\Equipo\\FinalCode")
import numpy as np
import cv2 as cv
import time
import math
#REMEMBER TO COPY-PASTE THIS FUNCTIONS ON TO FINAL CODE
from UtilityFunctions import *

timeStep = 16 * 2 


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
         
class Wheel:
    def __init__(self, wheel, maxVelocity):
        self.maxVelocity = maxVelocity
        self.wheel = wheel
        self.wheel.setPosition(float("inf"))
    # Moves the wheel at a ratio of the maximum speed (between 0 and 1)
    def move(self, ratio):
        if ratio > 1:
            ratio = 1
        self.wheel.setVelocity(ratio * self.maxVelocity)

class StateManager:
    def __init__(self, initialState):
        self.state = initialState
    def changeState(self, newState):
        self.state = newState
        return True
    def checkState(self, state):
        return self.state == state

class SequenceManager:
    def __init__(self):
        self.lineIdentifier = 0
        self.linePointer = 1
        self.done = False

    def resetSequence(self):
        self.linePointer = 1

    def startSequence(self):
        self.lineIdentifier = 0
        self.done = False

    def check(self):
        self.done = False
        self.lineIdentifier += 1
        return self.lineIdentifier == self.linePointer

    def nextSeq(self):
        self.linePointer += 1
        self.done = True

    def seqDone(self):
        return self.done

    # Can be used to make a function sequential or used in an if statement to make a code block sequential
    def simpleSeqEvent(self, function=None):
        if self.check():
            if function is not None:
                function()
                self.nextSeq()
            return True
        return False

    # The function inputted must return True when it ends
    def complexSeqEvent(self, function):
        if self.check():
            if function():
                self.nextSeq()
                return True
        return False
    
    def makeSimpleSeqEvent(self, function):
        def event(*args, **kwargs):
            if self.check():
                function(*args, **kwargs)
                self.nextSeq()
                return True
            return False
        return event

    def makeComplexSeqEvent(self, function):
        def event(*args, **kwargs):
            if self.check():
                if function(*args, **kwargs):
                    self.nextSeq()
                    return True
            return False
        return event


robot = Robot()

seqMg = SequenceManager()
seqPrint = seqMg.makeSimpleSeqEvent(print)

while robot.step(timeStep) != -1:
    seqMg.startSequence()
    seqPrint("Hello")
    seqMg.resetSequence()
