from numpy.lib.type_check import imag
from controller import Robot
import sys
import math
import numpy as np
import struct
import cv2 as cv

#REMEMBER TO COPY-PASTE THIS FUNCTIONS ON TO FINAL CODE
sys.path.append(r"/home/ale/rescate_laberinto/Competencias/Roboliga_2021/FinalCode")
from UtilityFunctions import *
from CameraDetection import *

# Captures images and processes them
class Camera:
    def __init__(self, camera, timeStep):
        self.camera = camera
        self.camera.enable(timeStep)
        self.height = self.camera.getHeight()
        self.width = self.camera.getWidth()

    # Gets an image from the raw camera data
    def getImg(self):
        imageData = self.camera.getImage()
        return np.array(np.frombuffer(imageData, np.uint8).reshape((self.height, self.width, 4)))


# Tracks global rotation
class Gyroscope:
    def __init__(self, gyro, index, timeStep):
        self.sensor = gyro
        self.sensor.enable(timeStep)
        self.oldTime = 0.0
        self.index = index
        self.rotation = 0
        self.lastRads = 0

    # Do on every timestep
    def update(self, time):
        #print("Gyro Vals: " + str(self.sensor.getValues()))
        timeElapsed = time - self.oldTime  # Time passed in time step
        radsInTimestep = (self.sensor.getValues())[self.index] * timeElapsed
        self.lastRads = radsInTimestep
        finalRot = self.rotation + radsInTimestep
        self.rotation = normalizeRads(finalRot)
        self.oldTime = time

    # Gets the actual angular Velocity
    def getDiff(self):
        if self.lastRads < 0:
            return self.lastRads * -1
        
        return self.lastRads

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
    def __init__(self, gps, timeStep, coordsMultiplier=1):
        self.gps = gps
        self.gps.enable(timeStep)
        self.multiplier = coordsMultiplier
        self.__prevPosition = []
        self.position = self.getPosition()

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
            posDiff = ((self.position[0] - self.__prevPosition[0]), (self.position[1] - self.__prevPosition[1]))
            accuracy = getDistance(posDiff)
            #print("accuracy: " + str(accuracy))
            if accuracy > 0.001:
                degs = getDegsFromCoords(posDiff)
                return normalizeDegs(degs)
        return None


# Returns a point cloud of the detctions it makes
class Lidar():
    def __init__(self, device, timeStep, pointIsCloseThresh, pointIsCloseRange):
        self.device = device
        self.device.enable(timeStep)
        self.x = 0
        self.y = 0
        self.z = 0
        self.rotation = 0
        self.fov = device.getFov()
        self.verticalFov = self.device.getVerticalFov()
        self.horizontalRes = self.device.getHorizontalResolution()
        self.verticalRes = self.device.getNumberOfLayers()
        self.hRadPerDetection = self.fov / self.horizontalRes
        self.vRadPerDetection = self.verticalFov / self.verticalRes
        self.detectRotOffset = 0 #math.pi * 0.75
        self.maxDetectionDistance = 0.06 * 10
        self.pointIsClose = False
        self.pointIsCloseThresh = pointIsCloseThresh
        self.pointIsCloseRange = pointIsCloseRange

    # Does a detection pass and returns a point cloud with the results
    def getPointCloud(self, layers=range(3)):
        self.pointIsClose = False
        
        #(degsToRads(359 - radsToDegs(self.rotation)))
        #rangeImage = self.device.getRangeImageArray()
        #print("Lidar vFov: ", self.verticalFov/ self.verticalRes)
        pointCloud = []
        
        for layer in layers:
            actualVDetectionRot = (layer * self.vRadPerDetection) + self.verticalFov / 2
            depthArray = self.device.getLayerRangeImage(layer)
            actualHDetectionRot = self.detectRotOffset + ((2 * math.pi) - self.rotation)
            for item in depthArray:
                if item <= self.maxDetectionDistance:
                    if item != float("inf") and item != float("inf") * -1 and item != 0:
                        x = item * math.cos(actualVDetectionRot)
                        x += 0.06 * 0.2

                        if degsToRads(self.pointIsCloseRange[0]) > actualHDetectionRot > degsToRads(self.pointIsCloseRange[1]) and x < self.pointIsCloseThresh:
                            self.pointIsClose = True

                        coords = getCoordsFromRads(actualHDetectionRot, x)
                        pointCloud.append([coords[0] - 0, (coords[1] * -1) - 0])
                actualHDetectionRot += self.hRadPerDetection
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
        self.velocity = 0
        self.wheel.setPosition(float("inf"))
        self.wheel.setVelocity(0)

    # Moves the wheel at a ratio of the maximum speed (between 0 and 1)
    def move(self, ratio):
        if ratio > 1:
            ratio = 1
        elif ratio < -1:
            ratio = -1
        self.velocity = ratio * self.maxVelocity
        self.wheel.setVelocity(self.velocity)

# Reads the colour sensor
class ColourSensor:
    def __init__(self, sensor, distancefromCenter, timeStep):
        self.distance = distancefromCenter
        self.sensor = sensor
        self.sensor.enable(timeStep)
        self.r = 0
        self.g = 0
        self.b = 0
    
    def getPosition(self, robotGlobalPosition, robotGlobalRotation):
        realPosition = getCoordsFromDegs(robotGlobalRotation, self.distance)
        return [robotGlobalPosition[0] + realPosition[0], robotGlobalPosition[1] + realPosition[1]]
    
    def __update(self):
        colour = self.sensor.getImage()
        #print("Colourimg:", colour)
        self.r = self.sensor.imageGetRed(colour, 1, 0, 0)
        self.g = self.sensor.imageGetGreen(colour, 1, 0, 0)
        self.b = self.sensor.imageGetBlue(colour, 1, 0, 0)
        #print("Colour:", self.r, self.g, self.b)
    
    def __isTrap(self):
        return (35 < self.r < 45 and 35 < self.g < 45)
    def __isSwamp(self):
        return (200 < self.r < 210 and 165 < self.g < 175 and 95 < self.b < 105)
    def __isCheckpoint(self):
        return (self.r > 232 and self.g > 232 and self.b > 232)
    def __isNormal(self):
        return self.r == 227 and self.g == 227
    def __isBlue(self):
        return (55 < self.r < 65 and 55 < self.g < 65 and 245 < self.b < 255)
    def __isPurple(self):
        return (135 < self.r < 145 and 55 < self.g < 65 and 215 < self.b < 225)
    def __isRed(self):
        return (245 < self.r < 255 and 55 < self.g < 65 and 55 < self.b < 65)
    
    # Returns the type of tyle detected from the colour data
    def getTileType(self):
        self.__update()
        tileType = "undefined"
        if self.__isNormal():
            tileType = "normal"
        elif self.__isTrap():
            tileType = "hole"
        elif self.__isSwamp():
            tileType = "swamp"
        elif self.__isCheckpoint():
            tileType = "checkpoint"
        elif self.__isBlue():
            tileType = "connection1-2"
        elif self.__isPurple():
            tileType = "connection2-3"
        elif self.__isRed():
            tileType = "connection1-3"

        #print("Color: " + tileType)
        #print("r: " + str(self.r) + "g: " + str(self.g) + "b: " +  str(self.b))
        return tileType


class Comunicator:
    def __init__(self, emmiter, receiver, timeStep):
        self.receiver = receiver
        self.emmiter = emmiter
        self.receiver.enable(timeStep)
        self.lackOfProgress = False
        self.doGetWordInfo = True
        self.gameScore = 0
        self.remainingTime = 0
    
    def sendVictim(self, position, victimtype):
        self.doGetWordInfo = False
        letter = bytes(victimtype, "utf-8")
        position = multiplyLists(position, [100, 100])
        position = [int(position[0]), int(position[1])]
        message = struct.pack("i i c", position[0], position[1], letter)
        self.emmiter.send(message)
        
    
    def sendLackOfProgress(self):
        self.doGetWordInfo = False
        message = struct.pack('c', 'L'.encode()) # message = 'L' to activate lack of progress
        self.emmiter.send(message)
        
    
    def sendEndOfPlay(self):
        self.doGetWordInfo = False
        exit_mes = struct.pack('c', b'E')
        self.emmiter.send(exit_mes)
        
        
        print("Ended!!!!!")
    
    def sendMap(self, npArray):
         ## Get shape
        print(npArray)
        s = npArray.shape
        ## Get shape as bytes
        s_bytes = struct.pack('2i',*s)
        ## Flattening the matrix and join with ','
        flatMap = ','.join(npArray.flatten())
        ## Encode
        sub_bytes = flatMap.encode('utf-8')
        ## Add togeather, shape + map
        a_bytes = s_bytes + sub_bytes
        ## Send map data
        self.emmiter.send(a_bytes)
        #STEP3 Send map evaluate request
        map_evaluate_request = struct.pack('c', b'M')
        self.emmiter.send(map_evaluate_request)
        self.doGetWordInfo = False
    
    def requestGameData(self):
        if self.doGetWordInfo:
            message = struct.pack('c', 'G'.encode()) # message = 'G' for game information
            self.emmiter.send(message) # send message

    def update(self):
        
        if self.doGetWordInfo:
            """
            self.requestGameData()
            if self.receiver.getQueueLength() > 0: # If receiver queue is not empty
                receivedData = self.receiver.getData()
                if len(receivedData) > 2:
                    tup = struct.unpack('c f i', receivedData) # Parse data into char, float, int
                    if tup[0].decode("utf-8") == 'G':
                        self.gameScore = tup[1]
                        self.remainingTime = tup[2]
                        self.receiver.nextPacket() # Discard the current data packet
            """

            #print("Remaining time:", self.remainingTime)
            self.lackOfProgress = False
            if self.receiver.getQueueLength() > 0: # If receiver queue is not empty
                receivedData = self.receiver.getData()
                print(receivedData)
                if len(receivedData) < 2:
                    tup = struct.unpack('c', receivedData) # Parse data into character
                    if tup[0].decode("utf-8") == 'L': # 'L' means lack of progress occurred
                        print("Detected Lack of Progress!")
                        self.lackOfProgress = True
                    self.receiver.nextPacket() # Discard the current data packetelse:
        else:
            self.doGetWordInfo = True
        
class DistanceSensor:
    def __init__(self, threshold, distanceFromCenter, angle, sensor, timeStep):
        self.sensor = sensor
        self.angle = angle
        self.distance = distanceFromCenter
        self.timeStep = timeStep
        self.threshold = threshold
        self.position = [0, 0]
        self.sensor.enable(self.timeStep)
    
    def isFar(self):
        distance = self.sensor.getValue()
        #print("Sensor distance:", distance)
        return distance > self.threshold
    
    def setPosition(self, robotPosition, robotRotation):
        sensorRotation = robotRotation + self.angle
        sensorPosition = getCoordsFromDegs(sensorRotation, self.distance)
        self.position = sumLists(sensorPosition, robotPosition)

class FrontDistanceSensor():
    def __init__(self, sensor, threshold, timeStep, offset=0):
        self.sensor = sensor
        self.timeStep = timeStep
        self.sensor.enable(self.timeStep)
        self.threshold = threshold
        self.offset = offset
    
    def getValue(self):
        return self.sensor.getValue() - self.offset
    
    def isClose(self):
        return self.getValue() < self.threshold
    


        
# Abstraction layer for robot
class RobotLayer:
    def __init__(self, timeStep):
        self.maxWheelSpeed = 6.28
        self.timeStep = timeStep
        self.robot = Robot()
        self.prevRotation = 0
        self.rotation = 0
        self.globalPosition = [0, 0]
        self.prevGlobalPosition = [0, 0]
        self.positionOffsets = [0, 0]
        self.__useGyroForRotation = True
        self.time = 0
        self.rotateToDegsFirstTime = True
        self.delayFirstTime = True
        self.delayStart = self.robot.getTime()
        self.gyroscope = Gyroscope(self.robot.getDevice("gyro"), 1, self.timeStep)
        self.gps = Gps(self.robot.getDevice("gps"), self.timeStep)
        self.lidar = Lidar(self.robot.getDevice("lidar"), self.timeStep, 0.03, (0, 360))
        self.leftWheel = Wheel(self.robot.getDevice("wheel1 motor"), self.maxWheelSpeed)
        self.rightWheel = Wheel(self.robot.getDevice("wheel2 motor"), self.maxWheelSpeed) 
        self.colorSensor = ColourSensor(self.robot.getDevice("colour_sensor"), 0.037, 32)
        self.leftGroundSensor = DistanceSensor(0.04, 0.0523, 45, self.robot.getDevice("distance sensor2"), self.timeStep)
        self.rightGroundSensor = DistanceSensor(0.04, 0.0523, -45, self.robot.getDevice("distance sensor1"), self.timeStep)
        
        
        self.frontRightSensor = FrontDistanceSensor(self.robot.getDevice("distance sensor5"), 0.001, self.timeStep)
        self.middleRightSensor = FrontDistanceSensor(self.robot.getDevice("distance sensor4"), 0.02, self.timeStep) #, 0.0195)
        self.sideRightSensor = FrontDistanceSensor(self.robot.getDevice("distance sensor3"), 0.03, self.timeStep) #, 0.086)

        self.frontLeftSensor = FrontDistanceSensor(self.robot.getDevice("distance sensor6"), 0.001, self.timeStep)
        self.middleLeftSensor = FrontDistanceSensor(self.robot.getDevice("distance sensor7"), 0.02, self.timeStep) #, 0.0195)
        self.sideLeftSensor = FrontDistanceSensor(self.robot.getDevice("distance sensor8"), 0.03, self.timeStep) #, 0.086)



        self.comunicator = Comunicator(self.robot.getDevice("emitter"), self.robot.getDevice("receiver"), self.timeStep)
        self.rightCamera = Camera(self.robot.getDevice("camera2"), self.timeStep)
        self.leftCamera = Camera(self.robot.getDevice("camera1"), self.timeStep)
        self.victimClasifier = VictimClassifier()

        self.pointIsClose = False

    def printFrontDistance(self):
        print("Front distance: ", self.frontRightSensor.getValue(), self.frontLeftSensor.getValue())
        #print("Middle distance: ", self.middleRightSensor.getValue() - self.frontRightSensor.getValue(), self.middleLeftSensor.getValue() - self.frontRightSensor.getValue())
        print("Middle distance: ", self.middleRightSensor.getValue(), self.middleLeftSensor.getValue())
        print("Side distance: ", self.sideRightSensor.getValue(), self.sideLeftSensor.getValue())
    
    def frontIsBlocked(self):
        for sensor in (self.frontRightSensor, self.frontLeftSensor, self.middleLeftSensor, self.middleRightSensor, self.sideLeftSensor, self.sideRightSensor):
            if sensor.isClose():
                return True
        else:
            return False
    
    def getVictims(self):
        poses = []
        imgs = []
        for index, camera in enumerate((self.rightCamera, self.leftCamera)):
            cposes, cimgs, binaryImgs = self.victimClasifier.getVictimImagesAndPositions(camera.getImg())
            poses += cposes
            imgs += cimgs
            if len(binaryImgs):
                cv.imshow("Binary_image_" + str(index), binaryImgs[0])
        print("Victim Poses: ",poses)
        for img in imgs:
            print("Victim shape:", img.shape)
        closeVictims = self.victimClasifier.getCloseVictims(poses, imgs)
        finalVictims = []
        for closeVictim in closeVictims:
            finalVictims.append(self.victimClasifier.classifyVictim(closeVictim))
        return finalVictims
    
    def reportVictims(self, letter):
        self.comunicator.sendVictim(self.globalPosition, letter)
    
    def sendArray(self, array):
        self.comunicator.sendMap(array)
    
    def sendEnd(self):
        print("End sended")
        self.comunicator.sendEndOfPlay()
        

    # Decides if the rotation detection is carried out by the gps or gyro
    @property
    def rotationDetectionType(self):
        if self.__useGyroForRotation:
            return "gyroscope"
        else:
            return "gps"

    @rotationDetectionType.setter
    def rotationDetectionType(self, rotationType):
        if rotationType == "gyroscope":
            self.__useGyroForRotation = True
            
        elif rotationType == "gps":
            self.__useGyroForRotation = False
        else:
            raise ValueError("Invalid rotation detection type inputted")

    def delaySec(self, delay):
        print("Current delay: ", delay)
        if self.delayFirstTime:
            self.delayStart = self.robot.getTime()
            self.delayFirstTime = False
        else:
            if self.time - self.delayStart >= delay:
                
                self.delayFirstTime = True
                return True
        return False

    # Moves the wheels at the specified ratio
    def moveWheels(self, leftRatio, rightRatio):
        self.leftWheel.move(leftRatio)
        self.rightWheel.move(rightRatio)

    def rotateToDegs(self, degs, orientation="closest", maxSpeed=0.5):
        accuracy = 2
        if self.rotateToDegsFirstTime:
            #print("STARTED ROTATION")
            self.rotateToDegsFirstTime = False
        self.seqRotateToDegsInitialRot = self.rotation  
        self.seqRotateToDegsinitialDiff = round(self.seqRotateToDegsInitialRot - degs)
        diff = self.rotation - degs
        moveDiff = max(round(self.rotation), degs) - min(self.rotation, degs)
        if diff > 180 or diff < -180:
            moveDiff = 360 - moveDiff
        speedFract = min(mapVals(moveDiff, accuracy, 90, 0.2, 0.8), maxSpeed)
        if accuracy  * -1 < diff < accuracy or 360 - accuracy < diff < 360 + accuracy:
            self.rotateToDegsFirstTime = True
            return True
        else:
            if orientation == "closest":
                if 180 > self.seqRotateToDegsinitialDiff > 0 or self.seqRotateToDegsinitialDiff < -180:
                    direction = "right"
                else:
                    direction = "left"
            elif orientation == "farthest":
                if 180 > self.seqRotateToDegsinitialDiff > 0 or self.seqRotateToDegsinitialDiff < -180:
                    direction = "left"
                else:
                    direction = "right"
            else:
                direction = orientation

            if moveDiff > 10:
                if direction == "right":
                    self.moveWheels(speedFract * -1, speedFract)
                elif direction == "left":
                    self.moveWheels(speedFract, speedFract * -1)
            else: 
                if direction == "right":
                    self.moveWheels(speedFract * -0.5, speedFract)
                elif direction == "left":
                    self.moveWheels(speedFract, speedFract * -0.5)
            #print("speed fract: " +  str(speedFract))
            #print("target angle: " +  str(degs))
            #print("moveDiff: " + str(moveDiff))
            #print("diff: " + str(diff))
            #print("orientation: " + str(orientation))
            #print("direction: " + str(direction))
            #print("initialDiff: " + str(self.seqRotateToDegsinitialDiff))

        #print("ROT IS FALSE")
        return False
    
    def rotateSmoothlyToDegs(self, degs, orientation="closest", maxSpeed=0.5):
        accuracy = 2 
        seqRotateToDegsinitialDiff = round(self.rotation  - degs)
        diff = self.rotation - degs
        moveDiff = max(round(self.rotation), degs) - min(self.rotation, degs)
        if diff > 180 or diff < -180:
            moveDiff = 360 - moveDiff
        speedFract = min(mapVals(moveDiff, accuracy, 90, 0.2, 0.8), maxSpeed)
        if accuracy  * -1 < diff < accuracy or 360 - accuracy < diff < 360 + accuracy:
            self.rotateToDegsFirstTime = True
            return True
        else:
            if orientation == "closest":
                if 180 > seqRotateToDegsinitialDiff > 0 or seqRotateToDegsinitialDiff < -180:
                    direction = "right"
                else:
                    direction = "left"
            elif orientation == "farthest":
                if 180 > seqRotateToDegsinitialDiff > 0 or seqRotateToDegsinitialDiff < -180:
                    direction = "left"
                else:
                    direction = "right"
            else:
                direction = orientation
            if direction == "right":
                self.moveWheels(speedFract * -0.5, speedFract)
            elif direction == "left":
                self.moveWheels(speedFract, speedFract * -0.5)
            #print("speed fract: " +  str(speedFract))
            #print("target angle: " +  str(degs))
            #print("moveDiff: " + str(moveDiff))
            #print("diff: " + str(diff))
            #print("orientation: " + str(orientation))
            #print("direction: " + str(direction))
            #print("initialDiff: " + str(seqRotateToDegsinitialDiff))

        #print("ROT IS FALSE")
        return False

    def moveToCoords(self, targetPos):
        errorMargin = 0.01
        descelerationStart = 0.5 * 0.12
        diffX = targetPos[0] - self.globalPosition[0]
        diffY = targetPos[1] - self.globalPosition[1]
        #print("Target Pos: ", targetPos)
        #print("Used global Pos: ", self.globalPosition)
        #print("diff in pos: " + str(diffX) + " , " + str(diffY))
        dist = getDistance((diffX, diffY))
        #print("Dist: "+ str(dist))
        if errorMargin * -1 < dist < errorMargin:
            #self.robot.move(0,0)
            #print("FinisehedMove")
            return True
        else:
            
            ang = getDegsFromCoords((diffX, diffY))
            ang = normalizeDegs(ang)
            #print("traget ang: " + str(ang))
            ratio = min(mapVals(dist, 0, descelerationStart, 0.1, 1), 1)
            ratio = max(ratio, 0.8)
            if self.rotateToDegs(ang):
                self.moveWheels(ratio, ratio)
                #print("Moving")
        return False
    
    # Gets a point cloud with all the detections from lidar and distance sensorss
    def getDetectionPointCloud(self):

        rawPointCloud = self.lidar.getPointCloud(layers=(2,3))
        self.pointIsClose = self.lidar.pointIsClose

        processedPointCloud = []
        for point in rawPointCloud:
            procPoint = [point[0] + self.globalPosition[0], point[1] + self.globalPosition[1]]
            #procPoint = [procPoint[0] + procPoint[0] * 0.1, procPoint[1] + procPoint[1] * 0.1]
            processedPointCloud.append(procPoint)
        return processedPointCloud
    
    def getColorDetection(self):
        pos = self.colorSensor.getPosition(self.globalPosition, self.rotation)
        detection = self.colorSensor.getTileType()
        return pos, detection
    
    def trapsAtSides(self):
        sides = []
        if self.leftGroundSensor.isFar():
            sides.append(self.leftGroundSensor.position)
        if self.rightGroundSensor.isFar():
            sides.append(self.rightGroundSensor.position)
        return sides
    
    # Returns True if the simulation is running
    def doLoop(self):
        return self.robot.step(self.timeStep) != -1
    
    def getWheelDirection(self):
        if self.rightWheel.velocity + self.leftWheel.velocity == 0:
            return 0
        return (self.rightWheel.velocity + self.leftWheel.velocity) / 2
    
    # Must run every TimeStep
    def update(self):
        # Updates the current time
        self.time = self.robot.getTime()
        # Updates the gps, gyroscope
        self.gps.update()
        self.gyroscope.update(self.time)

        # Gets global position
        self.prevGlobalPosition = self.globalPosition
        self.globalPosition = self.gps.getPosition()
        self.globalPosition[0] += self.positionOffsets[0]
        self.globalPosition[1] += self.positionOffsets[1]

        if self.gyroscope.getDiff() < 0.00001 and self.getWheelDirection() >= 0:
            self.rotationDetectionType = "gps"
            
        else:
            self.rotationDetectionType = "gyroscope"

        self.prevRotation = self.rotation

        # Gets global rotation
        if self.__useGyroForRotation:
            self.rotation = self.gyroscope.getDegrees()
            print("USING GYRO")
        else:
            print("USING GPS")
            val = self.gps.getRotation()
            if val is not None:
                self.rotation = val
            self.gyroscope.setDegrees(self.rotation)
        
        # Sets lidar rotation
        self.lidar.setRotationDegrees(self.rotation + 0)
        
        self.rightGroundSensor.setPosition(self.globalPosition, self.rotation)
        self.leftGroundSensor.setPosition(self.globalPosition, self.rotation)

        print("Delay time:", self.time - self.delayStart)
        

        self.comunicator.update()

        #victims = self.camera.getVictims()
        #print("Victims: ", victims)

