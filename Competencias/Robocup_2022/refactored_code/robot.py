# In charge of low level movement
from controller import Robot
import math
import numpy as np
import struct
import cv2 as cv

import utilities

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
        timeElapsed = time - self.oldTime  # Time passed in time step
        radsInTimestep = (self.sensor.getValues())[self.index] * timeElapsed
        self.lastRads = radsInTimestep
        finalRot = self.rotation + radsInTimestep
        self.rotation = utilities.normalizeRads(finalRot)
        self.oldTime = time

    # Gets the actual angular Velocity
    def getDiff(self):
        if self.lastRads < 0:
            return self.lastRads * -1
        
        return self.lastRads

    # Returns the rotation on degrees
    def getDegrees(self):
        return utilities.radsToDegs(self.rotation)

    # Returns the rotation on radians
    def getRadians(self):
        return self.rotation

    # Sets the rotation in radians
    def setRadians(self, rads):
        self.rotation = rads

    # Sets the rotation in degrees
    def setDegrees(self, degs):
        self.rotation = utilities.degsToRads(degs)


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
            accuracy = utilities.getDistance(posDiff)
            if accuracy > 0.001:
                degs = utilities.getDegsFromCoords(posDiff)
                return utilities.normalizeDegs(degs)
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
        self.detectRotOffset = 0  # math.pi * 0.75
        self.maxDetectionDistance = 0.06 * 5
        self.minDetectionDistance = 0.06 * 0.5
        self.pointIsClose = False
        self.pointIsCloseThresh = pointIsCloseThresh
        self.pointIsCloseRange = pointIsCloseRange
        self.distBias = 0.06 * 0.2
        self.distCoeff = 1
        self.distFactor = 1 #0.8
    
    def getRotationsAndDistances(self, layers=range(3)):
        self.pointIsClose = False
        
        # (degsToRads(359 - radsToDegs(self.rotation)))
        # rangeImage = self.device.getRangeImageArray()
        # print("Lidar vFov: ", self.verticalFov/ self.verticalRes)

        rots = []
        distances = []
        
        for layer in layers:
            actualVDetectionRot = (layer * self.vRadPerDetection) + self.verticalFov / 2
            depthArray = self.device.getLayerRangeImage(layer)
            actualHDetectionRot = self.detectRotOffset + ((2 * math.pi) - self.rotation)
            for item in depthArray:
                if self.minDetectionDistance <= item:# <= self.maxDetectionDistance:

                    if item == float("inf") or item == float("inf") * -1:
                        item = 0.5
                    x = item * math.cos(actualVDetectionRot)
                    
                    x += self.distBias
                    x *= self.distCoeff
                    x = x ** self.distFactor


                    if utilities.degsToRads(self.pointIsCloseRange[0]) > actualHDetectionRot > utilities.degsToRads(self.pointIsCloseRange[1]) and x < self.pointIsCloseThresh:
                        self.pointIsClose = True

                    rots.append(actualHDetectionRot)
                    distances.append(x)
                actualHDetectionRot += self.hRadPerDetection
        return rots, distances


    # Does a detection pass and returns a point cloud with the results
    def getPointCloud(self, layers=range(3)):
        self.pointIsClose = False
        
        # (degsToRads(359 - radsToDegs(self.rotation)))
        # rangeImage = self.device.getRangeImageArray()
        # print("Lidar vFov: ", self.verticalFov/ self.verticalRes)

        pointCloud = []
        
        for layer in layers:
            actualVDetectionRot = (layer * self.vRadPerDetection) + self.verticalFov / 2
            depthArray = self.device.getLayerRangeImage(layer)
            actualHDetectionRot = self.detectRotOffset + ((2 * math.pi) - self.rotation)
            for item in depthArray:
                if self.minDetectionDistance <= item <= self.maxDetectionDistance:
                        #item = self.maxDetectionDistance
                    if item != float("inf") and item != float("inf") * -1 and item != 0:
                        x = item * math.cos(actualVDetectionRot)
                        x += self.distBias
                        x *= self.distCoeff
                        x = x ** self.distFactor

                        if utilities.degsToRads(self.pointIsCloseRange[0]) > actualHDetectionRot > utilities.degsToRads(self.pointIsCloseRange[1]) and x < self.pointIsCloseThresh:
                            self.pointIsClose = True

                        coords = utilities.getCoordsFromRads(actualHDetectionRot, x)
                        pointCloud.append([coords[0] - 0, (coords[1] * -1) - 0])
                actualHDetectionRot += self.hRadPerDetection
        return pointCloud

    # Sets the rotation of the sensors in radians
    def setRotationRadians(self, rads):
        self.rotation = rads
    
    # Sets the rotation of the sensors in degrees
    def setRotationDegrees(self, degs):
        self.rotation = utilities.degsToRads(degs)


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
        self.position = [0, 0]
        self.sensor = sensor
        self.sensor.enable(timeStep)
        self.r = 0
        self.g = 0
        self.b = 0
    
    def setPosition(self, robotGlobalPosition, robotGlobalRotation):
        realPosition = utilities.getCoordsFromDegs(robotGlobalRotation, self.distance)
        self.position = [robotGlobalPosition[0] + realPosition[0], robotGlobalPosition[1] + realPosition[1]]
    
    def __update(self):
        colour = self.sensor.getImage()
        # print("Colourimg:", colour)
        self.r = self.sensor.imageGetRed(colour, 1, 0, 0)
        self.g = self.sensor.imageGetGreen(colour, 1, 0, 0)
        self.b = self.sensor.imageGetBlue(colour, 1, 0, 0)
        # print("Colour:", self.r, self.g, self.b)
    
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

        # print("Color: " + tileType)
        # print("r: " + str(self.r) + "g: " + str(self.g) + "b: " +  str(self.b))
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
        position = utilities.multiplyLists(position, [100, 100])
        position = [int(position[0]), int(position[1])]
        message = struct.pack("i i c", position[0], position[1], letter)
        self.emmiter.send(message)

    def sendLackOfProgress(self):
        self.doGetWordInfo = False
        message = struct.pack('c', 'L'.encode())  # message = 'L' to activate lack of progress
        self.emmiter.send(message)

    def sendEndOfPlay(self):
        self.doGetWordInfo = False
        exit_mes = struct.pack('c', b'E')
        self.emmiter.send(exit_mes)
        print("Ended!!!!!")

    def sendMap(self, npArray):
        # Get shape
        print(npArray)
        s = npArray.shape
        # Get shape as bytes
        s_bytes = struct.pack('2i', *s)
        # Flattening the matrix and join with ','
        flatMap = ','.join(npArray.flatten())
        # Encode
        sub_bytes = flatMap.encode('utf-8')
        # Add togeather, shape + map
        a_bytes = s_bytes + sub_bytes
        # Send map data
        self.emmiter.send(a_bytes)
        # STEP3 Send map evaluate request
        map_evaluate_request = struct.pack('c', b'M')
        self.emmiter.send(map_evaluate_request)
        self.doGetWordInfo = False

    def requestGameData(self):
        if self.doGetWordInfo:
            message = struct.pack('c', 'G'.encode())  # message = 'G' for game information
            self.emmiter.send(message)  # send message

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

            self.lackOfProgress = False
            if self.receiver.getQueueLength() > 0:  # If receiver queue is not empty
                receivedData = self.receiver.getData()
                print(receivedData)
                if len(receivedData) < 2:
                    tup = struct.unpack('c', receivedData)  # Parse data into character
                    if tup[0].decode("utf-8") == 'L':  # 'L' means lack of progress occurred
                        print("Detected Lack of Progress!")
                        self.lackOfProgress = True
                    self.receiver.nextPacket()  # Discard the current data packetelse:
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

        return distance > self.threshold

    def setPosition(self, robotPosition, robotRotation):
        sensorRotation = robotRotation + self.angle
        sensorPosition = utilities.getCoordsFromDegs(sensorRotation, self.distance)
        self.position = utilities.sumLists(sensorPosition, robotPosition)

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
        # Maximum wheel speed
        self.maxWheelSpeed = 6.28
        # The timestep
        self.timeStep = timeStep
        # Robot object provided by webots
        self.robot = Robot()
        self.prevRotation = 0
        self.rotation = 0
        self.position = [0, 0]
        self.prevGlobalPosition = [0, 0]
        self.positionOffsets = [0, 0]

        self.rotationSensor = "gyro"

        self.time = 0
        self.rotateToDegsFirstTime = True
        self.delayFirstTime = True
        self.delayStart = self.robot.getTime()

        self.autoDecideRotation = True
        self.gyroscope = Gyroscope(self.robot.getDevice("gyro"), 1, self.timeStep)
        self.gps = Gps(self.robot.getDevice("gps"), self.timeStep)
        self.lidar = Lidar(self.robot.getDevice("lidar"), self.timeStep, 0.03, (0, 360))
        self.leftWheel = Wheel(self.robot.getDevice("wheel1 motor"), self.maxWheelSpeed)
        self.rightWheel = Wheel(self.robot.getDevice("wheel2 motor"), self.maxWheelSpeed)

        self.comunicator = Comunicator(self.robot.getDevice("emitter"), self.robot.getDevice("receiver"), self.timeStep)
        self.centerCamera = Camera(self.robot.getDevice("camera1"), self.timeStep)
        self.rightCamera = Camera(self.robot.getDevice("camera2"), self.timeStep)
        self.leftCamera = Camera(self.robot.getDevice("camera3"), self.timeStep)
        

        self.pointIsClose = False

    def frontIsBlocked(self):
        for sensor in (
                self.frontRightSensor,
                self.frontLeftSensor,
                self.middleLeftSensor,
                self.middleRightSensor,
                self.sideLeftSensor,
                self.sideRightSensor):

            if sensor.isClose():
                return True
        else:
            return False

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
            # print("STARTED ROTATION")
            self.rotateToDegsFirstTime = False
        self.seqRotateToDegsInitialRot = self.rotation
        self.seqRotateToDegsinitialDiff = round(self.seqRotateToDegsInitialRot - degs)
        diff = self.rotation - degs
        moveDiff = max(round(self.rotation), degs) - min(self.rotation, degs)
        if diff > 180 or diff < -180:
            moveDiff = 360 - moveDiff
        speedFract = min(utilities.mapVals(moveDiff, accuracy, 90, 0.2, 0.8), maxSpeed)
        if accuracy * -1 < diff < accuracy or 360 - accuracy < diff < 360 + accuracy:
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
            # print("speed fract: " +  str(speedFract))
            # print("target angle: " +  str(degs))
            # print("moveDiff: " + str(moveDiff))
            # print("diff: " + str(diff))
            # print("orientation: " + str(orientation))
            # print("direction: " + str(direction))
            # print("initialDiff: " + str(self.seqRotateToDegsinitialDiff))

        # print("ROT IS FALSE")
        return False
    
    def rotateSmoothlyToDegs(self, degs, orientation="closest", maxSpeed=0.5):
        accuracy = 2
        seqRotateToDegsinitialDiff = round(self.rotation - degs)
        diff = self.rotation - degs
        moveDiff = max(round(self.rotation), degs) - min(self.rotation, degs)
        if diff > 180 or diff < -180:
            moveDiff = 360 - moveDiff
        speedFract = min(utilities.mapVals(moveDiff, accuracy, 90, 0.2, 0.8), maxSpeed)
        if accuracy * -1 < diff < accuracy or 360 - accuracy < diff < 360 + accuracy:
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
            # print("speed fract: " +  str(speedFract))
            # print("target angle: " +  str(degs))
            # print("moveDiff: " + str(moveDiff))
            # print("diff: " + str(diff))
            # print("orientation: " + str(orientation))
            # print("direction: " + str(direction))
            # print("initialDiff: " + str(seqRotateToDegsinitialDiff))

        # print("ROT IS FALSE")
        return False

    def moveToCoords(self, targetPos):
        errorMargin = 0.01
        descelerationStart = 0.5 * 0.12
        diffX = targetPos[0] - self.position[0]
        diffY = targetPos[1] - self.position[1]
        # print("Target Pos: ", targetPos)
        # print("Used global Pos: ", self.position)
        # print("diff in pos: " + str(diffX) + " , " + str(diffY))
        dist = utilities.getDistance((diffX, diffY))
        print("Dist: "+ str(dist))
        if errorMargin * -1 < dist < errorMargin:
            # self.robot.move(0,0)
            print("FinisehedMove")
            return True
        else:
            
            ang = utilities.getDegsFromCoords((diffX, diffY))
            ang = utilities.normalizeDegs(ang)
            # print("traget ang: " + str(ang))
            ratio = min(utilities.mapVals(dist, 0, descelerationStart, 0.1, 1), 1)
            ratio = max(ratio, 0.8)
            if self.rotateToDegs(ang):
                self.moveWheels(ratio, ratio)
                # print("Moving")
        return False
    
    # Gets a point cloud with all the detections from lidar and distance sensorss
    def getDetectionPointCloud(self):

        rawPointCloud = self.lidar.getPointCloud(layers=(2,3))
        self.pointIsClose = self.lidar.pointIsClose
        return rawPointCloud
    
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
        self.prevGlobalPosition = self.position
        self.position = self.gps.getPosition()
        self.position[0] += self.positionOffsets[0]
        self.position[1] += self.positionOffsets[1]

        # Decides wich sensor to use for roatation detection
        # if the robot is going srtaight i tuses the gps
        
        if self.autoDecideRotation:
            if self.gyroscope.getDiff() < 0.00001 and self.getWheelDirection() >= 0:
                self.rotationSensor = "gps"
            # if it isn't going straight it uses the gyro
            else:
                self.rotationSensor = "gyro"

        # Remembers the corrent rotation for the next timestep
        self.prevRotation = self.rotation

        # Gets global rotation
        if self.rotationSensor == "gyro":
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
        

        #print("Delay time:", self.time - self.delayStart)
        
        self.comunicator.update()
