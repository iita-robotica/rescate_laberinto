# In charge of low level movement
from controller import Robot

import utilities

# devices
from devices.wheel import Wheel

from devices.camera import Camera
from devices.colour_sensor import ColourSensor

from devices.lidar import Lidar

from devices.gps import Gps
from devices.gyroscope import Gyroscope

from devices.comunicator import Comunicator


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
