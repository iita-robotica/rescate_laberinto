from controller import Robot
import numpy as np
import struct
import math

# Global time step
timeStep = 16 * 2 

# Corrects the given angle to be in a range from 0 to 360
def normalizeAngle(ang):
    ang = ang % 360
    if ang < 0:
        ang += 360
    return ang

# Converts a number from a range of value to another
def mapVals(val, in_min, in_max, out_min, out_max):
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Gets x, y coordinates from a given angle and distance
def getCoords(angle, distance):
    rad = angle * math.pi / 180
    y = float(distance * math.cos(rad))
    x = float(distance * math.sin(rad))
    return [x, y]

# Gets the distance to given coordinates
def getDistance(position):
    return math.sqrt((position[0] ** 2) + (position[1] ** 2))

class Wheel:
    def __init__(self, wheel, maxVelocity):
        self.maxVelocity = maxVelocity
        self.wheel = wheel
        self.wheel.setPosition(float("inf"))
    # Moves the wheel at a ratio of the maximum speed
    def move(self, ratio):
        self.wheel.setVelocity(ratio * self.maxVelocity)

# Manages a distance sensor
class DistanceSensor:
    def __init__(self, sensor, sensorAngle, robotDiameter, timeStep):
        self.sensor = sensor
        self.sensor.enable(timeStep)
        self.offset = -1
        self.robotDiameter = robotDiameter
        self.angle = sensorAngle
    # Gets the distance from the sensor value
    def getDistance(self):
        val = self.sensor.getValue()
        dist = mapVals(val, 0.0, 0.8, 0, 12 * 2.4)
        dist += self.robotDiameter / 2
        dist += self.offset
        return dist
    # Gets the current rotation of the sensor
    def getAngle(self, globalRotation):
        return normalizeAngle(self.angle + globalRotation + 270)
    # Gets the global coordinates of the detection given the robot global rotation and position
    def getGlobalDetection(self, globalRotation, robotPos):
        pos = getCoords(self.getAngle(globalRotation), self.getDistance())
        pos[0] -= robotPos[0]
        pos[1] -= robotPos[1]
        return pos

# Tracks global rotation
class Gyroscope:
    def __init__(self, gyro, timeStep):
        self.sensor = gyro
        self.sensor.enable(timeStep)
        self.oldTime = 0.0
    # Do on every timestep
    def update(self, time, currentRotation):
        timeElapsed = time - self.oldTime  # Time passed in time step
        radsInTimestep = (self.sensor.getValues())[0] * timeElapsed
        degsInTimestep = radsInTimestep * 180 / math.pi
        finalRot = currentRotation + degsInTimestep
        finalRot = normalizeAngle(finalRot)
        self.oldTime = time
        return finalRot
    
    

# Reads the heat sensor
class HeatSensor:
    def __init__(self, sensor, thershold, timeStep):
        self.sensor = sensor
        self.sensor.enable(timeStep)
        self.threshold = thershold
    # Retuns True if it detects victim close
    def isClose(self):
        return self.sensor.getValue() > self.threshold

# Reads the colour sensor
class ColourSensor:
    def __init__(self, sensor, timeStep):
        self.sensor = sensor
        self.sensor.enable(timeStep)
        self.r = 0
        self.g = 0
        self.b = 0
    
    
    def __update(self):
        colour = self.sensor.getImage()
        self.r = colour[0]
        self.g = colour[1]
        self.b = colour[2]
    
    def __isTrap(self):
        return (57 < self.r < 61 and 57 < self.g < 61) or (self.r == 111 and self.g == 111)
    def __isSwamp(self):
        return (144 > self.r > 140 and 225 > self.g > 220 and self.b == 246)
    def __isCheckpoint(self):
        return (self.r == 255 and self.g == 255 and self.b == 255)
    def __isNormal(self):
        return self.r == 252 and self.g == 252
    # Returns the type of tyle detected from the colour data
    def getTileType(self):
        self.__update
        tileType = "undefined"
        if self.__isNormal():
            tileType = "normal"
        elif self.__isTrap():
            tileType = "trap"
        elif self.__isSwamp():
            tileType = "swamp"
        elif self.__isCheckpoint():
            tileType = "checkpoint"
        return tileType


# Tracks global position
class Gps:
    def __init__(self, gps,timeStep, coordsMultiplier=0):
        self.gps = gps
        self.gps.enable(timeStep)
        self.multiplier = coordsMultiplier
    # Returns the global position
    def getPosition(self):
        vals = self.gps.getValues()
        return [vals[0] * self.multiplier, vals[2] * self.multiplier]

# Captures images and processes them
class Camera:
    def __init__(self, camera, timeStep):
        self.camera = camera
        self.camera.enable(timeStep)
    # Gets an image from the raw camera data
    def getImg(self):
        imageData = self.camera.getImage()
        return np.array(np.frombuffer(imageData, np.uint8).reshape((self.camera.getHeight(), self.camera.getWidth(), 4)))

# Sends messages
class Emitter:
    def __init__(self, emmitter, coordsDivisor=0):
        self.emitter = emmitter
        self.divisor = coordsDivisor
    # Sends a message given a position and identifier
    def sendMessage(self,pos, identifier):
        message = struct.pack('i i c', pos[0] / self.divisor * 100, pos[1] / self.divisor * 100, identifier.encode())
        self.emitter.send(message)

class SeqOrder:
    def __init__(self):
        self.done = False
    
    def check(self):
        return self.done
    
    def finish(self):
        self.done = True

class StateManager:
    def __init__(self, initialState):
        self.state = initialState
    def changeSt(self, newState):
        self.state = newState
        return True
    def checkSt(self, state):
        return self.state == state

class SequenceManager:
    def __init__(self, initialState):
        self.state = initialState
        self.lineIdentifier = 0
        self.linePointer = 1
        self.done = False
    
    def resetSequence(self):
        self.linePointer = 1
    
    def startSequence(self):
        self.lineIdentifier = 0

        self.done = False
    
    def check(self):
        self.lineIdentifier += 1
        return self.lineIdentifier == self.linePointer
    
    def nextSeq(self):
        self.linePointer += 1
        self.done = True
    
    def seqDone(self):
        if self.done:
            self.done = False
            return True
        return False
    


# Clase de capa de obstarccion
# Abstraction layer class
class AbstractionLayer:
    def __init__(self,timeStep, initialState=""):

        self.robot = Robot()
        # For self.robot
        # Constants
        self.posMultiplier = 100
        self.timeStep = timeStep
        self.maxVelocity = 6.28
        self.robotDiameter = 0.071 * self.posMultiplier
        
        # Variables
        # Wheels
        self.leftWheel = Wheel(self.robot.getMotor("left wheel motor"), self.maxVelocity)
        self.rightWheel = Wheel(self.robot.getMotor("right wheel motor"), self.maxVelocity)
        #Cameras
        self.centreCamera = Camera(self.robot.getCamera("camera_centre"), self.timeStep)
        self.rightCamera = Camera(self.robot.getCamera("camera_right"), self.timeStep)
        self.leftCamera = Camera(self.robot.getCamera("camera_left"), self.timeStep)
        #Colour sensor
        self.colourSensor = ColourSensor(self.robot.getCamera("colour_sensor"), self.timeStep)
        #Emitter
        self.emitter = Emitter(self.robot.getEmitter("emitter"), self.posMultiplier)
        #Gps
        self.gps = Gps(self.robot.getGPS("gps"), self.timeStep, self.posMultiplier)
        #Gyro
        self.gyro = Gyroscope(self.robot.getGyro("gyro"), self.timeStep)
        #Heat sensors
        self.heatLeft = HeatSensor(self.robot.getLightSensor("left_heat_sensor"), 35, self.timeStep)
        self.heatRight = HeatSensor(self.robot.getLightSensor("right_heat_sensor"), 35, self.timeStep)

        # Variables for abstraction layer
        self.seqMg = SequenceManager(initialState)
        self.startTime = self.actualTime = self.robot.getTime()
        self.delayStart = 0.0
        self.delayFirstTime = True
        self.seqMoveDistStart = [0, 0]
        self.seqMoveDistFirstTime = True
        self.globalPos = self.prevGlobalPos = [0,0]
        self.globalRot = 0
        self.rotDetectMethod = "velocity"
        self.tileType = "undefined"
        self.diffInPos = 0
        self.firstStep = True
        self.doMap = True
    
    def sendMessage(self, indentifier):
        self.emitter.sendMessage(self.globalPos, indentifier)
    
    def getRotationByPos(self):
        if self.prevGlobalPos == self.globalPos:
            return -1
        else:
            posDiff = [self.globalPos[0] - self.prevGlobalPos[0], self.globalPos[1] - self.prevGlobalPos[1]]
            rads = math.atan2(posDiff[0], posDiff[1])
            degs = rads * 180 / math.pi
            degs = normalizeAngle(degs)
            return degs

    def move(self, ratio1, ratio2):
        self.rightWheel.move(ratio1)
        self.leftWheel.move(ratio2)

    def seqMove(self,ratio1, ratio2):
        if self.seqMg.check():
            self.move(ratio1, ratio2)
            self.seqMg.nextSeq()
        return self.seqMg.seqDone()
    
    def seqMoveDist(self, ratio, dist):
        if self.seqMg.check():
            if self.seqMoveDistFirstTime == True:
                self.seqMoveDistStart = self.globalPos
                self.seqMoveDistFirstTime = False
            else:
                diffInX = max(self.seqMoveDistStart[0], self.globalPos[0]) - min(self.seqMoveDistStart[0], self.globalPos[0])
                diffInY = max(self.seqMoveDistStart[1], self.globalPos[1]) - min(self.seqMoveDistStart[1], self.globalPos[1])
                distFromStart = getDistance([diffInX, diffInY])
                if distFromStart < dist:
                    self.move(ratio,ratio)
                else:
                    self.seqMoveDistFirstTime = True
                    self.move(0,0)
                    self.seqMg.nextSeq()
        return self.seqMg.seqDone()

    
    # Poner antes de empezar una sequencia o de usar una funcion sequencial
    # Put before starting a sequence or using a sequencial function
    def startSequence(self):
        self.seqMg.startSequence()
    
    def resetSequence(self):
        self.seqMg.resetSequence()

    def seqEvent(self):
        if self.seqMg.check():
            self.seqMg.nextSeq()
        return self.seqMg.seqDone()
    
    # Para la sequencia por la cantidad de segundos que uno le ponga
    # Stops a sequence for the given amount of seconds 
    def seqDelay(self, delay):
        if self.seqMg.check():
            if self.delayFirstTime:
                self.delayStart = self.robot.getTime()
                self.delayFirstTime = False
            else:
                if self.actualTime - self.delayStart >= delay:
                    self.delayFirstTime = True
                    self.seqMg.nextSeq()
        return self.seqMg.seqDone()
        
    # Hace un print en sequencia
    # Prints something in sequence
    def seqPrint(self, text):
        if self.seqMg.check():
            print(text)
            self.seqMg.nextSeq()
        return self.seqMg.seqDone()
        
    # returns True if simulation is running
    def step(self):
        return self.robot.step(timeStep) != -1


# Instanciacion de capa de abstracción
# Abstraction layer instantiation
r = AbstractionLayer(timeStep,"start")
stMg = StateManager("start")

#MAIN PROGRAM
while r.step():
    # Top updates
    r.actualTime = r.robot.getTime()
    r.globalPos = r.gps.getPosition()
    if r.firstStep:
        r.prevGlobalPos = r.globalPos
        r.firstStep = False

    if r.rotDetectMethod == "velocity":
        r.globalRot = r.gyro.update(r.actualTime, r.globalRot)
    elif r.rotDetectMethod == "position":
        rot = r.getRotationByPos()
        if rot != -1:
            r.globalRot = rot
    r.colourTileType = r.colourSensor.getTileType()
    diffInX = max(r.globalPos[0], r.prevGlobalPos[0]) -  min(r.globalPos[0], r.prevGlobalPos[0])
    diffInY = max(r.globalPos[1], r.prevGlobalPos[1]) -  min(r.globalPos[1], r.prevGlobalPos[1])
    r.diffInPos = getDistance([diffInX, diffInY])


    # --Put your program here--
    # v Demo program v

    if r.diffInPos > 11:
        stMg.changeSt("teleported")
    elif r.colourTileType == "trap":
        stMg.changeSt("navigation")

    # Start state
    if stMg.checkSt("start"):
        r.doMap = False
        r.startSequence()
        if r.seqEvent():
            r.rotDetectMethod = "position"
        if r.seqMoveDist(0.8, 6):
            r.rotDetectMethod = "velocity"
        if r.seqMoveDist(-0.8, 6):
            r.doMap = True
            stMg.changeSt("main")
            r.resetSequence()

    # Main state
    elif stMg.checkSt("main"):
        print("main state")

    # Analyze state
    elif stMg.checkSt("analyze"):
        print("analyze state")

    # Visual victim state
    elif stMg.checkSt("visualVictim"):
        print("visualVictim state")
        r.startSequence()
        if r.seqDelay(3):
            r.sendMessage("N")
            stMg.changeSt("main")
            r.resetSequence()
        
        
    # Heated victim state
    elif stMg.checkSt("heatVictim"):
        print("visualVictim state")
        r.startSequence()
        if r.seqDelay(3):
            r.sendMessage("T")
            stMg.changeSt("main")
            r.resetSequence()
        

    # Navigation state
    elif stMg.checkSt("navigation"):
        print("navigation state")
        stMg.changeSt("follow")
        r.resetSequence()

    elif stMg.checkSt("follow"):
        print("follow state")

    # Teletransported state
    elif stMg.checkSt("teleported"):
        r.doMap = False
        r.startSequence()
        if r.seqEvent():
            r.rotDetectMethod = "position"
        if r.seqMoveDist(0.8, 6):
            r.rotDetectMethod = "velocity"
        if r.seqMoveDist(-0.8, 6):
            r.doMap = True
            stMg.changeSt("main")
            r.resetSequence()

    #print("diff in pos: " + str(r.diffInPos))
    print("Global position: " + str(r.globalPos))
    #print("Global rotation: " + str(round(r.globalRot)))
    #print("Tile type: " + str(r.colourSensor.getTileType()))

    # Bottom updates
    r.prevGlobalPos = r.globalPos
    if r.doMap:
        pass
        # mapping

