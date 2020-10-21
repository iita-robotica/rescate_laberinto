from controller import Robot
import numpy as np
import struct
import math

# Global time step
timeStep = 32 // 2

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
    def __init__(self, sensor, timeStep):
        self.sensor = sensor
        self.sensor.enable(timeStep)
    # Retuns True if it detects victim close
    def isClose(self):
        return self.sensor.getValue() > 35

# Reads the colour sensor
class ColourSensor:
    def __init__(self, sensor, timeStep):
        self.sensor = sensor
        self.sensor.enable(timeStep)

    # Returns the type of tyle detected from the colour data
    def getTileType(self):
        colour = self.sensor.getImage()
        r = colour[0]
        g = colour[1]
        b = colour[2]
        tileType = "undefined"
        if r == 252 and g == 252:
            tileType = "normal"
        elif (57 < r < 61 and 57 < g < 61) or (r == 111 and g == 111):
            tileType = "trap"
        elif 144 > r > 140 and 225 > g > 220 and b == 246:
            tileType = "swamp"
        elif r == 255 and g == 255 and b == 255:
            tileType = "checkpoint"
        return tileType

# Tracks global position
class Gps:
    def __init__(self, gps, timeStep):
        self.gps = gps
        self.gps.enable(timeStep)
    # Returns the global position
    def getPosition(self):
        vals = self.gps.getValues()
        return [int(vals[0] * 100), int(vals[2] * 100)]

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
    def __init__(self, emmitter):
        self.emitter = emmitter
    # Sends a message given a position and identifier
    def sendMessage(self,pos, identifier):
        message = struct.pack('i i c', pos[0], pos[1] * 100, identifier.encode())
        self.emitter.send(message)


# Clase de capa de obstarccion
# Abstraction layer class
class AbstractionLayer:
    def __init__(self,timeStep, initialState=""):

        self.robot = Robot()
        # For self.robot
        # Constants
        self.timeStep = timeStep
        self.maxVelocity = 6.28
        self.robotDiameter = 0.071
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
        self.emitter = Emitter(self.robot.getEmitter("emitter"))
        #Gps
        self.gps = Gps(self.robot.getGPS("gps"), self.timeStep)
        #Gyro
        self.gyro = Gyroscope(self.robot.getGyro("gyro"), self.timeStep)
        #Heat sensors
        self.heatLeft = HeatSensor(self.robot.getLightSensor("left_heat_sensor"), self.timeStep)
        self.heatRight = HeatSensor(self.robot.getLightSensor("right_heat_sensor"), self.timeStep)

        # Variables for abstraction layer
        self.state = initialState
        self.startTime = self.actualTime = self.robot.getTime()
        self.lineIdentifier = -1
        self.linePointer = 0
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
    
    def getRotationByPos(self):
        if self.prevGlobalPos == self.globalPos:
            return -1
        else:
            posDiff = [self.globalPos[0] - self.prevGlobalPos[0], self.globalPos[1] - self.prevGlobalPos[1]]
            rads = math.atan2(posDiff[0], posDiff[1])
            degs = rads * 180 / math.pi
            degs = normalizeAngle(degs)
            return degs
    
    # Advances simulation by 1 timeStep, returns True if simulation is running
    def step(self):
        return self.robot.step(timeStep) != -1

    def move(self, ratio1, ratio2):
        self.rightWheel.move(ratio1)
        self.leftWheel.move(ratio2)

    def seqMove(self,ratio1, ratio2):
        self.lineIdentifier += 1
        if self.lineIdentifier == self.linePointer:
            self.move(ratio1, ratio2)
            self.linePointer += 1
            return True
        return False
    
    def seqMoveDist(self, ratio, dist):
        self.lineIdentifier += 1
        if self.lineIdentifier == self.linePointer:
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
                    self.linePointer += 1
                    return True
        return False

    
    # Poner antes de empezar una sequencia o de usar una funcion sequencial
    # Put before starting a sequence or using a sequencial function
    def startSequence(self):
        self.lineIdentifier = -1
    
    # Para la sequencia por la cantidad de segundos que uno le ponga
    # Stops a sequence for the given amount of seconds 
    def seqDelay(self, delay):
        self.lineIdentifier += 1
        if self.lineIdentifier == self.linePointer:
            if self.delayFirstTime:
                self.delayStart = self.robot.getTime()
                self.delayFirstTime = False
            else:
                if self.actualTime - self.delayStart >= delay:
                    self.delayFirstTime = True
                    self.linePointer += 1
                    return True
        return False
        
    # Hace un print en sequencia
    # Prints something in sequence
    def seqPrint(self, text):
        self.lineIdentifier += 1
        if self.lineIdentifier == self.linePointer:
            print(text)
            self.linePointer += 1
            return True
        return False
        
    # Cambia el estado
    # Changes the state
    def changeState(self, newState):
        self.state = newState
        self.linePointer = 0

    # Poner al inicio del loop principal
    # Put at the start of the main loop
    def atTop(self):
        
        self.actualTime = self.robot.getTime()
        self.globalPos = self.gps.getPosition()
        if self.firstStep:
            self.prevGlobalPos = self.globalPos
            self.firstStep = False

        if self.rotDetectMethod == "velocity":
            self.globalRot = self.gyro.update(self.actualTime, self.globalRot)
        elif self.rotDetectMethod == "position":
            rot = self.getRotationByPos()
            if rot != -1:
                self.globalRot = rot
        self.colourTileType = self.colourSensor.getTileType()
        diffInX = max(self.globalPos[0], self.prevGlobalPos[0]) -  min(self.globalPos[0], self.prevGlobalPos[0])
        diffInY = max(self.globalPos[1], self.prevGlobalPos[1]) -  min(self.globalPos[1], self.prevGlobalPos[1])
        self.diffInPos = getDistance([diffInX, diffInY])
        
    
    def atBottom(self):
        self.prevGlobalPos = self.globalPos
        if self.doMap:
            pass
            # mapping


# Instanciacion de capa de abstracción
# Abstraction layer instantiation
r = AbstractionLayer(timeStep,"start")


#MAIN PROGRAM
while r.step():
    r.atTop()
    # --Put your program here--
    # v Demo program v

    if r.diffInPos > 11:
        r.state = "teleported"
    elif r.colourTileType == "trap":
        r.state = "navigation"

    # Start state
    if r.state == "start":
        r.doMap = False
        r.startSequence()
        if r.seqDelay(0):
            r.rotDetectMethod = "position"
        if r.seqMoveDist(0.8, 6):
            r.rotDetectMethod = "velocity"
        if r.seqMoveDist(-0.8, 6):
            r.doMap = True
            r.changeState("main")
    # Main state
    elif r.state == ("main"):
        print("main state")
    # Analyze state
    elif r.state == ("analyze"):
        print("analyze state")
    # Visual victim state
    elif r.state == ("visualVictim"):
        print("visualVictim state")
    # Heated victim state
    elif r.state == ("heatVictim"):
        print("visualVictim state")
    # Navigation state
    elif r.state == ("navigation"):
        print("navigation state")
    # Teletransported state
    elif r.state == ("teleported"):
        r.doMap = False
        r.startSequence()
        if r.seqDelay(0):
            r.rotDetectMethod = "position"
        if r.seqMoveDist(0.8, 6):
            r.rotDetectMethod = "velocity"
        if r.seqMoveDist(-0.8, 6):
            r.doMap = True
            r.changeState("main")
    


    print("diff in pos: " + str(r.diffInPos))
    #print("Global position: " + str(r.globalPos))
    #print("Global rotation: " + str(round(r.globalRot)))
    #print("Tile type: " + str(r.colourSensor.getTileType()))

    r.atBottom()
