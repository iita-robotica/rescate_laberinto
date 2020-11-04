from controller import Robot
import cv2 as cv
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

# Node grid class for mapping
class NodeGrid:
    # Creates grid and prepares it
    def __init__(self, x, y, tileSize, offsets=[0,0]):
        self.grid = np.zeros((x, y), dtype=np.uint8)
        self.center = (self.grid.shape[0] // 2, self.grid.shape[1] // 2)
        self.tileSize = tileSize
        self.offsets = offsets
        self.orientations = {
            "up":[-1,0],
            "down": [1,0],
            "right": [0,1],
            "left": [0,-1],
            "center":[0,0]
        }

        for i in range(0, len(self.grid)):
            if i % 2 != 0:
                for j in range(0, len(self.grid[i])):
                    if j % 2 != 0:
                        self.grid[i][j] = 255

        # Empty walls
        for i in range(0, len(self.grid)):
            if (i + 1) % 2 != 0:
                for j in range(0, len(self.grid[i])):
                    if (j + 0) % 2 != 0:
                        self.grid[i][j] = 20

        for i in range(0, len(self.grid)):
            if (i + 0) % 2 != 0:
                for j in range(0, len(self.grid[i])):
                    if (j + 1) % 2 != 0:
                        self.grid[i][j] = 20

    # Prints the grid in the console
    def printMap(self):
        print(self.grid)

    # Returns the grid
    def getMap(self):
        return self.grid

    def setPosition(self, position, val, orientation="center", offsetTile=False):
        tile = self.getTileNode(position)
        return self.changeValue(tile, val, orientation)

    def getTileNode(self, pos):
        node = [int(((pos[1] + self.offsets[1]) // self.tileSize) * 2), int(((pos[0] + self.offsets[0]) // self.tileSize) * 2)]
        return node

    # Changes the value of a given node in the grid
    def changeValue(self, pos, val, orientation="center"):
        # print(pos)
        try:
            finalIndex = [pos[0] + self.center[0], pos[1] + self.center[1]]
            finalIndex[0] = finalIndex[0] + (self.orientations[orientation])[0]
            finalIndex[1] = finalIndex[1] + (self.orientations[orientation])[1]
            self.grid[int(finalIndex[0]), int(finalIndex[1])] = val
            if finalIndex[0] < 0 or finalIndex[1] < 0:
                return "undefined"
        except IndexError:
            return "undefined"

    # Gets the value of a given node of the grid
    def getValue(self, pos, orientation="center"):
        try:
            finalIndex = [pos[0] * 2 + self.center[0], pos[1] * 2 + self.center[1]]
            finalIndex[0] = finalIndex[0] + self.orientations[orientation][0]
            finalIndex[1] = finalIndex[1] + self.orientations[orientation][1]

            if finalIndex[0] < 0 or finalIndex[1] < 0:
                return "undefined"
            else:
                return self.grid[int(finalIndex[0]), int(finalIndex[1])]
        except IndexError:
            return "undefined"
    
    # Gets the tile of the position in the actual maze
    def getTile(self, position):
        node = [position[0] // self.tileSize, position[1] // self.tileSize]
        return node

    # Gets the walls and obstacles given the global positions
    def getOrientationInTile(self, inputPos):
        sideClearance = self.tileSize / 4
        centreClearance = self.tileSize / 4
        thicknessClearance = self.tileSize / 6
        #pos = inputPos
        pos = [inputPos[0] + self.offsets[0], inputPos[1] + self.offsets[1]]
        posTile = self.getTile(pos)
        tilePos = [posTile[0] * self.tileSize, posTile[1] * self.tileSize]
        posInTile = [pos[0] - tilePos[0], pos[1] - tilePos[1]]
        print("input pos: " + str(inputPos))
        print("tile: " + str(posTile))
        print("tile pos: " + str(tilePos))
        print("pos in tile: " + str(posInTile))
        if sideClearance < posInTile[1] < self.tileSize - sideClearance and posInTile[0] > self.tileSize - thicknessClearance:
            orientation =   "right"
        elif sideClearance < posInTile[1] < self.tileSize - sideClearance and posInTile[0] < thicknessClearance:
            orientation =  "left"
        elif sideClearance < posInTile[0] < self.tileSize - sideClearance and posInTile[1] > self.tileSize - thicknessClearance:
            orientation = "down"
        elif sideClearance < posInTile[0] < self.tileSize - sideClearance and posInTile[1] < thicknessClearance:
            orientation =  "up"
        elif centreClearance < posInTile[0] < self.tileSize - centreClearance and centreClearance < posInTile[1] < self.tileSize - centreClearance:
            orientation = "center"
        else:
            orientation = "undefined"
        print(orientation)
        return orientation


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
    def __init__(self, sensor, sensorAngle, robotDiameter, tileSize, timeStep):
        self.sensor = sensor
        self.sensor.enable(timeStep)
        self.offset = 1
        self.robotDiameter = robotDiameter
        self.angle = sensorAngle
        self.tileSize = tileSize
    # Gets the distance from the sensor value
    def getDistance(self):
        val = self.sensor.getValue()
        if val < 0.8:
            dist = mapVals(val, 0, 0.8, 0, self.tileSize * 2.4)
            dist += self.robotDiameter / 2
            dist += self.offset
            return dist
        return -1
    # Gets the current rotation of the sensor
    def __getAngle(self, globalRotation):
        return normalizeAngle(self.angle + globalRotation + 270)
    # Gets the global coordinates of the detection given the robot global rotation and position
    def getGlobalDetection(self, globalRotation, robotPos):
        dist = self.getDistance()
        angle = self.__getAngle(globalRotation)
        if dist != -1:
            pos = getCoords(angle, dist)
            pos[0] += robotPos[0]
            pos[1] += robotPos[1]
            return pos
        return -1

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
    
class RobotLayer:
    def __init__(self, timeStep, posMultiplier, maxVelocity, robotDiameter, tileSize):
        #Instantiations
        self.robot = Robot()
        # Constants
        self.posMultiplier = posMultiplier
        self.timeStep = timeStep
        self.maxVelocity = maxVelocity
        self.robotDiameter = robotDiameter
        self.tileSize = tileSize

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
        #Distance sensors
        self.distSensors = []
        self.distSensors.append(DistanceSensor(self.robot.getDistanceSensor("ps0"), 72.76564, self.robotDiameter, self.tileSize, self.timeStep))
        self.distSensors.append(DistanceSensor(self.robot.getDistanceSensor("ps1"), 44.11775, self.robotDiameter, self.tileSize, self.timeStep))
        self.distSensors.append(DistanceSensor(self.robot.getDistanceSensor("ps2"), 0, self.robotDiameter, self.tileSize, self.timeStep))
        self.distSensors.append(DistanceSensor(self.robot.getDistanceSensor("ps3"), 298.511, self.robotDiameter, self.tileSize, self.timeStep))
        self.distSensors.append(DistanceSensor(self.robot.getDistanceSensor("ps4"), 241.2152, self.robotDiameter, self.tileSize, self.timeStep))
        self.distSensors.append(DistanceSensor(self.robot.getDistanceSensor("ps5"), 180, self.robotDiameter, self.tileSize, self.timeStep))
        self.distSensors.append(DistanceSensor(self.robot.getDistanceSensor("ps6"), 135.791, self.robotDiameter, self.tileSize, self.timeStep))
        self.distSensors.append(DistanceSensor(self.robot.getDistanceSensor("ps7"), 107.1431, self.robotDiameter, self.tileSize, self.timeStep))
        

        # Variables for abstraction layer
        
        self.startTime = self.robot.getTime()
        
    def step(self):
        return self.robot.step(timeStep) != -1

    def getTime(self):
        return self.robot.getTime() - self.startTime
    
    def getRotationByPos(self, prevGlobalPos, globalPos):
        if prevGlobalPos == globalPos:
            return -1
        else:
            posDiff = [globalPos[0] - prevGlobalPos[0], globalPos[1] - prevGlobalPos[1]]
            rads = math.atan2(posDiff[0], posDiff[1])
            degs = rads * 180 / math.pi
            degs = normalizeAngle(degs)
            return degs
    
    

    def move(self, ratio1, ratio2):
        self.rightWheel.move(ratio1)
        self.leftWheel.move(ratio2)
    

    
# Clase de capa de obstarccion
# Abstraction layer class
class AbstractionLayer:
    def __init__(self,timeStep, initialState):
        # For self.robot
        # Constants
        self.posMultiplier = 100
        self.timeStep = timeStep
        self.maxVelocity = 6.28
        self.robotDiameter = 0.071 * self.posMultiplier
        self.tileSize = 0.12 * self.posMultiplier

        #Instantiations
        self.robot = RobotLayer(timeStep, self.posMultiplier, self.maxVelocity, self.robotDiameter, self.tileSize)
        self.seqMg = SequenceManager()
        self.stMg = StateManager(initialState)
        self.grid = NodeGrid(40, 40, self.tileSize, [self.tileSize / 2, self.tileSize / 2])
        

        # Variables for abstraction layer
        
        self.actualTime = self.robot.getTime()
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

    def doMapping(self):
        self.grid.setPosition(self.globalPos, 100)
        for sensor in self.robot.distSensors:
            detection = sensor.getGlobalDetection(self.globalRot, self.globalPos)
            if detection != -1:
                orientation = self.grid.getOrientationInTile(detection)
                if orientation != "undefined":
                        self.grid.setPosition(detection, 255, orientation, offsetTile=True)
    
    def showGrid(self):
        cv.imshow("ventana", cv.resize(self.grid.getMap(), (400, 400), interpolation=cv.INTER_AREA))     
    
    def sendMessage(self, indentifier):
        self.robot.emitter.sendMessage(self.globalPos, indentifier)
    
    def followCalculatedPath(self):
        pass

    def followPath(self):
        pass

    def seqMove(self,ratio1, ratio2):
        if self.seqMg.check():
            self.robot.move(ratio1, ratio2)
            self.seqMg.nextSeq()
        return self.seqMg.seqDone()
    
    # This function doesnt stop if the robot is blocked by an obstacle
    def seqMoveDist(self, ratio, dist):
        if self.seqMg.check():
            if self.seqMoveDistFirstTime:
                self.seqMoveDistStart = self.globalPos
                self.seqMoveDistFirstTime = False
            else:
                diffInX = max(self.seqMoveDistStart[0], self.globalPos[0]) - min(self.seqMoveDistStart[0], self.globalPos[0])
                diffInY = max(self.seqMoveDistStart[1], self.globalPos[1]) - min(self.seqMoveDistStart[1], self.globalPos[1])
                distFromStart = getDistance([diffInX, diffInY])
                if distFromStart < dist:
                    self.robot.move(ratio,ratio)
                else:
                    self.seqMoveDistFirstTime = True
                    self.robot.move(0,0)
                    self.seqMg.nextSeq()
        return self.seqMg.seqDone()

    
    # Poner antes de empezar una sequencia o de usar una funcion sequencial
    # Put before starting a sequence or using a sequencial function
    def startSequence(self):
        self.seqMg.startSequence()
    
    def changeState(self, newState):
        self.stMg.changeState(newState)
        self.seqMg.resetSequence()
    
    def isState(self, state):
        return self.stMg.checkState(state)

    def seqEvent(self):
        if self.seqMg.check():
            self.seqMg.nextSeq()
        return self.seqMg.seqDone()
    
    # Para la sequencia por la cantidad de segundos que uno le ponga
    # Stops a sequence for the given amount of seconds 
    def seqDelaySec(self, delay):
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
    def update(self):
        self.bottomUpdate()
        stepping = self.robot.step()
        self.topUpdate()
        return stepping
    
    def topUpdate(self):
            # Top updates
        self.actualTime = self.robot.getTime()
        self.globalPos = self.robot.gps.getPosition()
        if self.firstStep:
            self.prevGlobalPos = self.globalPos
            self.firstStep = False
        if self.rotDetectMethod == "velocity":
            self.globalRot = self.robot.gyro.update(r.actualTime, r.globalRot)
        elif self.rotDetectMethod == "position":
            rot = self.robot.getRotationByPos(self.prevGlobalPos, self.globalPos)
            if rot != -1:
                self.globalRot = rot
        self.colourTileType = self.robot.colourSensor.getTileType()
        diffInX = max(self.globalPos[0], self.prevGlobalPos[0]) -  min(self.globalPos[0], self.prevGlobalPos[0])
        diffInY = max(self.globalPos[1], self.prevGlobalPos[1]) -  min(self.globalPos[1], self.prevGlobalPos[1])
        self.diffInPos = getDistance([diffInX, diffInY])
        self.showGrid()
    
    def bottomUpdate(self):
        # Bottom updates
        self.prevGlobalPos = self.globalPos
        if self.doMap:
            self.doMapping()


# Instanciacion de capa de abstracción
# Abstraction layer instantiation
r = AbstractionLayer(timeStep, "start")

#MAIN PROGRAM
# Updates the global position, rotation, colorSensor position and colors, shows the grid and does mapping
while r.update():
    # v Program v

    if r.diffInPos >= r.tileSize:
        r.changeState("teleported")
    elif r.colourTileType == "trap":
        r.changeState("navigation")

    # Start state
    if r.isState("start"):
        r.doMap = False
        r.startSequence()
        if r.seqEvent():
            r.rotDetectMethod = "position"
        if r.seqMoveDist(0.8, r.tileSize / 2):
            r.rotDetectMethod = "velocity"
        if r.seqMoveDist(-0.8, r.tileSize / 2):
            r.doMap = True
            r.changeState("main")

    # Main state
    elif r.isState("main"):
        print("main state")
        r.followCalculatedPath()

    # Analyze state
    elif r.isState("analyze"):
        print("analyze state")

    # Visual victim state
    elif r.isState("visualVictim"):
        print("visualVictim state")
        r.startSequence()
        if r.seqDelaySec(3):
            r.sendMessage("N")
            r.changeState("main")
        
    # Heated victim state
    elif r.isState("heatVictim"):
        print("visualVictim state")
        r.startSequence()
        if r.seqDelaySec(3):
            r.sendMessage("T")
            r.changeState("main")

    # Teleported state
    elif r.isState("teleported"):
        r.doMap = False
        r.startSequence()
        if r.seqEvent():
            r.rotDetectMethod = "position"
        if r.seqMoveDist(0.8, r.tileSize / 2):
            r.rotDetectMethod = "velocity"
        if r.seqMoveDist(-0.8, r.tileSize / 2):
            r.doMap = True
            r.changeState("main")

    #print("diff in pos: " + str(r.diffInPos))
    print("Global position: " + str(r.globalPos))
    #print("Global rotation: " + str(round(r.globalRot)))
    #print("Tile type: " + str(r.colourSensor.getTileType()))

    cv.waitKey(1)



