from controller import Robot
import math
import copy
import numpy as np
import cv2 as cv

#-----------Utilty functions----------

# Corrects the given angle in degrees to be in a range from 0 to 360
def normalizeDegs(ang):
    ang = ang % 360
    if ang < 0:
        ang += 360
    if ang == 360:
        ang = 0
    return ang

# Corrects the given angle in radians to be in a range from 0 to a full rotaion
def normalizeRads(rad):
    ang = radsToDegs(rad)
    normAng = normalizeDegs(ang)
    return degsToRads(normAng)

# Converts from degrees to radians
def degsToRads(deg):
    return deg * math.pi / 180

# Converts from radians to degrees
def radsToDegs(rad):
    return rad * 180 / math.pi

# Converts a number from a range of value to another
def mapVals(val, in_min, in_max, out_min, out_max):
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Gets x, y coordinates from a given angle in radians and distance
def getCoordsFromRads(rad, distance):
    y = float(distance * math.cos(rad))
    x = float(distance * math.sin(rad))
    return (x, y)

# Gets x, y coordinates from a given angle in degrees and distance
def getCoordsFromDegs(deg, distance):
    rad = degsToRads(deg)
    y = float(distance * math.cos(rad))
    x = float(distance * math.sin(rad))
    return (x, y)

def getRadsFromCoords(coords):
    return math.atan2(coords[0], coords[1])

def getDegsFromCoords(coords):
    rads = math.atan2(coords[0], coords[1])
    return radsToDegs(rads)

# Gets the distance to given coordinates
def getDistance(position):
    return math.sqrt((position[0] ** 2) + (position[1] ** 2))

# Checks if a value is between two values
def isInRange(val, minVal, maxVal):
    return minVal < val < maxVal

def roundDecimal(number, decimal):
    return (round(number * decimal) / decimal)


# ----------------Robot Layer---------------------

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
    def __init__(self, gps, timeStep, coordsMultiplier=1):
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
            posDiff = ((self.position[0] - self.__prevPosition[0]), (self.position[1] - self.__prevPosition[1]))
            accuracy = getDistance(posDiff)
            #print("accuracy: " + str(accuracy))
            if accuracy > 0.001:
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
        self.verticalRes = self.device.getNumberOfLayers()
        self.hRadPerDetection = self.fov / self.horizontalRes
        self.vRadPerDetection = self.verticalFov / self.verticalRes
        self.detectRotOffset = 0 #math.pi * 0.75
        self.maxDetectionDistance = 0.06 * 10

    # Does a detection pass and returns a point cloud with the results
    def getPointCloud(self, layers=range(3)):
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
        self.wheel.setPosition(float("inf"))
        self.wheel.setVelocity(0)

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
        self.maxWheelSpeed = 6.28
        self.timeStep = timeStep
        self.robot = Robot()
        self.rotation = 0
        self.globalPosition = [0, 0]
        self.positionOffsets = [0, 0]
        self.__useGyroForRoation = True
        self.time = 0
        self.rotateToDegsFirstTime = True
        self.delayFirstTime = True
        self.gyroscope = Gyroscope(self.robot.getDevice("gyro"), 1, self.timeStep)
        self.gps = Gps(self.robot.getDevice("gps"), self.timeStep)
        self.lidar = Lidar(self.robot.getDevice("lidar"), self.timeStep)
        self.leftWheel = Wheel(self.robot.getDevice("wheel1 motor"), self.maxWheelSpeed)
        self.rightWheel = Wheel(self.robot.getDevice("wheel2 motor"), self.maxWheelSpeed) 

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
    
    def delaySec(self, delay):
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

    def rotateToDegs(self, degs, orientation="closest", maxSpeed=0.7):
        accuracy = 2
        if self.rotateToDegsFirstTime:
            #print("STARTED ROTATION")
            self.seqRotateToDegsInitialRot = self.rotation
            self.seqRotateToDegsinitialDiff = round(self.seqRotateToDegsInitialRot - degs)
            self.rotateToDegsFirstTime = False
        
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
            if direction == "right":
                self.moveWheels(speedFract * -1, speedFract)
            elif direction == "left":
                self.moveWheels(speedFract, speedFract * -1)
            #print("speed fract: " +  str(speedFract))
            #print("target angle: " +  str(degs))
            #print("moveDiff: " + str(moveDiff))
            #print("diff: " + str(diff))
            #print("orientation: " + str(orientation))
            #print("direction: " + str(direction))
            #print("initialDiff: " + str(self.seqRotateToDegsinitialDiff))

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
        processedPointCloud = []
        for point in rawPointCloud:
            procPoint = [point[0] + self.globalPosition[0], point[1] + self.globalPosition[1]]
            #procPoint = [procPoint[0] + procPoint[0] * 0.1, procPoint[1] + procPoint[1] * 0.1]
            processedPointCloud.append(procPoint)
        return processedPointCloud
    
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
        self.globalPosition[0] += self.positionOffsets[0]
        self.globalPosition[1] += self.positionOffsets[1]

        # Gets global rotation
        if self.__useGyroForRoation:
            self.rotation = self.gyroscope.getDegrees()
        else:
            val = self.gps.getRotation()
            if val is not None:
                self.rotation = val

        # Sets lidar rotation
        self.lidar.setRotationDegrees(self.rotation + 0)


#--------------Point Cloud To Grid --------------

# Converts a point cloud in to tiles with positions
class PointCloudQueManager:

    def __init__(self, queSize, pointMultiplier, queStep = 1):
        self.queSize = queSize # Defines the size of the point cloud que
        self.que = [] # A que of point clouds
        self.queStep = queStep
        self.queActualStep = 0
        self.pointMultiplier = pointMultiplier
        for _ in range(queSize):
            self.que.append([])
    
    # Converts the point coords in to ints and multiplies it with the point multiplier
    def processPointCloud(self, pointCloud):

        finalPointCloud = []
        for point in pointCloud:
            fpoint = [int(point[0] * self.pointMultiplier), int(point[1] * self.pointMultiplier), 1]
            alreadyInFinal = False
            for finalPoint in finalPointCloud:
                if fpoint[:2] == finalPoint[:2]:
                    finalPoint[2] += 1
                    alreadyInFinal = True
            if not alreadyInFinal:
                finalPointCloud.append(fpoint)
        return finalPointCloud
    
    # Merges all the point clouds in the que and returns them
    def getTotalPointCloud(self):
        totalPointCloud = []
        isInFinal = False
        for pointCloud in self.que:
            for item in pointCloud:
                for totalItem in totalPointCloud:
                    if item[:2] == totalItem[:2]:
                        isInFinal = True
                        totalItem[2] += item[2]
                if not isInFinal:
                    totalPointCloud.append(item)
        #print("total point cloud: ", totalPointCloud)
        return totalPointCloud

    # Adds a new point cloud to the que and removes the last element
    def update(self, pointCloud):
        if self.queActualStep >= self.queStep:
            self.que.pop(0)
            self.que.append(self.processPointCloud(pointCloud))
            self.queActualStep = 0
        else:
            self.queActualStep += 1
    
    

class PointCloudDivider:
    def __init__(self, tileSize, pointMultiplier, pointPermanenceThresh):
        # Defines the size of a tie in the scale of the original coordinates
        self.tileSize = tileSize
         # Defines the number to multiply the coordinates to convert them in to ints
        self.pointMultiplier = pointMultiplier
        # Defines the number of times a point has to repeat to be considered definitive
        self.pointPermanenceThresh = pointPermanenceThresh
        self.realTileSize = self.tileSize * self.pointMultiplier
    
    

    def getTile(self, position):
        return (int(position[0] // self.realTileSize), int(position[1] // self.realTileSize))
    
    def getPosInTile(self, position):
        return (int(position[0] % self.realTileSize), int(position[1] % self.realTileSize))
    
    # Returns a list with dictionarys containing the tile number and the position inside of said tile
    def getTiles(self, totalPointCloud):
        tiles = []
        #print("Total Point Cloud: ", totalPointCloud)
        for item in totalPointCloud:
            inTiles = False
            if item[2] >= self.pointPermanenceThresh:
                #print(item[:2])
                itemTile = self.getTile(item[:2])
                itemPosInTile = self.getPosInTile(item[:2])
                for tile in tiles:
                    if tile["tile"] == itemTile:
                        inTiles = True
                        tile["posInTile"].append(itemPosInTile)

                if not inTiles:
                    tiles.append({"tile":itemTile, "posInTile":[itemPosInTile]})
        #print("Tiles: ", tiles)
        return tiles

class PointCloudConverter:

    def __init__(self, tileSize, pointMultiplier):
        self.queManager = PointCloudQueManager(queSize=5, pointMultiplier=pointMultiplier)
        self.divider = PointCloudDivider(tileSize, pointMultiplier, pointPermanenceThresh=30)
        self.totalPointCloud = []
    

    def loadPointCloud(self, pointCloud):
        self.queManager.update(pointCloud)
        
    
    def getTilesWithPoints(self):
        self.totalPointCloud = self.queManager.getTotalPointCloud()
        return (self.divider.getTiles(self.totalPointCloud))
    

# Uses a dict of tile templates to return the elements present in a tile given points inside that tile
class Classifier:
    def __init__(self, tilesDict):
        self.tilesDict = tilesDict
        self.validTileDictKeys = list(self.tilesDict.keys())
        self.tilesDictPositivesCount = {}
        for key, value in self.tilesDict.items():
            count = 0
            for y in range(len(value)):
                for x in range(len(value[y])):
                    count += value[x][y]
            self.tilesDictPositivesCount[key] = count

    # Returns the similarity of the points imputted to the templates in percentages
    def getCalsificationPercentages(self, pointList):
        elementsDict = {}
        for key in self.validTileDictKeys:
            elementsDict[key] = 0

        for point in pointList:
            for key, modelGrid in self.tilesDict.items():
                elementsDict[key] += modelGrid[point[0]][point[1]]
                    
        return elementsDict

# ----------------- Analysis ----------------------------

# Class that defines a tile node in the grid
class TileNode:
    __wallFixtureTypes = ("harmed", "secure", "unharmed", "flammable_gas", "poison", "corrosive", "organic_proxide")
    # Tuple with all allowed tile types
    __allowedTypes = ("undefined", "normal", "hole", "swamp", "checkpoint", "start", "connection1-2", "connection2-3")
    __allowedCurvedWalls = ([1, 1], [-1, -1], [-1, 1], [-1, 1])
    __typesToNumbers = {"undefined" : "0", "normal": "0", "hole":"2", "swamp":"3", "checkpoint":"4", "start":"5", "connection1-2":"6", "connection1-3":"7", "connection2-3":"8"}
    def __init__(self, tileType="undefined", curvedWall=[0,0], fixtures=[], obstacles=[]):
        self.dimensions = [0.06, 0.06] # Dimensions of the tile
        self.tileType = tileType # Can be undefined, start, normal, swamp, hole, checkpoint, connection1-2, connection2-3
        self.traversed = False
        self.curvedWall = curvedWall # if it is a tile with curved walls and curved wall position
        self.fixtures = fixtures # List of all fixtures in walls adjacent to tile
        self.obstacles = obstacles # List of obstacles in tile
    
    
    # Defines what to print if I ask to print it
    def __repr__(self):
        return self.__typesToNumbers[self.tileType]
    def __str__(self):
        return self.__typesToNumbers[self.tileType]
    

# Class that defines a wall node in the grid
class WallNode:
    __wallFixtureTypes = ("H", "S", "U", "F", "P", "C", "O")
    def __init__(self, occupied=False, fixtures=[]):
        self.dimensions = [0.06, 0.06, 0.01] # Dimensions of the wall
        self.occupied = occupied # If there is a wall. Can be True or false.
        self.isFloating = False # If it is a floating wal
        self.traversed = False
        self.fixtures = fixtures # List of all fixtures in wall
    
     # Defines what to print if I ask to print it
    def __repr__(self):
        if len(self.fixtures):
            returnString = "".join(self.fixtures)
        elif self.occupied: returnString = "1"
        else: returnString = "0"
        return returnString
    def __str__(self):
        if len(self.fixtures):
            returnString = "".join(self.fixtures)
        elif self.occupied: returnString = "1"
        else: returnString = "0"
        return returnString


#Class that defines a vortex node in the grid
class VortexNode:
    def __init__(self, occupied=False):
        self.dimensions = [0.01, 0.01, 0.06] # Dimensions of the vortex
        self.occupied = occupied # If there is a vortex. Can be True or false.
        self.traversed = False
    
     # Defines what to print if I ask to print it
    def __repr__(self):
        return str(int(self.occupied))
    def __str__(self):
        return str(int(self.occupied))

# A virtual representation of the competition map
class Grid:
    
    def __init__(self, chunk, initialSize):
        self.startingSize = initialSize # The initial size of the grid, cant be 0 and has to be divisible by the size of the chunk
        self.size = [2, 2] # The actual size of the grid
        self.offsets = [0, 0] # Offsets of the grid to allow negative indexes
        self.grid = [[]] # The grid containing the data
        self.chunk = chunk # A chunk of nodes constituting the grid
        self.chunkSize = (len(chunk), len(chunk[0]))
        self.__constructGrid()
        
    # Given a string indicating direction returns an array directing to that direction
    def directionToNumber(self, direction):
        if direction == "center" or direction == "centre":
            n = [0, 0]
        elif direction == "right":
            n = [0, 1]
        elif direction == "left":
            n = [0, -1]
        elif direction == "up":
            n = [-1, 0]
        elif direction == "down":
            n = [1, 0]
        elif direction == "right-up" or direction == "up-right":
            n = [1, -1]
        elif direction == "right-down" or direction == "down-right":
            n = [1, 1]
        elif direction == "left-down" or direction == "down-left":
            n = [-1, 1]
        elif direction == "left-up" or direction == "up-left":
            n = [-1, -1]
        return n

    # Constructs the grid
    def __constructGrid(self):
        self.grid = copy.deepcopy(self.chunk)
        for _ in range((self.startingSize[0] // self.chunkSize[0]) - 1):
            self.addColumnAtEnd()
        for _ in range((self.startingSize[1] // self.chunkSize[1]) - 1):
            self.addRowAtEnd()

        self.offsets[0] = self.startingSize[0] // self.chunkSize[0]
        self.offsets[1] = self.startingSize[1] // self.chunkSize[1]
        if not self.offsets[0] % self.chunkSize[0]:
            self.offsets[0] -= 1
        if not self.offsets[1] % self.chunkSize[1]:
            self.offsets[1] -= 1

        self.size = self.startingSize
    
    # Adds a row at the end of the grid
    def addRowAtEnd(self):
        row = copy.deepcopy(self.chunk)
        if self.size[0] > 1:
            for _ in range((self.size[0] // self.chunkSize[0]) - 1):
                row = np.hstack((row.copy(), copy.deepcopy(self.chunk)))
            self.grid = np.vstack((self.grid.copy(), copy.deepcopy(row)))
            self.size[1] += self.chunkSize[0]
    
    # Adds a row at the start of the grid
    def addRowAtStart(self):
        row = copy.deepcopy(self.chunk)
        if self.size[0] > 1:
            for _ in range((self.size[0] // self.chunkSize[0]) - 1):
                row = np.hstack((row.copy(), copy.deepcopy(self.chunk)))
            self.grid = np.vstack((copy.deepcopy(row), self.grid.copy()))
            self.size[1] += self.chunkSize[0]
            self.offsets[1] += self.chunkSize[0]

    # Adds a column at the end of the grid
    def addColumnAtEnd(self):
        column = self.chunk.copy()
        if self.size[1] > 1:
            for _ in range((self.size[1] // self.chunkSize[1]) - 1):
                column = np.vstack((column.copy(), copy.deepcopy(self.chunk)))
            self.grid = np.hstack((self.grid.copy(), copy.deepcopy(column)))
            self.size[0] += self.chunkSize[1]

    # Adds a column at the start of the grid
    def addColumnAtStart(self):
        column = copy.deepcopy(self.chunk)
        if self.size[1] > 1:
            for _ in range((self.size[1] // self.chunkSize[1]) - 1):
                column = np.vstack((column.copy(), copy.deepcopy(self.chunk)))
            self.grid = np.hstack((copy.deepcopy(column), self.grid.copy()))
            self.size[0] += self.chunkSize[1]
            self.offsets[0] += self.chunkSize[1]
    
    # returns the node in the position in the grid taking in to account offsets
    def getRawNode(self, position):
        x = position[0] + self.offsets[0]
        y = position[1] + self.offsets[1]
        return self.grid[y][x]
    
    # Sets a value in the position in the grid taking in to account offsets
    def setRawNode(self, position, value):
        x = position[0] + self.offsets[0]
        y = position[1] + self.offsets[1]
        self.grid[y][x] = value

    def processedToRawNode(self, position, side=[0,0]):
        if isinstance(side, str):
            x = position[0] * self.chunkSize[0] + self.directionToNumber(side)[0]
            y = position[1] * self.chunkSize[1] + self.directionToNumber(side)[1]
        else:
            x = position[0] * self.chunkSize[0] + side[0]
            y = position[1] * self.chunkSize[1] + side[1]
        if x < self.offsets[0] * -1:
            raise IndexError("Index too small for list with min index " + str(self.offsets[0] * -1))
        if y < self.offsets[1] * -1:
            raise IndexError("Index too small for list with min index " + str(self.offsets[0] * -1))
        return (x,y)
    
    def rawToProcessedNode(self, rawNode):
        x = rawNode[0] // self.chunkSize[0]
        y = rawNode[1] // self.chunkSize[1]
        sideX = rawNode[0] % self.chunkSize[0]
        sideY = rawNode[1] % self.chunkSize[1]
        quadrant = [0, 0]
        if sideX > 0: quadrant[0] = 1
        if sideY > 0: quadrant[1] = 1
        return (x, y), quadrant



    # Returns a node given the position of a tile and directions to indicate walls and vertices
    def getNode(self, position, side=[0,0]):
        return self.getRawNode(self.processedToRawNode(position, side))
    
    # Sets a node given the position of a tile and directions to indicate walls and vertices
    def setNode(self, position, value, side=[0,0]):
        self.setRawNode(self.processedToRawNode(position, side), value)
    
    def getNumpyPrintableArray(self):
        printableArray = np.zeros(self.size, np.uint8)
        for y in range(len(self.grid)):
            for x, node in enumerate(self.grid[y]):
                if isinstance(node, TileNode):
                    if node.tileType == "hole":
                        printableArray[x][y] = 255
                elif isinstance(node, VortexNode):
                    if node.traversed:
                        printableArray[x][y] = 150
                else:
                    if node.occupied:
                        printableArray[x][y] = 255
                    else:
                        printableArray[x][y] = 50
        
        return np.flip(printableArray, 1)
                

# aStarNode class for A* pathfinding (Not to be confused with the node grid)
class aStarNode():
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


# Finds the best path to follow
class PathFinder:
    def __init__(self, vortexNode, wallNode, tileNode, grid, searchLimit):
        self.grid = grid
        self.startVortex = [0, 0]
        self.objectiveVortex = [0, 0]
        self.vortexNode = vortexNode
        self.wallNode = wallNode
        self.tileNode = tileNode
        self.searchLimit = searchLimit

    def isTraversable(self, index):
        node = self.grid.getRawNode(index)
        if isinstance(node, TileNode):
            raise ValueError("Invalid instance")
            return node.tileType != "hole"
        if isinstance(node, WallNode):
            raise ValueError("Invalid instance")
            return not node.occupied
        if isinstance(node, VortexNode):
            if node.occupied:
                return False
            traversable = True
            for adjacentIndex in ((-1, 1), (1, -1), (1, -1), (-1, 1), (0, 1), (0, -1), (1, 0), (-1, 0)):
                adjacent = self.grid.getRawNode((index[0] + adjacentIndex[0], index[1] + adjacentIndex[1]))
                if isinstance(adjacent, TileNode):
                    if adjacent.tileType == "":
                        traversable = False
                elif isinstance(adjacent, WallNode):
                    if adjacent.occupied:
                        traversable = False
                else:
                    raise ValueError(("invalid instance: " + str(type(adjacent))))
            return traversable
        return False
        

    # Returns a list of tuples as a path from the given start to the given end in the given maze
    def aStar(self, start, end):
        # Create start and end node
        startNode = aStarNode(None, (start[0], start[1]))
        startNode.g = startNode.h = startNode.f = 0
        endNode = aStarNode(None, (end[0], end[1]))
        endNode.g = endNode.h = endNode.f = 0
        # Initialize open and closed list
        openList = []
        closedList = []
        # Add the start node
        openList.append(startNode)
        # Loop until end
        while len(openList) > 0:
            # Get the current node
            currentNode = openList[0]
            currentIndex = 0
            for index, item in enumerate(openList):
                if item.f < currentNode.f:
                    currentNode = item
                    currentIndex = index
            # Pop current off open list, add to closed list
            openList.pop(currentIndex)
            closedList.append(currentNode)
            # If found the goal
            if currentNode == endNode:
                path = []
                current = currentNode
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                return path[::-1]  # Return reversed path
            # Generate children
            children = []
            for newPosition in ((0, 1), (0, -1), (-1, 0), (1, 0)):  # Adjacent squares
                # Get node position
                nodePosition = (currentNode.position[0] + (newPosition[0] * 2), currentNode.position[1] + (newPosition[1] * 2))
                # Make sure walkable terrain
                if not self.isTraversable(nodePosition):
                    continue
                # Create new node
                newNode = aStarNode(currentNode, nodePosition)
                # Append
                children.append(newNode)
            # Loop through children
            for child in children:
                continueLoop = False
                # Child is on the closed list
                for closedChild in closedList:
                    if child == closedChild:
                        continueLoop = True
                        break
                # Create the f, g, and h values
                child.g = currentNode.g + 1
                child.h = ((child.position[0] - endNode.position[0]) ** 2) + (
                            (child.position[1] - endNode.position[1]) ** 2)
                child.f = child.g + child.h
                # Child is already in the open list
                for openNode in openList:
                    if child == openNode and child.g > openNode.g:
                        continueLoop = True
                        break
                if continueLoop:
                    continue
                # Add the child to the open list
                openList.append(child)
    
    def isBfsAddable(self, index):
        node = self.grid.getRawNode(index)
        if isinstance(node, self.vortexNode):
            for adjacentPos in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
                adjacent = [index[0] + adjacentPos[0], index[1] + adjacentPos[1]]
                if not self.grid.getRawNode(adjacent).traversed:
                    return True
            return False
        else:
            return False

    # Breath First Search algorithm
    # Returns the tiles with in order and with the distance of each one
    def bfs(self, start, limit="undefined"):
        visited = []
        queue = []
        found = []
        start = [start[0], start[1], 0]
        visited.append(start)
        queue.append(start)
        while queue:
            coords = queue.pop(0)
            y = coords[1]
            x = coords[0]
            dist = coords[2]
            if limit != "undefined":
                if dist > limit:
                    break
            
            if self.isBfsAddable(coords):
                found.append(coords)
            for newPosition in (0, 1), (0, -1), (-1, 0), (1, 0):
                neighbour = [x + newPosition[0] * 2, y + newPosition[1] * 2, dist + 1]
                inList = False
                for node in visited:
                    if node[0] == neighbour[0] and node[1] == neighbour[1]:
                        inList = True
                        break
                if inList:
                    continue

                # Make sure walkable terrain
                try:
                    if self.isTraversable(neighbour):
                        visited.append(neighbour)
                        queue.append(neighbour)
                except IndexError:
                    pass
        return found

    def getPreferabilityScore(self, node):
        pass

    def setStartVortex(self, startRawVortexPos):
        if not isinstance(self.grid.getRawNode(startRawVortexPos), self.vortexNode):
            raise ValueError("Inputed position does not correspond to a vortex node")
        
        self.startVortex = startRawVortexPos
        if not self.isTraversable(startRawVortexPos):
            print("INITIAL VORTEX NOT TRAVERSABLE")
    
    def setGrid(self, grid):
        self.grid = grid

    def getBestPath(self):
        possibleNodes = self.bfs(self.startVortex, self.searchLimit)
        if len(possibleNodes) > 1:
            bestNode = possibleNodes[0]
            bestPath = self.aStar(self.startVortex ,bestNode)
            print("BFS NODES: ", possibleNodes[0:50])
            print("AStar PATH: ", bestPath)
            print("Start Vortex: ", self.startVortex)
            """
            if self.startVortex == bestPath[0]:
                return bestPath[1:]
            """
            return bestPath#[1:]
        return []
    
class Analyst:
    def __init__(self, tileSize):
        # Important variables
        self.tileSize = tileSize
        self.posMultiplier = 100
        #Grid
        gridChunk = np.array([[VortexNode(), WallNode()],
                              [WallNode()  , TileNode()]])
        self.grid = Grid(gridChunk, (100, 100))
        # Converter
        self.converter = PointCloudConverter(self.tileSize, pointMultiplier=self.posMultiplier)
        # Classifier
        classifTemp = {
        
            ("straight", "right"): np.array([[0, 0, 0, 0, 1, 1],
                                         [0, 0, 0, 0, 1, 1],
                                         [0, 0, 0, 0, 1, 1],
                                         [0, 0, 0, 0, 1, 1],
                                         [0, 0, 0, 0, 1, 1],
                                         [0, 0, 0, 0, 1, 1]]),

            ("straight", "left"): np.array([[1, 1, 0, 0, 0, 0],
                                        [1, 1, 0, 0, 0, 0],
                                        [1, 1, 0, 0, 0, 0],
                                        [1, 1, 0, 0, 0, 0],
                                        [1, 1, 0, 0, 0, 0],
                                        [1, 1, 0, 0, 0, 0]]),
                                
            ("straight", "up"): np.array([[1, 1, 1, 1, 1, 1],
                                         [1, 1, 1, 1, 1, 1],
                                         [0, 0, 0, 0, 0, 0],
                                         [0, 0, 0, 0, 0, 0],
                                         [0, 0, 0, 0, 0, 0],
                                        [0, 0, 0, 0, 0, 0]]),
                                
            ("straight", "down"): np.array([[0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0],
                                [1, 1, 1, 1, 1, 1],
                                [1, 1, 1, 1, 1, 1]]),

            ("curved", "right-up"): np.array([[1, 1, 1, 1, 0, 0],
                                        [1, 1, 1, 1, 1, 0],
                                        [0, 0, 0, 1, 1, 1],
                                        [0, 0, 0, 0, 1, 1],
                                        [0, 0, 0, 0, 1, 1],
                                        [0, 0, 0, 0, 1, 1]]),

            ("curved", "left-up"): np.array([[0, 0, 1, 1, 1, 1],
                                    [0, 1, 1, 1, 1, 1],
                                    [1, 1, 1, 0, 0, 0],
                                    [1, 1, 0, 0, 0, 0],
                                    [1, 1, 0, 0, 0, 0],
                                    [1, 1, 0, 0, 0, 0]]),
                                
            ("curved", "right-down"): np.array([[0, 0, 0, 0, 1, 1],
                                        [0, 0, 0, 0, 1, 1],
                                        [0, 0, 0, 0, 1, 1],
                                        [0, 0, 0, 1, 1, 1],
                                        [1, 1, 1, 1, 1, 0],
                                        [1, 1, 1, 1, 0, 0]]),
                                
            ("curved", "left-down"): np.array([[1, 1, 0, 0, 0, 0],
                                        [1, 1, 0, 0, 0, 0],
                                        [1, 1, 0, 0, 0, 0],
                                        [1, 1, 1, 0, 0, 0],
                                        [0, 1, 1, 1, 1, 1],
                                        [0, 0, 1, 1, 1, 1]])
            }
        self.classifier = Classifier(classifTemp)
        # Path finder
        self.pathFinder = PathFinder(VortexNode, WallNode, TileNode, self.grid, 10)
        self.pathFinder.setStartVortex((1, 1))
        #self.pathFinder.getBestPath()
        # Variables
        self.actualRawNode = []
        self.__bestPath = []
        self.calculatePath = True
        self.pathIndex = 0
        self.positionReachedThresh = 0.02
        self.startRawNode = [0 ,0]
    
    def loadPointCloud(self, pointCloud):
        self.converter.loadPointCloud(pointCloud)
        tilesWithPoints = self.converter.getTilesWithPoints()
        #print("tilesWithPoints: ", tilesWithPoints)
        for item in tilesWithPoints:
            percentages = self.classifier.getCalsificationPercentages(item["posInTile"])
            #print("percentages: ", percentages)
            for key, value in percentages.items():
                wallType, orientation = key
                if wallType == "straight":
                    if value >= 5:
                        self.grid.getNode(item["tile"], orientation).occupied = True
    
    def loadColorDetection(self, colorSensorPosition, tileType):
        convPos = self.getTile(colorSensorPosition)
        self.grid.getNode(convPos).tileType = tileType

    def getQuadrant(self, posInTile):
        if posInTile[0] > self.tileSize / 2: x = 1
        else: x = -1
        if posInTile[1] > self.tileSize / 2: y = 1
        else: y = -1
        return [x, y]

    
    def getTile(self, position):
        return (int(position[0] // self.tileSize), int(position[1] // self.tileSize))
    
    def getPosInTile(self, position):
        return ((position[0] % self.tileSize), (position[1] % self.tileSize))
    
    def getVortexPosInTile(self, quadrant):
        return [(self.tileSize / 2) + (quadrant[0] * (self.tileSize / 2)), (self.tileSize / 2) + (quadrant[1] * (self.tileSize / 2))]

    def getTilePos(self, tile):
        return (tile[0] * self.tileSize, tile[1] * self.tileSize)

    def multiplyPos(self, position):
        return (position[0] * self.posMultiplier, position[1] * self.posMultiplier)
    
    def getStartRawNodePos(self):
        node, quadrant = self.grid.rawToProcessedNode(self.pathFinder.startVortex)
        nodePos = self.getTilePos(node)
        
        vortexPos = self.getVortexPosInTile(quadrant)
        return [nodePos[0] + vortexPos[0], nodePos[1] + vortexPos[1]]


  
    def update(self, position):
        posInTile = self.getPosInTile(position)
        quadrant = self.getQuadrant(posInTile)
        tile = self.getTile(position)
        startRawNode = self.grid.processedToRawNode(tile, quadrant)
        self.startRawNode = startRawNode
        print("startRawNode: ", startRawNode)
        self.pathFinder.setStartVortex(startRawNode)
        self.pathFinder.setGrid(self.grid)

        vortexPosInTile = self.getVortexPosInTile(quadrant)
        diff = [vortexPosInTile[0] - posInTile[0], vortexPosInTile[1] - posInTile[1]]
        distToVortex = getDistance(diff)
        if distToVortex < self.positionReachedThresh:
            self.grid.getRawNode(self.startRawNode).traversed = True
            for adjacentPos in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
                adjacent = [self.startRawNode[0] + adjacentPos[0], self.startRawNode[1] + adjacentPos[1]]
                self.grid.getRawNode(adjacent).traversed = True
            

        if len(self.__bestPath):
            print("Dist to Vortex: ", distToVortex)
            if distToVortex < self.positionReachedThresh and startRawNode == self.__bestPath[self.pathIndex]:
                self.pathIndex += 1

            

        print("PathLenght: ", len(self.__bestPath))
        if self.pathIndex >= len(self.__bestPath):
            self.calculatePath = True

        else:
            bestNode = self.getBestRawNodeToMove()
            if bestNode is not None:
                if not self.pathFinder.isTraversable(bestNode):
                    self.calculatePath = True

        if self.calculatePath:
            print("Calculating path")
            self.__bestPath = self.pathFinder.getBestPath()
            self.pathIndex = 0
            self.calculatePath = False
    
    def getBestRawNodeToMove(self):
        print("Best path: ", self.__bestPath)
        print("Index: ", self.pathIndex)
        if len(self.__bestPath):
            return self.__bestPath[self.pathIndex]
        else:
            return None
    
    def getBestPosToMove(self):
        bestRawNode = self.getBestRawNodeToMove()
        print("BEST PATH: ", bestRawNode)
        if bestRawNode is None:
            return None
        node, quadrant = self.grid.rawToProcessedNode(bestRawNode)
        
        nodePos = self.getTilePos(node)
        
        vortexPos = self.getVortexPosInTile(quadrant)
        return [nodePos[0] + vortexPos[0], nodePos[1] + vortexPos[1]]
        #return nodePos
    
    def getBestPoses(self):
        bestPoses = []
        for bestRawNode in self.__bestPath:
            node, quadrant = self.grid.rawToProcessedNode(bestRawNode)
        
            nodePos = self.getTilePos(node)
        
            vortexPos = self.getVortexPosInTile(quadrant)
            print("Vortex pos: ", vortexPos)
            #return 
            bestPoses.append([nodePos[0] + vortexPos[0], nodePos[1] + vortexPos[1]])
        return bestPoses

    def getGrid(self):
        return self.grid.grid
    
    def showGrid(self):
        cv.imshow("Analyst grid", cv.resize(self.grid.getNumpyPrintableArray(), (400, 400), interpolation=cv.INTER_NEAREST))


# ------------------- State machines ------------

# Manages states
class StateManager:
    def __init__(self, initialState):
        self.state = initialState

    # Sets the state to a certain value
    def changeState(self, newState):
        self.state = newState
        return True

    # Checks if the state corresponds to a specific value
    def checkState(self, state):
        return self.state == state

# Makes it possible to run arbitrary code sequentially without interrupting other code that must run continuoulsy
class SequenceManager:
    def __init__(self):
        self.lineIdentifier = 0
        self.linePointer = 1
        self.done = False

    # Resets the sequence and makes it start from the first event
    def resetSequence(self):
        self.linePointer = 1
        print("----------------")
        print("reseting sequence")
        print("----------------")

    def seqResetSequence(self):
        if self.check():
            self.resetSequence()
            
            return True
        return False

    # This has to be at the start of any sequence of events
    def startSequence(self):
        self.lineIdentifier = 0
        self.done = False

    # Returns if the line pointer and identifier match and increases the identifier
    # Must be included at the end of any sequential function
    def check(self):
        self.done = False
        self.lineIdentifier += 1
        return self.lineIdentifier == self.linePointer

    # Changes to the next event
    def nextSeq(self):
        self.linePointer += 1
        self.done = True

    # returns if the sequence has reached its end
    def seqDone(self):
        return self.done

    # Can be used to make a function sequential or used in an if statement to make a code block sequential
    def simpleSeqEvent(self, function=None, *args, **kwargs):
        if self.check():
            if function is not None:
                function(*args, **kwargs)
            self.nextSeq()
            return True
        return False

    # The function inputted must return True when it ends
    def complexSeqEvent(self, function, *args, **kwargs):
        if self.check():
            if function(*args, **kwargs):
                self.nextSeq()
                return True
        return False
    
    # When inpuuted any function it returns a sequential version of it that can be used in a sequence
    def makeSimpleSeqEvent(self, function):
        def event(*args, **kwargs):
            if self.check():
                function(*args, **kwargs)
                self.nextSeq()
                return True
            return False
        return event

    # When inputted a function that returns True when it ends returns a sequential version of it that can be used in a sequence
    def makeComplexSeqEvent(self, function):
        def event(*args, **kwargs):
            if self.check():
                if function(*args, **kwargs):
                    self.nextSeq()
                    return True
            return False
        return event


# ----------------------- Abstraction Layer -----------

class PlottingArray:
    def __init__(self, size, offsets, scale, tileSize):
        self.scale = scale
        self.size = size
        self.offsets = offsets
        self.scale = scale
        self.tileSize = tileSize
        self.gridPlottingArray = np.zeros(self.size, np.uint8)
        """
        for y in range(0, len(self.gridPlottingArray), int(self.tileSize * scale)):
            for x in range(len(self.gridPlottingArray[0])):
                self.gridPlottingArray[x][y] = 100
        for x in range(0, len(self.gridPlottingArray), int(self.tileSize * scale)):
            for y in range(len(self.gridPlottingArray[0])):
                self.gridPlottingArray[x][y] = 100
        """
    
    def plotPoint(self, point, value):
        procPoint = [int(point[0] * self.scale), int(point[1] * self.scale * -1)]
        finalx = procPoint[0] + int(self.offsets[0] * self.tileSize)
        finaly = procPoint[1] + int(self.offsets[1] * self.tileSize)
                
        if self.size[0] * -1 < finalx < self.size[0] and self.size[0] * -1 < finaly < self.size[1]:
            self.gridPlottingArray[finalx][finaly] = value
    
    def getPoint(self, point):
        procPoint = [int(point[0] * self.scale), int(point[1] * self.scale * -1)]
        finalx = procPoint[0] + int(self.offsets[0] * self.tileSize)
        finaly = procPoint[1] + int(self.offsets[1] * self.tileSize)
                
        if self.size[0] * -1 < finalx < self.size[0] and self.size[0] * -1 < finaly < self.size[1]:
            return self.gridPlottingArray[finalx][finaly]
    
    def reset(self):
        self.gridPlottingArray = np.zeros(self.size, np.uint8)
        """
        for y in range(0, len(self.gridPlottingArray), int(self.tileSize * self.scale)):
            for x in range(len(self.gridPlottingArray[0])):
                self.gridPlottingArray[x][y] = 100
        for x in range(0, len(self.gridPlottingArray), int(self.tileSize * self.scale)):
            for y in range(len(self.gridPlottingArray[0])):
                self.gridPlottingArray[x][y] = 100
        """



class AbstractionLayer():

    def __init__(self):
        #Variables
        self.tileSize = 0.06
        self.timeStep = 32
        self.gridPlotter = PlottingArray((300, 300), [1500, 1500], 150, self.tileSize)
        self.doWallMapping = False

        # Components
        self.robot = RobotLayer(self.timeStep)
        self.seqMg = SequenceManager()
        self.analyst = Analyst(self.tileSize)

        # -- Functions --
        self.seqPrint = self.seqMg.makeSimpleSeqEvent(print)
        self.seqDelaySec = self.seqMg.makeComplexSeqEvent(self.robot.delaySec)
        self.seqMoveWheels = self.seqMg.makeSimpleSeqEvent(self.robot.moveWheels)
        self.seqRotateToDegs = self.seqMg.makeComplexSeqEvent(self.robot.rotateToDegs)
        self.seqMoveToCoords = self.seqMg.makeComplexSeqEvent(self.robot.moveToCoords)

    def calibrate(self):
        self.seqMg.startSequence()
        self.seqDelaySec(0.5)
        if self.seqMg.simpleSeqEvent(): 
            actualTile = [self.position[0] // self.tileSize, self.position[1] // self.tileSize]
            self.robot.positionOffsets =  [round((actualTile[0] * self.tileSize) - self.position[0]) + self.tileSize // 2, round((actualTile[1] * self.tileSize) - self.position[1]) + self.tileSize // 2]
            self.robot.positionOffsets = [self.robot.positionOffsets[0] % self.tileSize, self.robot.positionOffsets[1] % self.tileSize]

            print("positionOffsets: ", self.robot.positionOffsets)
        self.seqDelaySec(0.5)
        
        if self.seqMg.simpleSeqEvent(): self.robot.rotationDetectionType = "gps"
        self.seqMoveWheels(1, 1)
        self.seqDelaySec(0.2)
        if self.seqMg.simpleSeqEvent(): self.robot.rotationDetectionType = "gyroscope"
        self.seqDelaySec(0.2)
        self.seqMoveWheels(0, 0)
        self.seqMoveWheels(-1, -1)
        self.seqDelaySec(0.4)
        self.seqMoveWheels(0, 0)
        if self.seqMg.simpleSeqEvent(): self.doWallMapping = True
        return self.seqMg.seqResetSequence()
    
    @property
    def rotation(self):
        return self.robot.rotation
    
    @property
    def position(self):
        return self.robot.globalPosition

    
    def getBestPos(self):
        return self.analyst.getBestPosToMove()
    
    def doLoop(self):
        return self.robot.doLoop()
    


    def update(self):
        self.robot.update()
        
        if self.doWallMapping:
            print("Doing wall mapping")
            pointCloud = self.robot.getDetectionPointCloud()
            
            """
            for point in pointCloud:
                
                if self.gridPlotter.getPoint(point) < 250:
                    self.gridPlotter.plotPoint(point, self.gridPlotter.getPoint(point) + 5)
            """
            #tileType = self.robot.get
            self.analyst.loadPointCloud(pointCloud)
            #self.analyst.loadColorDetection(self.position, "hole")
            self.analyst.update(self.position)

            
            self.gridPlotter.reset()
            for point in self.analyst.converter.totalPointCloud:
                if point[2] > 30:
                    ppoint = [point[0] / 100, point[1] / 100]
                    self.gridPlotter.plotPoint(ppoint, 100)
            
            bestPos = self.analyst.getStartRawNodePos()
            if bestPos is not None:
                self.gridPlotter.plotPoint(bestPos, 255)
            
            bestPos = self.analyst.getBestPosToMove()
            if bestPos is not None:
                self.gridPlotter.plotPoint(bestPos, 200)
            
        
        #self.gridPlotter.plotPoint(self.position, 150)

        
        bestPoses = self.analyst.getBestPoses()
        for bestPos in bestPoses:
            self.gridPlotter.plotPoint(bestPos, 255)
        
        

        #self.analyst.showGrid()
        
        
        cv.imshow("raw detections", cv.resize(self.gridPlotter.gridPlottingArray, (400, 400), interpolation=cv.INTER_NEAREST))
        cv.waitKey(1)

        print("--------------------------------------------------------------------")


#:::::::::::::::::MAIN PROGRAM::::::::::::::::::

stMg = StateManager("init")
r = AbstractionLayer()

# While the simulation is running
while r.doLoop():
    # Update the robot
    r.update()
    print("rotation: " + str(r.rotation))
    print("position: " + str(r.position))

    if stMg.checkState("init"):
        if r.calibrate():
            stMg.changeState("followBest")
    
    if stMg.checkState("stop"):
        r.seqMg.startSequence()
        r.seqMoveWheels(0, 0)
    
    if stMg.checkState("followBest"):
        r.seqMg.startSequence()
        bestPos = r.getBestPos()
        if bestPos is not None:
            r.seqMoveToCoords(bestPos)
        r.seqMg.seqResetSequence()


    if stMg.checkState("main"):
        
        r.seqMg.startSequence()
        #print(r.seqMoveToCoords((-0.233, -0.36)))
        #r.seqMoveWheels(0.2, -0.2)
        #r.seqRotateToDegs(90)
        r.seqMoveToCoords([-0.48, -0.48])
        r.seqMoveWheels(0, 0)
        r.seqMoveToCoords([-0.48, 0.3])
        r.seqMoveWheels(0, 0)
        r.seqMg.seqResetSequence()