from controller import Robot
import cv2 as cv
import numpy as np
import struct
import math

# Global time step
timeStep = 16 * 1

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

def isInRange(val, minVal, maxVal):
    return minVal < val < maxVal



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

# aStarNode grid class for mapping
class NodeGrid:
    # Creates grid and prepares it
    def __init__(self, x, y, tileSize, nodeTypeDict, offsets=[0,0]):
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

        self.nodeColors = nodeTypeDict
        self.colorNames = {}
        for item in self.nodeColors.items():
            self.colorNames[item[1]] = item[0]
        #print(self.nodeColors)
        #print(self.colorNames)

        for i in range(0, len(self.grid)):
            if i % 2 != 0:
                for j in range(0, len(self.grid[i])):
                    if j % 2 != 0:
                        self.grid[i][j] = self.nodeColors["occupied"]

        # Empty walls
        """
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
        """
    # A Star algorithm
    # Returns a list of tuples as a path from the given start to the given end in the given maze
    def astar(self, start, end):
        # Create start and end node
        start_node = aStarNode(None, (start[0], start[1]))
        start_node.g = start_node.h = start_node.f = 0
        end_node = aStarNode(None, (end[0], end[1]))
        end_node.g = end_node.h = end_node.f = 0
        # Initialize open and closed list
        open_list = []
        closed_list = []
        # Add the start node
        open_list.append(start_node)
        # Loop until end
        while len(open_list) > 0:
            # Get the current node
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index
            # Pop current off open list, add to closed list
            open_list.pop(current_index)
            closed_list.append(current_node)
            # If found the goal
            if current_node == end_node:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                return path[::-1]  # Return reversed path
            # Generate children
            children = []
            for new_orientation in ("up", "down", "right", "left"):  # Adjacent squares

                new_position = self.orientations[new_orientation]
                # Get node position
                node_position = (current_node.position[0] + new_position[0] * 2, current_node.position[1] + new_position[1] * 2)

                """
                # Make sure within range
                if node_position[0] > (len(self.grid) - 1) or node_position[0] < 0 or node_position[1] > (
                        len(self.grid[len(self.grid) - 1]) - 1) or node_position[1] < 0:
                    continue
                """
                
                # Make sure walkable terrain
                
                if self.getValue(node_position) == "occupied" or self.getValue(node_position) == "undefined":
                    continue
                if self.getValue(current_node.position, new_orientation) == "occupied":
                    continue
                
                
                
                # Create new node
                new_node = aStarNode(current_node, node_position)
                # Append
                children.append(new_node)
            # Loop through children
            for child in children:
                continueLoop = False
                # Child is on the closed list
                for closed_child in closed_list:
                    if child == closed_child:
                        continueLoop = True
                        break
                # Create the f, g, and h values
                child.g = current_node.g + 1
                child.h = ((child.position[0] - end_node.position[0]) ** 2) + (
                            (child.position[1] - end_node.position[1]) ** 2)
                child.f = child.g + child.h
                # Child is already in the open list
                for open_node in open_list:
                    if child == open_node and child.g > open_node.g:
                        continueLoop = True
                        break
                if continueLoop:
                    continue
                # Add the child to the open list
                open_list.append(child)
    # Breath First Search algorithm
    # Returns the tiles with the color given in objectives in order and with the distance of each one
    def bfs(self, start, objectives, limit="undefined"):
        visited = []
        queue = []
        found = []
        start = [start[0], start[1], 0]
        visited.append(start)
        queue.append(start)
        for i in objectives:
            found.append(list())
        while queue:
            coords = queue.pop(0)
            x = coords[0]
            y = coords[1]
            dist = coords[2]
            if limit != "undefined":
                if dist > limit:
                    break
            for index, objective in enumerate(objectives):
                if self.getValue([x,y]) == objective:
                    found[index].append(coords)
            for newOrientation in ("up", "down", "right", "left"):
                newPosition = self.orientations[newOrientation]
                neighbour = [x + newPosition[0] * 2, y + newPosition[1] * 2, dist + 1]
                
                inList = False
                for node in visited:
                    if node[0] == neighbour[0] and node[1] == neighbour[1]:
                        inList = True
                        break
                if inList:
                    continue

                # Make sure walkable terrain
                if self.getValue([x,y], newOrientation) == "occupied" \
                    or self.getValue([x,y], newOrientation) == "undefined":
                        continue
                if self.getValue([neighbour[0], neighbour[1]]) == "occupied" \
                    or self.getValue([neighbour[0], neighbour[1]]) == "undefined":
                        continue
                visited.append(neighbour)
                queue.append(neighbour)
        return found
    # Prints the grid in the console
    def printMap(self):
        print(self.grid)

    # Returns the grid
    def getMap(self):
        return self.grid

    def setPosition(self, position, val, orientation="center"):
        tile = self.getTileNode(position)
        return self.changeValue(tile, val, orientation)
    
    def getPosition(self, position, orientation="center"):
        tile = self.getTileNode(position)
        return self.getValue(tile, orientation)


    def getTileNode(self, pos):
        node = [int(((pos[1] + self.offsets[1]) // self.tileSize) * 2), int(((pos[0] + self.offsets[0]) // self.tileSize) * 2)]
        return node
    
    def getPosfromTileNode(self, tileNode):
        return [tileNode[0] // 2 * self.tileSize, tileNode[1] // 2 * self.tileSize]

    # Changes the value of a given node in the grid
    def changeValue(self, pos, val, orientation="center"):
        # print(pos)
        if val in self.nodeColors.keys():
            val = self.nodeColors[val]
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
            finalIndex = [pos[0] + self.center[0], pos[1] + self.center[1]]
            finalIndex[0] = finalIndex[0] + self.orientations[orientation][0]
            finalIndex[1] = finalIndex[1] + self.orientations[orientation][1]

            if finalIndex[0] < 0 or finalIndex[1] < 0:
                return "undefined"
            else:
                node = self.grid[int(finalIndex[0]), int(finalIndex[1])]
                if node in self.colorNames.keys():
                    node = self.colorNames[node]
                return node
                
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
        #print("input pos: " + str(inputPos))
        #print("tile: " + str(posTile))
        #print("tile pos: " + str(tilePos))
        #print("pos in tile: " + str(posInTile))
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
        #print(orientation)
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
    def __init__(self, sensor, sensorAngle, robotDiameter, tileSize, timeStep, detectionLimit=1):
        self.sensor = sensor
        self.sensor.enable(timeStep)
        self.offset = 1
        self.robotDiameter = robotDiameter
        self.angle = sensorAngle
        self.tileSize = tileSize
        self.maxDetect = 0.8
        self.detectionLimit = detectionLimit
    # Gets the distance from the sensor value
    def getDistance(self):
        val = self.sensor.getValue()
        if val < self.maxDetect * self.detectionLimit:
            dist = mapVals(val, 0, self.maxDetect, 0, self.tileSize * 2.4)
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
    def __init__(self, sensor, distancefromCenter, timeStep):
        self.distance = distancefromCenter
        self.sensor = sensor
        self.sensor.enable(timeStep)
        self.r = 0
        self.g = 0
        self.b = 0
    
    def getPosition(self, robotGlobalPosition, robotGlobalRotation):
        relPosition = getCoords(robotGlobalRotation, self.distance)
        return [robotGlobalPosition[0] + relPosition[0], robotGlobalPosition[1] + relPosition[1]]
    
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
        self.__update()
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
    def __init__(self, camera, tileRanges, timeStep):
        self.camera = camera
        self.camera.enable(timeStep)
        self.height = self.camera.getHeight()
        self.width = self.camera.getWidth()
        self.tileRanges = tileRanges

    # Gets an image from the raw camera data
    # Obtiene una imagen tipo numpy array de la data original de la camara que esta en bytes
    def getImg(self):
        imageData = self.camera.getImage()
        return np.array(np.frombuffer(imageData, np.uint8).reshape((self.height, self.width, 4)))

    #Obtiene los cuadraditos de cada victima en camara y sus posiciones(Las de sus esquinas superiores izquierdas)
    def getVictimImagesAndPositions(self):
        img = self.getImg()
        # Hace una copia de la imagen
        img1 = img.copy()
        # Filtra la copia para aislar su elemento azul
        img1[:, :, 2] = np.zeros([img1.shape[0], img1.shape[1]])
        # Hace una version es escala de grises
        gray = cv.cvtColor(img1, cv.COLOR_BGR2GRAY)
        # Hace un thershold para hacer la imagen binaria
        thresh = cv.threshold(gray, 140, 255, cv.THRESH_BINARY)[1]
        #cv.imshow("thresh", thresh)
        # Encuentra los contornos, aunque se puede confundir con el contorno de la letra
        contours, _ = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        # Pra evitar la confusion dibuja rectangulos blancos donde estan los contornos en la imagen y despues vuelve a
        # sacar los contornos para obtener solo los del rectangulo, no los de las letras.
        for c0 in contours:
            x, y, w, h = cv.boundingRect(c0)
            cv.rectangle(thresh, (x, y), (x + w, y + h), (225, 255, 255), -1)
        #cv.imshow("thresh2", thresh)
        contours, _ = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        # saca las medidas y la posicion de los contornos y agrega a la lista de imagenes la parte esa de la imagen original
        # Tambien anade la posicion de cada recuadro en la imagen original
        finalPoses = []
        finalImages = []
        for c in contours:
            x, y, w, h = cv.boundingRect(c)
            finalImages.append(img[y:y + h, x:x + w])
            finalPoses.append((y, x))
        return finalPoses, finalImages

    # Requiere de argumentos provenientes del metodo getImagesAndPositions
    def isVictimInRange(self, pos, img):
        # Chequea si la victima esta en rango para ser clasificada y la cantidad de casillas hasta el robot
        status = "undefined"
        # Valores obtenidos probandolos de acuerdo a la cercania o tamaño de la victima 
        # en camara necesaria para su clasificacion
        heightThreshold = (50, 105)
        minRatio = (0.8)
        if img.shape[0] != 0:
            ratio = img.shape[1] / img.shape[0]
        else:
            ratio = 0
        # Determina la distancia de la victima hasta la camara teniendo en cuenta los thresholds de la camara
        for index, ranges in enumerate(self.tileRanges):
            if isInRange(img.shape[0], ranges[0], ranges[1]):
                status = index
            if ratio < minRatio and status == 0:
                status = "undefined"
        #print("Ratio: " + str(ratio))
        #print("Shape: " + str(img.shape))
        #print("Status: " + str(status))
        return status
                


# Sends messages
class Emitter:
    def __init__(self, emmitter, coordsDivisor=0):
        self.emitter = emmitter
        self.divisor = coordsDivisor
    # Sends a message given a position and identifier
    def sendMessage(self,pos, identifier):
        message = struct.pack('i i c', int(pos[0] / self.divisor * 100), int(pos[1] / self.divisor * 100), identifier.encode())
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
    def __init__(self, timeStep, posMultiplier, maxVelocity, robotDiameter, tileSize, distSensorLimit=1):
        #Instantiations
        self.robot = Robot()
        # Constants
        self.posMultiplier = posMultiplier
        self.timeStep = timeStep
        self.maxVelocity = maxVelocity
        self.robotDiameter = robotDiameter
        self.tileSize = tileSize
        colourSensorOffset = 1
        # Variables
        # Wheels
        self.leftWheel = Wheel(self.robot.getMotor("left wheel motor"), self.maxVelocity)
        self.rightWheel = Wheel(self.robot.getMotor("right wheel motor"), self.maxVelocity)
        #Cameras
        self.centreCamera = Camera(self.robot.getCamera("camera_centre"), [(50, 105), ], self.timeStep)
        self.rightCamera = Camera(self.robot.getCamera("camera_right"), [], self.timeStep)
        self.leftCamera = Camera(self.robot.getCamera("camera_left"), [], self.timeStep)
        #Colour sensor
        
        self.colourSensor = ColourSensor(self.robot.getCamera("colour_sensor"), self.robotDiameter / 2 + colourSensorOffset, self.timeStep)
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
        self.distSensors.append(DistanceSensor(self.robot.getDistanceSensor("ps0"), 72.76564, self.robotDiameter, self.tileSize, self.timeStep, distSensorLimit))
        self.distSensors.append(DistanceSensor(self.robot.getDistanceSensor("ps1"), 44.11775, self.robotDiameter, self.tileSize, self.timeStep, distSensorLimit))
        self.distSensors.append(DistanceSensor(self.robot.getDistanceSensor("ps2"), 0, self.robotDiameter, self.tileSize, self.timeStep, distSensorLimit))
        self.distSensors.append(DistanceSensor(self.robot.getDistanceSensor("ps3"), 298.511, self.robotDiameter, self.tileSize, self.timeStep, distSensorLimit))
        self.distSensors.append(DistanceSensor(self.robot.getDistanceSensor("ps4"), 241.2152, self.robotDiameter, self.tileSize, self.timeStep, distSensorLimit))
        self.distSensors.append(DistanceSensor(self.robot.getDistanceSensor("ps5"), 180, self.robotDiameter, self.tileSize, self.timeStep, distSensorLimit))
        self.distSensors.append(DistanceSensor(self.robot.getDistanceSensor("ps6"), 135.791, self.robotDiameter, self.tileSize, self.timeStep, distSensorLimit))
        self.distSensors.append(DistanceSensor(self.robot.getDistanceSensor("ps7"), 107.1431, self.robotDiameter, self.tileSize, self.timeStep, distSensorLimit))
        

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
        self.distSensorLimit = 0.6
        self.nodeTypes = {
            "occupied":255,
            "unknown":0,
            "unoccupied":30,
            "uncollectedVictim":100,
            "collectedVictim":170,
            "checkpoint":50
        }
        #Instantiations
        self.robot = RobotLayer(timeStep, self.posMultiplier, self.maxVelocity, self.robotDiameter, self.tileSize, self.distSensorLimit)
        self.seqMg = SequenceManager()
        self.stMg = StateManager(initialState)
        self.grid = NodeGrid(40, 40, self.tileSize, self.nodeTypes,[self.tileSize / 2, self.tileSize / 2])
        

        # Variables for abstraction layer
        
        self.actualTime = self.robot.getTime()
        self.delayStart = 0.0
        self.delayFirstTime = True
        self.seqMoveDistStart = [0, 0]
        self.seqMoveDistFirstTime = True
        self.globalPos = self.prevGlobalPos = [0,0]
        self.globalRot = 0
        self.rotDetectMethod = "velocity"
        self.colourTileType = "undefined"
        self.diffInPos = 0
        self.firstStep = True
        self.doWallMap = self.doTileMap = False
        self.seqRotateToDegsFirstTime = True
        self.seqRotateToDegsInitialRot = 0
        self.followPathIndex = 0
        self.calculatedPath = []
        self.do360FirstTime = True
        self.do360OriginAngle = 0
        self.closestReachableNodeTile = [0,0]
        self.cameraData = {"centre":{"images":[], "poses":[]}, 
                            "right":{"images":[], "poses":[]}, 
                            "left":{"images":[], "poses":[]}}

    def doWallMapping(self):
        mapped = False
        for sensor in self.robot.distSensors:
            detection = sensor.getGlobalDetection(self.globalRot, self.globalPos)
            if detection != -1:
                orientation = self.grid.getOrientationInTile(detection)
                if orientation != "undefined":
                        if self.grid.getPosition(detection, orientation) not in ("occupied", ):
                            self.grid.setPosition(detection, "occupied", orientation)
                            mapped = True
        return mapped

    def doTileMapping(self):
        #print(self.grid.getPosition(self.globalPos))
        if self.grid.getPosition(self.globalPos) in ("unknown",):
            self.grid.setPosition(self.globalPos, "unoccupied")
        if self.colourTileType == "trap":
            self.grid.setPosition(self.robot.colourSensor.getPosition(self.globalPos, self.globalRot), "occupied")
            return True
        return False
    
    def showGrid(self):
        cv.imshow("ventana", cv.resize(self.grid.getMap(), (400, 400), interpolation=cv.INTER_AREA))     
    
    def sendMessage(self, indentifier):
        self.robot.emitter.sendMessage(self.globalPos, indentifier)
    
    def seqFollowCalculatedPath(self):
        self.seqFollowPath(self.calculatedPath)

    def calculatePath(self):
        #print(" CALCULATING ")
        start = self.grid.getTileNode(self.globalPos)
        bfsResults = self.grid.bfs(start, ("unknown", ), 10)
        unknownResults = bfsResults[0]
        print("BFS Results: " + str(unknownResults))
        priorClosestReachable = False
        for result in unknownResults:
            if result[0] == self.closestReachableNodeTile[0] and result[1] == self.closestReachableNodeTile[1]:
                priorClosestReachable = True
                break
        if not priorClosestReachable:
            self.closestReachableNodeTile = unknownResults[0]
        print("closest: " + str(self.grid.getPosfromTileNode(self.closestReachableNodeTile)))
        path = self.grid.astar(start, self.closestReachableNodeTile)
        self.calculatedPath = []
        for node in path:
            self.calculatedPath.append(self.grid.getPosfromTileNode([node[1], node[0]]))
        self.calculatedPath.pop(0)
        self.followPathIndex = 0
        print("Path: " + str(self.calculatedPath))
        """
        end = self.grid.getTileNode([-72, 0])
        path = self.grid.astar(start, end)
        for node in path:
            self.grid.changeValue(node, 110)
        """

    def seqDo360(self, direction="right"):
        if self.seqEvent():
            if self.do360FirstTime:
                self.do360OriginAngle = self.globalRot
                self.do360FirstTime = False
        if direction == "right":
            self.seqMove(7, -7)
            self.seqDelaySec(0.2)
            if self.seqRotateToDegs(self.do360OriginAngle, "farthest"):
                self.do360FirstTime = True
        else:
            self.seqMove(-7, 7)
            self.seqDelaySec(0.2)
            if self.seqRotateToDegs(self.do360OriginAngle, "farthest"):
                self.do360FirstTime = True

    
    def seqFollowPath(self, path):
        if self.seqMg.check():
            
            if self.followPathIndex == len(path):
                print("finished")
                self.seqMg.nextSeq()
                self.followPathIndex = 0
            elif self.moveToCoords(path[self.followPathIndex]):
                print("moved!")
                self.followPathIndex += 1
        else:
            self.followPathIndex = 0
        return self.seqMg.seqDone()


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

    def rotateToDegs(self, degs, orientation="closest"):
        accuracy = 1
        if self.seqRotateToDegsFirstTime:
            self.seqRotateToDegsInitialRot = self.globalRot
            self.seqRotateToDegsFirstTime = False
        diff = self.globalRot - degs
        initialDiff = self.seqRotateToDegsInitialRot - degs
        moveDiff = max(round(self.globalRot), degs) - min(self.globalRot, degs)
        if diff > 180 or diff < -180:
            moveDiff = 360 - moveDiff
        speedFract = min(mapVals(moveDiff, 0, 90, 0.2, 1), 0.7)
        if accuracy  * -1 < diff < accuracy or 360 + (accuracy  * -1) < diff < 360 + accuracy:
            self.seqRotateToDegsFirstTime = True
            return True
        else:
            if orientation == "closest":
                if 180 > initialDiff > 0 or initialDiff < -180:
                    direction = "right"
                else:
                    direction = "left"
            elif orientation == "farthest":
                if 180 > initialDiff > 0 or initialDiff < -180:
                    direction = "left"
                else:
                    direction = "right"
            else:
                direction = orientation
            if direction == "right":
                self.robot.move(speedFract * -1, speedFract)
            elif direction == "left":
                self.robot.move(speedFract, speedFract * -1)
        return False

    def seqRotateToDegs(self, degs, orientation="closest"):
        if self.seqMg.check():
            if self.rotateToDegs(degs, orientation):
                self.robot.move(0,0)
                self.seqMg.nextSeq()
        return self.seqMg.seqDone()

    def moveToCoords(self, targetPos):
        errorMargin = 0.1
        diffX = targetPos[0] - self.globalPos[0]
        diffY = targetPos[1] - self.globalPos[1]
        #print("diff in pos: " + str(diffX) + " , " + str(diffY))
        dist = math.sqrt(diffX ** 2 + diffY ** 2)
        #print("Dist: "+ str(dist))
        if errorMargin * -1 < dist < errorMargin:
            self.robot.move(0,0)
            return True
        else:
            rad = math.atan2(diffX, diffY)
            ang = rad * 180 / math.pi
            ang = normalizeAngle(ang)
            #print("traget ang: " + str(ang))
            ratio = min(mapVals(dist, 0, self.tileSize, 0.1, 1), 0.8)
            ratio = max(ratio, 0.2)
            if self.rotateToDegs(ang):
                self.robot.move(ratio, ratio)
        return False

    def seqMoveToCoords(self, targetPos):
        if self.seqMg.check():
            if self.moveToCoords(targetPos):
                self.seqMg.nextSeq()
        return self.seqMg.seqDone()

    # Poner antes de empezar una sequencia o de usar una funcion sequencial
    # Put before starting a sequence or using a sequencial function
    def startSequence(self):
        self.seqMg.startSequence()
    
    def changeState(self, newState):
        self.stMg.changeState(newState)
        self.seqMg.resetSequence()
        self.followPathIndex = 0
        self.do360FirstTime = True
    
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

        temp = self.robot.centreCamera.getVictimImagesAndPositions()
        self.cameraData["centre"]["poses"] =  temp[0]
        self.cameraData["centre"]["images"] =  temp[1]
        temp = self.robot.rightCamera.getVictimImagesAndPositions()
        self.cameraData["right"]["poses"] =  temp[0]
        self.cameraData["right"]["images"] =  temp[1]
        temp = self.robot.leftCamera.getVictimImagesAndPositions()
        self.cameraData["left"]["poses"] =  temp[0]
        self.cameraData["left"]["images"] =  temp[1]
        #self.showGrid()
    
    def bottomUpdate(self):
        # Bottom updates
        self.prevGlobalPos = self.globalPos
        newTiles = False
        if self.doTileMap:
            newTiles = self.doTileMapping()
        cv.waitKey(1)


# Instanciacion de capa de abstracción
# Abstraction layer instantiation
r = AbstractionLayer(timeStep, "start")

#MAIN PROGRAM
# Updates the global position, rotation, colorSensor position and colors, shows the grid and does mapping
while r.update():
    # v Program v

    # Start state
    if r.isState("start"):
        r.startSequence()
        if r.seqEvent():
            r.doWallMap = False
            r.doTileMap = False
        if r.seqEvent():
            r.rotDetectMethod = "position"
        if r.seqMoveDist(0.8, r.tileSize / 2):
            r.rotDetectMethod = "velocity"
        r.seqMoveDist(-0.8, r.tileSize / 2)
        if r.seqEvent():
            r.doWallMap = True
            r.doTileMap = True
        r.seqDo360()
        if r.seqEvent():
            r.calculatePath()
            r.changeState("main")
        
    #-------Nuevo Codigo aqui-----
    # Main state
    elif r.isState("main"):
        # Para crear nuevas funciones ir a la clase de camara en linea 438
        # Se deben escribir para correrlas como r.robot.cameraCentre/Right/Left.nombreDeFuncion
        #Las funciones y el dccionario que se usan aca para guerdar los datos y sacar 
        # si la victima esta en rango son de la clase AbstractionLayer para simplificar algunas cosas:

        #areVictimsInRange esta definida dentro de AbstractionLayer en linea 887, para funcionar utiliza 
        # la funcion de la clase Camera isVictimInRange.

        #El diccionario de cameraData se incializa en la linea 662, tiene como elementos el nombre 
        # de las 3 camaras (centre, right y left), estas a su vez son un diccionario con los elementos 
        # de images y poses. De esta manera se puede acceder a los datos de estas camaras con dos indices: 
        # r.cameraData[nombreDeCamara][Datos],  como se ve en el codigo de ejemplo

        # Estos datos se obtienen desde la linea 926  hasta 934 usando las funciones de la camara en la 
        # funcion de top update, que ocurre cada ciclo.
        #las "images" son las imagenes individuales de cada victima en la camara (cada cuadradito blanco con la letra)
        #las "poses" son las posiciones de la punta de arriba a la izquierda de los cuadrados de las victimas 
        #en la imagen de la camara

        #Elresto esta comentado en la clase camara

        imgsCamaraCentro = r.cameraData["centre"]["images"]
        posesEnImagenCamaraCentro = r.cameraData["centre"]["poses"]
        for pos, img in zip(r.cameraData["centre"]["poses"], r.cameraData[camera]["images"]):
                if self.robot.leftCamera.isVictimInRange(pos, img) == 0:
                    isInRange = True

        imgsCamaraDerecha = r.cameraData["right"]["images"]
        posesEnImagenCamaraDerecha = r.cameraData["right"]["poses"]
        victimasEnRangoCamaraDerecha = r.areVictimsInRange("right")

        imgsCamaraIzquierda = r.cameraData["left"]["images"]
        posesEnImagenCamaraIzquierda = r.cameraData["left"]["poses"]
        victimasEnRangoCamaraIzquierda = r.areVictimsInRange("left")

        for index, img in enumerate(imgsCamaraCentro):
            cv.imshow("victima" + str(index), img)
            print("escala de imagenes del centro: " + str(imgsCamaraCentro[index].shape))

            #Para gurdar imagenes
            #directorio = r"C:/..."
            #cv.imwrite(str(directorio) + "/victima" + str(index) + ".png", img)
        print("Posicion de victimas en imagen de camara del centro: " + str(posesEnImagenCamaraCentro))
        print("Hay victimas en rango en el centro: " + str(victimasEnRangoCamaraCentro))
        