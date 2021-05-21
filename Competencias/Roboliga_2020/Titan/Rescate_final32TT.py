from controller import Robot
import cv2 as cv
import numpy as np
import struct
import math

# Version de python 3.7.7 64-bit
# Librerias necesarias:
# Externas:
# numpy
# opencv
# De Python:
# struct
# math
# SIN API de deteccion (NO DETECTION API)


# Global time step
timeStep = 16 * 2

# Corrects the given angle to be in a range from 0 to 360
def normalizeAngle(ang):
    ang = ang % 360
    if ang < 0:
        ang += 360
    if ang == 360:
        ang = 0
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
            "centre":[0,0]
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

    def setPosition(self, position, val, orientation="centre"):
        tile = self.getTileNode(position)
        return self.changeValue(tile, val, orientation)
    
    def getPosition(self, position, orientation="centre"):
        tile = self.getTileNode(position)
        return self.getValue(tile, orientation)


    def getTileNode(self, pos):
        node = [int(((pos[1] + self.offsets[1]) // self.tileSize) * 2), int(((pos[0] + self.offsets[0]) // self.tileSize) * 2)]
        return node
    
    def getPosfromTileNode(self, tileNode):
        return [(tileNode[0] // 2 * self.tileSize) + (self.tileSize // 2 - self.offsets[0]), (tileNode[1] // 2 * self.tileSize) + (self.tileSize // 2 - self.offsets[1])]

    # Changes the value of a given node in the grid
    def changeValue(self, pos, val, orientation="centre"):
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
    def getValue(self, pos, orientation="centre"):
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
        thicknessClearance = self.tileSize / 12
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
            orientation = "centre"
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
        if ratio > 1:
            ratio = 1
        self.wheel.setVelocity(ratio * self.maxVelocity)

# Manages a distance sensor
class DistanceSensor:
    def __init__(self, sensor, sensorAngle, robotDiameter, tileSize, timeStep, detectionLimit=1):
        self.sensor = sensor
        self.sensor.enable(timeStep)
        self.offset = 0.8
        self.robotDiameter = robotDiameter
        self.angle = sensorAngle
        self.tileSize = tileSize
        self.maxDetect = 0.8
        self.detectionLimit = detectionLimit
    # Gets the distance from the sensor value
    def getDistance(self):
        val = self.sensor.getValue()
        if val < self.maxDetect * self.detectionLimit:
            dist = mapVals(val, 0, self.maxDetect, 0, self.tileSize * 2.7)
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
    def __init__(self, gyro, index, timeStep):
        self.sensor = gyro
        self.sensor.enable(timeStep)
        self.oldTime = 0.0
        self.index = index
    # Do on every timestep
    def update(self, time, currentRotation):
        #print("Gyro Vals: " + str(self.sensor.getValues()))
        timeElapsed = time - self.oldTime  # Time passed in time step
        radsInTimestep = (self.sensor.getValues())[self.index] * timeElapsed
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

        #print("Color: " + tileType)
        #print("r: " + str(self.r) + "g: " + str(self.g) + "b: " +  str(self.b))
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
        self.classifyThresh = 20

    # Gets an image from the raw camera data
    def getImg(self):
        imageData = self.camera.getImage()
        return np.array(np.frombuffer(imageData, np.uint8).reshape((self.height, self.width, 4)))

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
    def getVictimRange(self, pos, img):
        # Chequea si la victima esta en rango para ser clasificada y la cantidad de casillas hasta el robot
        status = "undefined"
        # Valores obtenidos probandolos de acuerdo a la cercania o tamaño de la victima 
        # en camara necesaria para su clasificacion
        ignoreThresholds = [20, 20]
        minRatio = (0.7)
        if img.shape[0] != 0:
            ratio = img.shape[1] / img.shape[0]
        else:
            ratio = 0
        # Determina la distancia de la victima hasta la camara teniendo en cuenta los thresholds de la camara
        for index, ranges in enumerate(self.tileRanges):
            #print(isInRange(pos[0], 0 + ignoreThresholds[0], self.height - ignoreThresholds[1]))
            try:
                if index != 0:
                    if isInRange(img.shape[0], ranges[0], ranges[1]) and isInRange(pos[0], 0 + ignoreThresholds[0], self.height - ignoreThresholds[1]):
                        status = index
                else:
                    if isInRange(img.shape[0], ranges[0], ranges[1]) and ratio > minRatio:
                        status = index
            except:
                pass
        #print("Ratio: " + str(ratio))
        #print("Shape: " + str(img.shape))
        #print("Status: " + str(status))
        return status

    def classifyVictim(self, img):
        img = cv.resize(img, (100, 100))
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        threshVal1 = 25
        threshVal2 = 100
        thresh1 = cv.threshold(gray, threshVal1, 255, cv.THRESH_BINARY_INV)[1]
        thresh2 = cv.threshold(gray, threshVal2, 255, cv.THRESH_BINARY_INV)[1]
        #conts, h = cv.findContours(thresh1, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        white = 255
        #print(conts)
        maxX = 0
        maxY = 0
        minX = thresh1.shape[0]
        minY = thresh1.shape[1]
        for yIndex, row in enumerate(thresh1):
            for xIndex, pixel in enumerate(row):
                if pixel == white:
                    maxX = max(maxX, xIndex)
                    maxY = max(maxY, yIndex)
                    minX = min(minX, xIndex)
                    minY = min(minY, yIndex)

        letter = thresh2[minY:maxY, minX:maxX]
        letter = cv.resize(letter, (100, 100), interpolation=cv.INTER_AREA)
        #cv.imshow("letra", letter)
        #cv.imshow("thresh", thresh1)
        #letterColor = cv.cvtColor(letter, cv.COLOR_GRAY2BGR)
        areaWidth = 20
        areaHeight = 30
        areas = {
            "top": ((0, areaHeight),(50 - areaWidth // 2, 50 + areaWidth // 2)),
            "middle": ((50 - areaHeight // 2, 50 + areaHeight // 2), (50 - areaWidth // 2, 50 + areaWidth // 2)),
            "bottom": ((100 - areaHeight, 100), (50 - areaWidth // 2, 50 + areaWidth // 2 ))
            }
        images = {
            "top": letter[areas["top"][0][0]:areas["top"][0][1], areas["top"][1][0]:areas["top"][1][1]],
            "middle": letter[areas["middle"][0][0]:areas["middle"][0][1], areas["middle"][1][0]:areas["middle"][1][1]],
            "bottom": letter[areas["bottom"][0][0]:areas["bottom"][0][1], areas["bottom"][1][0]:areas["bottom"][1][1]]
            }
        #cv.rectangle(letterColor,(areas["top"][1][0], areas["top"][0][0]), (areas["top"][1][1], areas["top"][0][1]), (0, 255, 0), 1)
        #cv.rectangle(letterColor, (areas["middle"][1][0], areas["middle"][0][0]), (areas["middle"][1][1], areas["middle"][0][1]), (0, 0, 255), 1)
        #cv.rectangle(letterColor,(areas["bottom"][1][0], areas["bottom"][0][0]), (areas["bottom"][1][1], areas["bottom"][0][1]), (225, 0, 255), 1)
        counts = {}
        for key in images.keys():
            count = 0
            for row in images[key]:
                for pixel in row:
                    if pixel == white:
                        count += 1
            counts[key] = count > self.classifyThresh
        letters = {
            "H":{'top': False, 'middle': True, 'bottom': False},
            "S":{'top': True, 'middle': True, 'bottom': True},
            "U":{'top': False, 'middle': False, 'bottom': True}
            }

        finalLetter = "N"
        for letterKey in letters.keys():
            if counts == letters[letterKey]:
                finalLetter = letterKey
                break
        
        #print(counts)
        #print(finalLetter)
        return finalLetter
    
    def getObstacleImagesAndPositions(self):
        img = self.getImg()
        # Hace una copia de la imagen
        img1 = img.copy()
        # Filtra la copia para aislar su elemento azul
        img1[:, :, 2] = np.zeros([img1.shape[0], img1.shape[1]])
        # Hace una version es escala de grises
        gray = cv.cvtColor(img1, cv.COLOR_BGR2GRAY)
        # Hace un thershold para hacer la imagen binaria
        thresh = cv.threshold(gray, 50, 255, cv.THRESH_BINARY)[1]
        cv.imshow("thresh", thresh)
        # Encuentra los contornos
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

# Sends messages
class Emitter:
    def __init__(self, emmitter, coordsDivisor=0):
        self.emitter = emmitter
        self.divisor = coordsDivisor
    # Sends a message given a position and identifier
    def sendMessage(self,pos, identifier):
        print("Sent message " + identifier + " with pos " + str((int(pos[0] / self.divisor * 100), int(pos[1] / self.divisor * 100))))
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
        self.cameras = {
            "centre":Camera(self.robot.getCamera("camera_centre"), ((50, 105), ), self.timeStep),
            "right":Camera(self.robot.getCamera("camera_right"), ("undefied", (13, 32)), self.timeStep),
            "left":Camera(self.robot.getCamera("camera_left"), ("undefied", (13, 32)), self.timeStep)
        }
        
        #Colour sensor
        self.colourSensor = ColourSensor(self.robot.getCamera("colour_sensor"), self.robotDiameter / 2 + colourSensorOffset, self.timeStep * 2)
        #Emitter
        self.emitter = Emitter(self.robot.getEmitter("emitter"), self.posMultiplier)
        #Gps
        self.gps = Gps(self.robot.getGPS("gps"), self.timeStep, self.posMultiplier)
        #Gyro
        gyroSensor = self.robot.getGyro("gyro")
        self.gyro = Gyroscope(gyroSensor, 0, self.timeStep)
        self.rollGyro = Gyroscope(gyroSensor, 1, self.timeStep)
        self.pitchGyro = Gyroscope(gyroSensor, 2, self.timeStep)
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
        
        self.startTime = self.robot.getTime()
        
    def step(self):
        return self.robot.step(timeStep) != -1

    def getTime(self):
        return self.robot.getTime() - self.startTime
    
    def getRotationByPos(self, prevGlobalPos, globalPos):
        if prevGlobalPos == globalPos:
            return -1
        else:
            posDiff = [(globalPos[0] - prevGlobalPos[0]), (globalPos[1] - prevGlobalPos[1])]

            
            rads = math.atan2(posDiff[0], posDiff[1])
            accuracy = math.sqrt(posDiff[0] ** 2 + posDiff[1] ** 2)
            #print("accuracy: " + str(accuracy))
            if accuracy < 0.1:
                return -1
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
        self.distSensorLimit = 0.5
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
        self.grid = NodeGrid(80, 80, self.tileSize, self.nodeTypes,[0, 0])
        
        # Variables for abstraction layer
        self.actualTimeStep = 0
        self.actualTime = self.robot.getTime()
        self.delayStart = 0.0
        self.delayFirstTime = True
        self.seqMoveDistStart = [0, 0]
        self.seqMoveDistFirstTime = True
        self.ending = False
        self.globalPos = self.prevGlobalPos = [0,0]
        self.actualTile = [0, 0]
        self.actualTileNode = [0, 0]
        self.prevTSTileNode = [0, 0]
        self.prevTileNode = [0, 0]
        self.globalRot = 0
        self.globalRoll = 0
        self.globalPitch = 0
        self.startPos = [0,0]
        self.rotDetectMethod = "velocity"
        self.colourTileType = "undefined"
        self.diffInPos = 0
        self.firstStep = True
        self.doWallMap = self.doTileMap = False
        self.seqRotateToDegsFirstTime = True
        self.seqRotateToDegsInitialRot = 0
        self.seqRotateToDegsinitialDiff = 0
        self.followPathIndex = 0
        self.calculatedPath = []
        self.do360FirstTime = True
        self.do360OriginAngle = 0
        self.closestReachableNodeTile = ["undefined","undefined"]
        self.movedInPath = True
        self.overridePath = False
        self.doCalculatePath = True
        self.doAutoMapCalculating = False
        self.isHot = False
        self.cameras = {"centre":{"images":[], "poses":[], "camera":self.robot.cameras["centre"]}, 
                            "right":{"images":[], "poses":[], "camera":self.robot.cameras["right"]}, 
                            "left":{"images":[], "poses":[], "camera":self.robot.cameras["left"]}}
        self.offsets = [0,0]
        self.timeWithoutMoving = 0
        self.stoppedMovingFT = True
        self.stoppedMovingST = 0
        self.stoppedMovingSP = [0,0]

    def doWallMapping(self):
        mapped = False
        for sensor in self.robot.distSensors:
            detection = sensor.getGlobalDetection(self.globalRot, self.globalPos)
            if detection != -1:
                orientation = self.grid.getOrientationInTile(detection)
                if orientation != "undefined":
                        if self.grid.getPosition(detection, orientation) not in ("occupied", "unoccupied", "collectedVictim"):
                            self.grid.setPosition(detection, "occupied", orientation)
                            mapped = True
        return mapped

    

    def doTileMapping(self):
        mapped = False
        passedOrient = self.getPassedWall()

        if passedOrient != "undefined":
            self.grid.changeValue(self.actualTileNode, "unoccupied", passedOrient)

        if self.grid.getPosition(self.globalPos) in ("unknown", "occupied") and self.isInCenter(2):
            self.grid.setPosition(self.globalPos, "unoccupied")
            mapped = True
        
        alignment = self.getAligment(8)
        if alignment != "undefined" and self.isInCenter(2):
            for camera in self.cameras.keys():
                for img, pos in zip(self.cameras[camera]["images"], self.cameras[camera]["poses"]):
                    victimRange = self.cameras[camera]["camera"].getVictimRange(pos, img)
                    if victimRange != "undefined" and victimRange != 0:
                        possibleVictimTile = []
                        possibleVictimTile.append(self.grid.orientations[alignment][0] * victimRange * 2 + self.grid.getTileNode(self.globalPos)[0])
                        possibleVictimTile.append(self.grid.orientations[alignment][1] * victimRange * 2 + self.grid.getTileNode(self.globalPos)[1])
                        if self.grid.getValue(possibleVictimTile) not in ("occupied", "uncollectedVictim", "collectedVictim"):
                            self.grid.changeValue(possibleVictimTile, "uncollectedVictim")
                            mapped = True
        return mapped

    def doAfterTimesteps(self, nOfSteps):
        if math.ceil((self.actualTimeStep / nOfSteps) % 1):
            return False
        else:
            return True

    def getAligment(self, errorMargin):
        if 90 - errorMargin < self.globalRot < 90 + errorMargin:
            direction = "right"
        elif 180 - errorMargin < self.globalRot < 180 + errorMargin:
            direction = "up"
        elif 270 - errorMargin < self.globalRot < 270 + errorMargin:
            direction = "left"
        elif self.globalRot > 360 - errorMargin or  self.globalRot < 0 + errorMargin:
            direction = "down"
        else:
            direction = "undefined"
        return direction
    
    def isDistanceLessThan(self, pos1, pos2, errorMargin):
        diff = [max(pos1[0], pos2[0]) - min(pos1[0], pos2[0]), max(pos1[1], pos2[1]) - min(pos1[1], pos2[1])]
        distToCenter = getDistance(diff)
        if errorMargin * -1 < distToCenter < errorMargin:
            return True
        else:
            return False

    def isInCenter(self, errorMargin):
        center = [(self.actualTile[0] * self.tileSize) + ((self.tileSize // 2 - self.offsets[0]) % self.tileSize), (self.actualTile[1] * self.tileSize) + ((self.tileSize // 2 - self.offsets[1]) % self.tileSize)]
        return self.isDistanceLessThan(self.globalPos, center, errorMargin)

    def showGrid(self):
        cv.imshow("ventana", cv.resize(self.grid.getMap(), (400, 400), interpolation=cv.INTER_AREA))     
    
    def getWallBetween(self, tileNode1, tileNode2):
        diff = [(tileNode1[0] - tileNode2[0]) // 2, (tileNode1[1] - tileNode2[1]) // 2]
        if diff == [-1,0]:
            orient = "down"
        elif diff == [1,0]:
            orient = "up"
        elif diff == [0,1]:
            orient = "left"
        elif diff == [0,-1]:
            orient = "right"
        else:
            orient = "undefined"
        return orient

    def getPassedWall(self):
        return self.getWallBetween(self.actualTileNode, self.prevTileNode)

        

    def sendMessage(self, indentifier):
        self.robot.emitter.sendMessage(self.globalPos, indentifier)
    
    def seqFollowCalculatedPath(self):
        self.seqFollowPath(self.calculatedPath)

    def calculatePath(self):
        print(" CALCULATING ")
        start = self.grid.getTileNode(self.globalPos)
        bfsResults = self.grid.bfs(start, ("unknown", "uncollectedVictim"), 10)
        unknownResults = bfsResults[0]
        uncVictimResults = bfsResults[1]
        if len(uncVictimResults) == 0:
            bfsResults = self.grid.bfs(start, ("unknown", "uncollectedVictim"))
            uncVictimResults = bfsResults[1]
            unknownResults = bfsResults[0]
        priorClosestReachable = False
        if len(unknownResults) and not len(uncVictimResults):
            for result in unknownResults:
                if result[0] == self.closestReachableNodeTile[0] and result[1] == self.closestReachableNodeTile[1]: 
                    priorClosestReachable = True
                    break
        if not priorClosestReachable:
            if len(uncVictimResults) and uncVictimResults[0][2] < 10:
                self.closestReachableNodeTile = uncVictimResults[0]
            elif len(unknownResults):
                self.closestReachableNodeTile = unknownResults[0]
            else:
                self.closestReachableNodeTile = self.grid.getTileNode(self.startPos)
                self.ending = True

        #print("closest: " + str(self.grid.getPosfromTileNode(self.closestReachableNodeTile)))
        path = self.grid.astar(start, self.closestReachableNodeTile)
        self.calculatedPath = []
        for node in path:
            self.calculatedPath.append(self.grid.getPosfromTileNode([node[1], node[0]]))
        if len(self.calculatedPath) != 1:
            self.calculatedPath.pop(0)
        self.followPathIndex = 0
        print("Path: " + str(self.calculatedPath))

    def seqDo360(self, direction="right", maxSpeed=0.7):
        if self.seqEvent():
            if self.do360FirstTime:
                self.do360OriginAngle = self.globalRot
                self.do360FirstTime = False
        if direction == "right":
            self.seqMove(maxSpeed, maxSpeed * -1)
            self.seqDelaySec(0.2)
            if self.seqRotateToDegs(self.do360OriginAngle, "farthest", maxSpeed):
                self.do360FirstTime = True
        else:
            self.seqMove(maxSpeed * -1, maxSpeed)
            self.seqDelaySec(0.2)
            if self.seqRotateToDegs(self.do360OriginAngle, "farthest", maxSpeed):
                self.do360FirstTime = True

    def seqFollowPath(self, path):
        if self.seqMg.check():
            #print("Following")
            if self.followPathIndex == len(path):
                self.seqMg.nextSeq()
                self.followPathIndex = 0
                #print("ENDED")
            elif self.moveToCoords(path[self.followPathIndex]):
                #print("Moved")
                self.followPathIndex += 1
                self.movedInPath = True
            else:
                self.movedInPath = False
        else:
            self.followPathIndex = 0
            self.movedInPath = False
        return self.seqMg.seqDone()

    def areVictimsAtRange(self, camera, inputRange):
        victimInRange = False
        for pos, img in zip(r.cameras[camera]["poses"], r.cameras[camera]["images"]):
            #print(self.cameras[camera]["camera"].getVictimRange(pos, img))
            #print(img.shape)
            if self.cameras[camera]["camera"].getVictimRange(pos, img) == inputRange:
                victimInRange = True
                break
        return victimInRange
    
    def getVictimLetter(self, camera):
        for pos, img in zip(r.cameras[camera]["poses"], r.cameras[camera]["images"]):
            if self.cameras[camera]["camera"].getVictimRange(pos, img) == 0:
                return self.cameras[camera]["camera"].classifyVictim(img)

        
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

    def rotateToDegs(self, degs, orientation="closest", maxSpeed=0.7):
        accuracy = 2
        if self.seqRotateToDegsFirstTime:
            #print("STARTED ROTATION")
            self.seqRotateToDegsInitialRot = self.globalRot
            self.seqRotateToDegsinitialDiff = round(self.seqRotateToDegsInitialRot - degs)
            self.seqRotateToDegsFirstTime = False
        diff = self.globalRot - degs
        
        moveDiff = max(round(self.globalRot), degs) - min(self.globalRot, degs)
        if diff > 180 or diff < -180:
            moveDiff = 360 - moveDiff
        speedFract = min(mapVals(moveDiff, accuracy, 90, 0.2, 1), maxSpeed)
        if accuracy  * -1 < diff < accuracy or 360 - accuracy < diff < 360 + accuracy:
            self.seqRotateToDegsFirstTime = True
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
                self.robot.move(speedFract * -1, speedFract)
            elif direction == "left":
                self.robot.move(speedFract, speedFract * -1)
            #print("speed fract: " +  str(speedFract))
            #print("target angle: " +  str(degs))
            #print("moveDiff: " + str(moveDiff))
            #print("diff: " + str(diff))
            #print("orientation: " + str(orientation))
            #print("direction: " + str(direction))
            #print("initialDiff: " + str(self.seqRotateToDegsinitialDiff))
        return False

    def seqRotateToDegs(self, degs, orientation="closest", maxSpeed=0.7):
        if self.seqMg.check():
            if self.rotateToDegs(degs, orientation, maxSpeed):
                self.robot.move(0,0)
                self.seqMg.nextSeq()
        return self.seqMg.seqDone()

    def moveToCoords(self, targetPos):
        errorMargin = 0.2
        descelerationStart = 0.5 * self.tileSize
        diffX = targetPos[0] - self.globalPos[0]
        diffY = targetPos[1] - self.globalPos[1]
        #print("diff in pos: " + str(diffX) + " , " + str(diffY))
        dist = math.sqrt(diffX ** 2 + diffY ** 2)
        #print("Dist: "+ str(dist))
        if errorMargin * -1 < dist < errorMargin:
            #self.robot.move(0,0)
            #print("FinisehedMove")
            return True
        else:
            #print("Moving")
            rad = math.atan2(diffX, diffY)
            ang = rad * 180 / math.pi
            ang = normalizeAngle(ang)
            #print("traget ang: " + str(ang))
            ratio = min(mapVals(dist, 0, descelerationStart, 0.1, 1), 1)
            ratio = max(ratio, 0.8)
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
        if not self.stMg.checkState(newState):
            self.stMg.changeState(newState)
            self.seqMg.resetSequence()
            #self.followPathIndex = 0
            self.do360FirstTime = True
            self.seqRotateToDegsFirstTime = True
            self.delayFirstTime = True
    
    def resetState(self):
        self.stMg.changeState(self.stMg.state)
        self.seqMg.resetSequence()
        #self.followPathIndex = 0
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
        self.actualTimeStep += 1
        self.actualTime = self.robot.getTime()
        self.globalPos = self.robot.gps.getPosition()
        self.actualTile = self.grid.getTile([self.globalPos[0] + self.offsets[0], self.globalPos[1] + self.offsets[1]])
        self.actualTileNode = self.grid.getTileNode(self.globalPos)
        self.prevTSTileNode = self.grid.getTileNode(self.prevGlobalPos)
        if self.actualTileNode != self.prevTSTileNode:
            self.prevTileNode =  self.prevTSTileNode
        if self.firstStep:
            self.prevGlobalPos = self.globalPos
            self.prevTSTileNode = self.prevTileNode = self.actualTileNode
            self.startPos = self.globalPos
            self.offsets =  [round((self.actualTile[0] * self.tileSize) - self.globalPos[0]) + self.tileSize // 2, round((self.actualTile[1] * self.tileSize) - self.globalPos[1]) + self.tileSize // 2]
            self.offsets = [self.offsets[0] % self.tileSize, self.offsets[1] % self.tileSize]
            print("OFFSETS: " + str(self.offsets))
            self.grid.offsets = self.offsets
            self.firstStep = False
        self.globalRoll = self.robot.rollGyro.update(self.actualTime, self.globalRoll)
        self.globalPitch = self.robot.pitchGyro.update(self.actualTime, self.globalPitch)
        if self.rotDetectMethod == "velocity":
            self.globalRot = self.robot.gyro.update(self.actualTime, self.globalRot)
        elif self.rotDetectMethod == "position":
            rot = self.robot.getRotationByPos(self.prevGlobalPos, self.globalPos)
            if rot != -1:
                #print("ROT: " + str(rot))
                self.globalRot = rot
        self.isHot = self.robot.heatLeft.isClose() or self.robot.heatRight.isClose()
        self.colourTileType = self.robot.colourSensor.getTileType()
        diffInX = max(self.globalPos[0], self.prevGlobalPos[0]) -  min(self.globalPos[0], self.prevGlobalPos[0])
        diffInY = max(self.globalPos[1], self.prevGlobalPos[1]) -  min(self.globalPos[1], self.prevGlobalPos[1])
        self.diffInPos = getDistance([diffInX, diffInY])
        # Updating cameras
        self.cameras["centre"]["poses"], self.cameras["centre"]["images"] = self.cameras["centre"]["camera"].getVictimImagesAndPositions()
        self.cameras["right"]["poses"], self.cameras["right"]["images"] = self.cameras["right"]["camera"].getVictimImagesAndPositions()
        self.cameras["left"]["poses"], self.cameras["left"]["images"] = self.cameras["left"]["camera"].getVictimImagesAndPositions()
        self.showGrid()
    
    def bottomUpdate(self):
        # Bottom updates
        self.prevGlobalPos = self.globalPos

        if self.globalRoll < 180:
            varInRoll = self.globalRoll
        else:
            varInRoll = 360 - r.globalRoll
        if r.globalPitch < 180:
            varInPitch = self.globalPitch
        else:
            varInPitch = 360 - self.globalPitch

        newWalls = False
        newTiles = False
        if self.doAfterTimesteps(4):
            if self.doWallMap:
                newWalls = self.doWallMapping()
        
        if self.stoppedMovingFT:
                self.stoppedMovingST = self.actualTime
                self.stoppedMovingSP = self.globalPos
                self.stoppedMovingFT = False

        if self.isDistanceLessThan(self.globalPos, self.stoppedMovingSP, 1):
            self.timeWithoutMoving = self.actualTime - self.stoppedMovingST
        else:
            self.timeWithoutMoving = 0
            self.stoppedMovingFT = True

        if self.doTileMap:
            newTiles = self.doTileMapping()
        

        if newWalls:
            self.doCalculatePath = True
        
        if self.doAutoMapCalculating:
            if self.doCalculatePath and (self.overridePath or ((not self.overridePath) and self.movedInPath)):
                self.calculatePath()
                self.doCalculatePath = False
            
            if self.doAfterTimesteps(10):
                try:
                    nextPathPos = self.calculatedPath[self.followPathIndex]
                    wallInBetween = self.getWallBetween(self.actualTileNode, self.grid.getTileNode(nextPathPos))
                    if wallInBetween != "undefined":
                        if self.grid.getValue(self.actualTileNode, wallInBetween) == "occupied":
                            self.calculatePath()
                except IndexError:
                    pass

                for pos in self.calculatedPath:
                    if self.grid.getPosition(pos) == "occupied":
                        self.calculatePath()
                        break

        cv.waitKey(1)


# Instanciacion de capa de abstracción
# Abstraction layer instantiation
r = AbstractionLayer(timeStep, "start")

#MAIN PROGRAM
# Updates the global position, rotation, colorSensor position and colors, shows the grid and does mapping
while r.update():
    # v Program v
    # --This are checks that need to happen on any state--

    # Detects if the robot has teleported and changes to the corresponding state
    if r.diffInPos >= r.tileSize * 0.5:
        r.changeState("teleported")
    # Detects if the robot is close to a hole and changes to the corresponding state
    if r.colourTileType == "trap":
        r.changeState("trap")

    # Start state
    # runs at the start of th program. Calibrates the offsets for mapping and the initial global rotaion of the robot.
    if r.isState("start"):
        # This happens in sequence (One order executes after the other)
        r.startSequence()
        if r.seqEvent():
            r.doWallMap = False
            r.doTileMap = False
            r.doAutoMapCalculating = False
        if r.seqEvent():
            r.rotDetectMethod = "position"
        r.seqMove(1, 1)
        r.seqDelaySec(0.2)
        if r.seqMove(0,0):
            print("Initial Rotation: " + str(r.globalRot))
            r.rotDetectMethod = "velocity"
        r.seqMove(-1, -1)
        r.seqDelaySec(0.2)
        r.seqMove(0,0)
        if r.seqEvent():
            r.doWallMap = True
            r.doTileMap = True
            
        r.seqDo360()
        if r.seqEvent():
            r.calculatePath()
            r.doAutoMapCalculating = True
            r.changeState("main")
        

    # Main state
    elif r.isState("main"):
        # This happens in sequence (One order executes after the other)
        r.startSequence()
        if r.seqEvent():
            r.stoppedMovingFT = True
        r.seqFollowCalculatedPath()
        r.seqMove(0,0)
        if r.seqEvent():
            r.calculatePath()
            print(r.calculatedPath)
            r.resetState()

        #This happens continously

        #Checks if the robot wants to end and if it is on the exit/start tile
        if r.grid.getTileNode(r.globalPos) == r.grid.getTileNode(r.startPos) and r.ending:
            r.changeState("exit")
        #checks if there are victims to send to the controller in camera and it hasnt sent them yet 
        # and changes to the corrsponding state
        if r.areVictimsAtRange("centre", 0) and r.grid.getPosition(r.globalPos) != "collectedVictim":
            r.changeState("visualVictim")
        # Checks if the temperature of the heat sensor is high enough to send the victim to the controller 
        # and changes to the corresponding state
        if r.isHot:
            r.changeState("heatVictim")
        
        # It is in a tile with possible victims in it, changes to the corresponding state
        if r.movedInPath and r.grid.getPosition(r.globalPos) == "uncollectedVictim":
            r.changeState("analize")
        
        if r.timeWithoutMoving > 6:
            r.changeState("stopped moving")

    #Trap state. It gets away from the detected trap and maps it on to the grid
    elif r.isState("trap"):
        # This happens in sequence (One order executes after the other)
        r.startSequence()
        r.seqMove(0,0)
        if r.seqEvent():
            r.doWallMap == False
            print("MAPPED TRAP")
            r.grid.setPosition(r.robot.colourSensor.getPosition(r.globalPos, r.globalRot), "occupied")
        r.seqMove(-0.2, -0.2)
        r.seqDelaySec(1)
        r.seqMove(0,0)
        if r.seqEvent():
            r.calculatePath()
            r.doWallMap == True
            r.changeState("main")
        
    elif r.isState("analize"):
         # This happens in sequence (One order executes after the other)
        r.startSequence()
        r.seqMove(0,0)
        r.seqDo360(maxSpeed=0.5)
        if r.seqEvent():
            r.grid.setPosition(r.globalPos, "collectedVictim")
            r.calculatePath()
            r.changeState("main")
        if r.areVictimsAtRange("centre", 0) and r.grid.getPosition(r.globalPos) != "collectedVictim":
            r.changeState("visualVictim")
            
        #print("ANALIZING")

    
    elif r.isState("visualVictim"):
         # This happens in sequence (One order executes after the other)
        r.startSequence()
        r.seqMove(0,0)
        if r.seqDelaySec(3):
            letter = r.getVictimLetter("centre")
            if letter is not None:
                r.sendMessage(letter)
        r.seqDelaySec(0.2)
        if r.seqEvent():
            r.grid.setPosition(r.globalPos, "collectedVictim")
            r.changeState("main")
        
    # Heated victim state
    elif r.isState("heatVictim"):
         # This happens in sequence (One order executes after the other)
        r.startSequence()
        r.seqMove(0,0)
        if r.seqDelaySec(3):
            r.sendMessage("T")
        r.seqDelaySec(0.2)
        if r.seqEvent():
            r.changeState("main")
    
    # Exit state
    elif r.isState("exit"):
        r.sendMessage("E")

    # Teleported state
    elif r.isState("teleported"):
         # This happens in sequence (One order executes after the other)
        r.startSequence()
        if r.seqEvent():
            r.doWallMap = False
            #r.rotDetectMethod = "position"
            r.globalPitch = 0
            r.globalRoll = 0
        r.seqMove(0, 0)
        r.seqDelaySec(0.4)
        r.seqMove(1, 1)
        if r.seqEvent():
            r.rotDetectMethod = "position"
        if r.seqDelaySec(0.19):
            r.rotDetectMethod = "velocity"
        r.seqDelaySec(0.01)
        if r.seqMove(0,0):
            print("Initial Rotation: " + str(r.globalRot))
            
        r.seqMove(-1, -1)
        r.seqDelaySec(0.2)
        if r.seqMove(0,0):
            #r.rotDetectMethod = "velocity"
            r.doWallMap = True
            r.calculatePath()
            r.changeState("main")

    elif r.isState("stopped moving"):
        # This happens in sequence (One order executes after the other)
        r.startSequence()
        r.seqMove(-0.2, -0.2)
        r.seqDelaySec(1)
        r.seqMove(0,0)
        r.seqDo360()
        if r.seqEvent():
            r.calculatePath()
            r.changeState("main")
        

    #print("diff in pos: " + str(r.diffInPos))
    #print("Global position: " + str(r.globalPos))
    #print("Global rotation: " + str(round(r.globalRot)))
    #print("Tile type: " + str(r.colourSensor.getTileType()))
    #print("State: " + r.stMg.state)
    #print("-----------------")
    