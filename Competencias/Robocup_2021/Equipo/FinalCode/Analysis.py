from Competencias.Robocup_2021.Equipo.FinalCode.UtilityFunctions import getDistance
import numpy as np
import cv2 as cv
import sys
import copy

sys.path.append(r"C:\\Users\\ANA\\Desktop\\Webots - Erebus\\rescate_laberinto\\Competencias\\Robocup_2021\\Equipo\\FinalCode")
from UtilityFunctions import *
import PointCloudToGrid

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
                        printableArray[x][y] = 100
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
            return node.tileType != "hole"
        if isinstance(node, WallNode):
            return not node.occupied
        if isinstance(node, VortexNode):
            if node.occupied:
                return False
            for adjacentIndex in ((-1, 1), (1, -1), (1, -1), (-1, 1), (0, 1), (0, -1), (1, 0), (-1, 0)):
                adjacent = node = self.grid.getRawNode((index[0] + adjacentIndex[0], index[1] + adjacentIndex[0]))
                if isinstance(adjacent, TileNode):
                    return node.tileType != "hole"
                elif isinstance(node, WallNode):
                    return not node.occupied
            return True
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
            for newOrientation in ((0, 1), (0, -1), (-1, 0), (1, 0)):  # Adjacent squares
                newPosition = self.orientations[newOrientation]
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
        """
        if not self.isTraversable(index):
            return False
        """
        node = self.grid.getRawNode(index)
        if isinstance(node, self.vortexNode):
            return not node.traversed
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
            x = coords[0]
            y = coords[1]
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
    
    def setGrid(self, grid):
        self.grid = grid

    def getBestPath(self):
        possibleNodes = self.bfs(self.startVortex, self.searchLimit)
        return possibleNodes[1]
    
class Analyst:
    def __init__(self, tileSize):
        self.tileSize = tileSize
        gridChunk = np.array([[VortexNode(), WallNode()],
                              [WallNode()  , TileNode()]])
        self.grid = Grid(gridChunk, (100, 100))
        self.queManager = PointCloudToGrid.PointCloudQueManager(queSize=5, pointMultiplier=100, queStep=1)
        self.divider = PointCloudToGrid.PointCloudDivider(tileSize=0.06, pointMultiplier=100, pointPermanenceThresh=30)
        from ClassifierTemplate import tilesDict as classifTemp
        self.classifier = PointCloudToGrid.Classifier(classifTemp)
        self.pathFinder = PathFinder(VortexNode, WallNode, TileNode, self.grid, 50)
        self.pathFinder.setStartVortex((1, 1))
        self.pathFinder.getBestPath()
        self.totalPointCloud = []
        self.actualRawNode = []
        self.__bestPath = []
        self.calculatePath = True
        self.pathIndex = 0
        self.positionReachedThresh = 1
    
    def loadPointCloud(self, pointCloud):
        self.queManager.update(pointCloud)
        self.totalPointCloud = self.queManager.getTotalPointCloud()
        tilesWithPoints = self.divider.getTiles(self.totalPointCloud)
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
        convPos  = self.multiplyPos(colorSensorPosition)
        convPos = self.divider.getTile(convPos)
        self.grid.getNode(convPos).tileType = tileType

    def getQuadrant(self, posInTile):
        if posInTile[0] > self.tileSize / 2: x = 1
        else: x = -1
        if posInTile[1] > self.tileSize / 2: y = 1
        else: y = -1
        return [x, y]
    
    def getVortexPosInTile(self, quadrant):
        return [(self.tileSize / 2) + (quadrant[0] * self.tileSize // 2), (self.tileSize / 2) + (quadrant[1] * self.tileSize // 2)]


    def multiplyPos(self, position):
        return (position[0] * self.divider.pointMultiplier, position[1] * self.divider.pointMultiplier)

        
    def update(self, position):
        convPos = self.multiplyPos(position)
        tile = self.divider.getTile(convPos)
        posInTile = self.divider.getPosInTile(convPos)
        quadrant = self.getQuadrant(posInTile)
        startRawNode = self.grid.processedToRawNode(tile, quadrant)

        self.pathFinder.setStartVortex(startRawNode)

        vortexPos = self.multiplyPos(self.getVortexPosInTile(quadrant))
        diff = [vortexPos[0] - posInTile[0], vortexPos[1] - posInTile[1]]
        distToVortex = getDistance(diff)

        if distToVortex < self.positionReachedThresh and startRawNode == self.__bestPath[self.pathIndex]:
            self.pathIndex += 1

        if self.pathIndex >= len(len(self.__bestPath)):
            self.calculatePath = True

    
        if self.calculatePath:
            self.__bestPath = self.pathFinder.getBestPath()
            self.pathIndex = 0
            self.calculatePath = False
    
    def getBestNodeToMove(self):
        return self.__bestPath[self.pathIndex]

        
    
    def getGrid(self):
        return self.grid.grid
    
    def showGrid(self):
        cv.imshow("Analyst grid", cv.resize(self.grid.getNumpyPrintableArray(), (400, 400), interpolation=cv.INTER_NEAREST))



if __name__ == "__main__":

    """
    chunk1 = np.array([[0, 1],
                    [1, 3]])

    chunk2 = np.array([[VortexNode(), WallNode()],
                    [WallNode()  , TileNode(tileType="swamp")]])

    grid = Grid(chunk2, [10, 10])
    print("--------------")

    #grid.addRowAtStart()


    #grid.getNode((0,0)).occupied = True
    grid.getNode((0,0)).tileType = "hole"

    for i in range(grid.offsets[0] // 2 * -1, grid.size[0] // 2 - grid.offsets[0] // 2):
        print(i)
        grid.getNode((i, 0), "up").occupied = True

    print(grid.grid)
    print("Offsets: " + str(grid.offsets))


    print("--------------")
    """


    analyst = Analyst(0.06)
    pointCloud = [[0,0.02], [1.5, 1.12]]
    analyst.loadPointCloud(pointCloud)

