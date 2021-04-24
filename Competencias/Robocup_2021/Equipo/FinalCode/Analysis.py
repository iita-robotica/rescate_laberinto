import numpy as np
import cv2 as cv
import sys
sys.path.append(r"C:\\Users\\ANA\\Desktop\\Webots - Erebus\\rescate_laberinto\\Competencias\\Robocup_2021\\Equipo\\FinalCode")
from UtilityFunctions import *

# Class that defines a tile node in the grid
class TileNode:
    __wallFixtureTypes = ("harmed", "secure", "unharmed", "flammable_gas", "poison", "corrosive", "organic_proxide")
    # Tuple with all allowed tile types
    __allowedTypes = ("undefined", "normal", "hole", "swamp", "checkpoint", "start", "connection1-2", "connection2-3")
    __allowedCurvedWalls = ([1, 1], [-1, -1], [-1, 1], [-1, 1])
    def __init__(self):
        self.dimensions = [0.06, 0.06] # Dimensions of the tile
        self.type = "undefined" # Can be undefined, start, normal, swamp, hole, checkpoint, connection1-2, connection2-3
        self.curvedWall = [0, 0] # if it is a tile with curved walls and curved wall position
        self.fixtures = [] # List of all fixtures in walls adjacent to tile
        self.obstacles = [] # List of obstacles in tile

# Class that defines a wall node in the grid
class WallNode:
    __wallFixtureTypes = ("harmed", "secure", "unharmed", "flammable_gas", "poison", "corrosive", "organic_proxide")
    def __init__(self):
        self.dimensions = [0.06, 0.06, 0.01] # Dimensions of the wall
        self.occupied = False # If there is a wall. Can be True or false.
        self.isFloating = False # If it is a floating wal
        self.fixtures = [] # List of all fixtures in wall


#Class that defines a vortex node in the grid
class VortexNode:
    def __init__(self):
        self.dimensions = [0.01, 0.01, 0.06] # Dimensions of the vortex
        self.occupied = False # If there is a vortex. Can be True or false.

class Grid:
    
    def __init__(self, chunk, initialSize):
        self.startingSize = initialSize
        self.size = [2, 2]
        self.offsets = [0, 0]
        self.grid = [[]]
        self.chunk = chunk
        self.__constructGrid()
        
        
    def directionToNumber(self, direction):
        if direction == "center" or direction == "centre":
            n = [0, 0]
        elif direction == "right":
            n = [1, 0]
        elif direction == "left":
            n = [-1, 0]
        elif direction == "up":
            n = [0, -1]
        elif direction == "down":
            n = [0, 1]
        elif direction == "right-up" or direction == "up-right":
            n = [1, -1]
        elif direction == "right-down" or direction == "down-right":
            n = [1, 1]
        elif direction == "left-down" or direction == "down-left":
            n = [-1, 1]
        elif direction == "left-up" or direction == "up-left":
            n = [-1, -1]
        return n

    def __constructGrid(self):
        self.grid = self.chunk.copy()
        for _ in range((self.startingSize[0] // 2) - 1):
            self.addColumnAtEnd()
        for _ in range((self.startingSize[1] // 2) - 1):
            self.addRowAtEnd()

        self.offsets[0] = self.startingSize[0] // 2
        self.offsets[1] = self.startingSize[1] // 2

        self.size = self.startingSize
        
    def addRowAtEnd(self):
        row = self.chunk.copy()
        if self.size[0] > 1:
            for _ in range((self.size[0] // 2) - 1):
                row = np.hstack((row.copy(), self.chunk.copy()))
            self.grid = np.vstack((self.grid.copy(), row.copy()))
            self.size[1] += 2
            
    def addRowAtStart(self):
        row = self.chunk.copy()
        if self.size[0] > 1:
            for _ in range((self.size[0] // 2) - 1):
                row = np.hstack((row.copy(), self.chunk.copy()))
            self.grid = np.vstack((row.copy(), self.grid.copy()))
            self.size[1] += 2
            self.offsets[1] += 2

    def addColumnAtEnd(self):
        column = self.chunk.copy()
        if self.size[1] > 1:
            for _ in range((self.size[1] // 2) - 1):
                column = np.vstack((column.copy(), self.chunk.copy()))
            self.grid = np.hstack((self.grid.copy(), column.copy()))
            self.size[0] += 2

    def addColumnAtStart(self):
        column = self.chunk.copy()
        if self.size[1] > 1:
            for _ in range((self.size[1] // 2) - 1):
                column = np.vstack((column.copy(), self.chunk.copy()))
            self.grid = np.hstack((column.copy(), self.grid.copy()))
            self.size[0] += 2
            self.offsets[0] += 2
    

    def getRawNode(self, position):
        x = position[0] + self.offsets[0]
        y = position[1] + self.offsets[1]
        return self.grid[x][y]
    
    def setRawNode(self, position, value):
        x = position[0] + self.offsets[0]
        y = position[1] + self.offsets[1]
        self.grid[x][y] = value

    def getNode(self, position, side=[0,0]):
        if isinstance(side, str):
            x = position[0] * 2 + self.directionToNumber(side)[0]
            y = position[1] * 2 + self.directionToNumber(side)[1]
        else:
            x = position[0] * 2 + side[0]
            y = position[1] * 2 + side[1]
        return self.getRawNode((x, y))
    
    def setNode(self, position, value, side=[0,0]):
        if isinstance(side, str):
            x = position[0] * 2 + self.directionToNumber(side)[0]
            y = position[1] * 2 + self.directionToNumber(side)[1]
        else:
            x = position[0] * 2 + side[0]
            y = position[1] * 2 + side[1]
        x1, y1 = self.getRawNode((x, y))
        self.grid[x1, y1] = value
    
    def duplicate(self, node):
        self.grid[node[0], node[1]] +=  10

class PathFinder:

    def __init__(self):
        self.grid = None
        self.start = [0, 0]
        self.objective = [0, 0]

    def isTraversble(self, node):
        return True

    def AStar(self):
        pass

    def bfs(self):
        pass

    def getPreferabilityScore(self, node):
        pass

    def getBestPath(self):
        return []
    

class ConverterPoint:

    def __init__(self, startingTime, maxTime, definitiveThreshold):
        self.definitive = False
        self.count = 0
        self.maxTime = maxTime
        self.definitiveThreshold = definitiveThreshold
        self.startingTime = startingTime
        self.time = 0
    
    def update(self, time):
        self.time = time - self.startingTime
        if not self.definitive:
            if self.count >= self.definitiveThreshold:
                self.definitive = True
            elif self.time > self.maxTime:
                self.count = 0
                self.startingTime = self.time

    def increaseCount(self):
        self.count += 1

class PointCloudToTileConverter:

    __tilesDict = {"wallRight": np.array([[0, 0, 0, 0, 1, 1],
                                          [0, 0, 0, 0, 1, 1],
                                          [0, 0, 0, 0, 1, 1],
                                          [0, 0, 0, 0, 1, 1],
                                          [0, 0, 0, 0, 1, 1],
                                          [0, 0, 0, 0, 1, 1]]),

                    "wallLeft": np.array([[1, 1, 0, 0, 0, 0],
                                          [1, 1, 0, 0, 0, 0],
                                          [1, 1, 0, 0, 0, 0],
                                          [1, 1, 0, 0, 0, 0],
                                          [1, 1, 0, 0, 0, 0],
                                          [1, 1, 0, 0, 0, 0]]),
                    
                    "wallUp": np.array([[1, 1, 1, 1, 1, 1],
                                        [1, 1, 1, 1, 1, 1],
                                        [0, 0, 0, 0, 0, 0],
                                        [0, 0, 0, 0, 0, 0],
                                        [0, 0, 0, 0, 0, 0],
                                        [0, 0, 0, 0, 0, 0]]),
                    
                    "wallDown": np.array([[0, 0, 0, 0, 0, 0],
                                          [0, 0, 0, 0, 0, 0],
                                          [0, 0, 0, 0, 0, 0],
                                          [0, 0, 0, 0, 0, 0],
                                          [1, 1, 1, 1, 1, 1],
                                          [1, 1, 1, 1, 1, 1]])
                  }

    def __init__(self, size, initialTime, maxPointTime, pointPermanenceThreshold):

        self.maxPointTime = maxPointTime
        self.pointPermanenceThresh = pointPermanenceThreshold
        self.possibleGrid = self.makeGrid(initialTime)
        self.size = size

    def makeGrid(self, time):
        row = []
        for _ in self.size:
            row.append(ConverterPoint(time, self.maxPointTime, self.pointPermanenceThresh))
        grid = []
        for _ in self.size:
            grid.append(row.copy())
        return np.array(grid)


    def update(self, time, pointCloud):
        for point in pointCloud:
            self.possibleGrid[point[0], point[1]].increaseCount()
            self.possibleGrid[point[0], point[1]].update(time)


class PointCloudToNodeGridConverter:
    def __init__(self):
        self.pointCloud = []


chunk1 = np.array([[0, 1],
                   [1, 3]])

chunk2 = np.array([[VortexNode(), WallNode()],
                   [WallNode()  , TileNode()]])

grid = Grid(chunk2, (10, 10))
print("--------------")

print(grid.grid)
print("Offsets: " + str(grid.offsets))


print("--------------")

print(grid.getNode((2, 2), side="up").isCurved)

