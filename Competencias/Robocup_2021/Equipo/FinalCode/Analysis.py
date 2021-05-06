import numpy as np
import cv2 as cv
import sys
import copy
sys.path.append(r"C:\\Users\\ANA\\Desktop\\Webots - Erebus\\rescate_laberinto\\Competencias\\Robocup_2021\\Equipo\\FinalCode")
from UtilityFunctions import *

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

    # Returns a node given the position of a tile and directions to indicate walls and vertices
    def getNode(self, position, side=[0,0]):
        if isinstance(side, str):
            x = position[0] * self.chunkSize[0] + self.directionToNumber(side)[0]
            y = position[1] * self.chunkSize[1] + self.directionToNumber(side)[1]
        else:
            x = position[0] * self.chunkSize[0] + side[0]
            y = position[1] * self.chunkSize[1] + side[1]
        return self.getRawNode((x, y))
    
    # Sets a node given the position of a tile and directions to indicate walls and vertices
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

# Finds the best path to follow
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
    




chunk1 = np.array([[0, 1],
                   [1, 3]])

chunk2 = np.array([[VortexNode(), WallNode()],
                   [WallNode()  , TileNode(tileType="swamp")]])

grid = Grid(chunk2, [10, 10])
print("--------------")

#grid.addRowAtStart()


#grid.getNode((0,0)).occupied = True
grid.getNode((0,0)).tileType = "hole"

for i in range(3):
    grid.getNode((i, 0), "up").occupied = True

print(grid.grid)
print("Offsets: " + str(grid.offsets))


print("--------------")



