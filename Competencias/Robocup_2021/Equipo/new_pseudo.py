# Class that defines wall fixture in the nodes of the grid, like victims and hazard maps
class WallFixture:
    # Tuple with all allowed identifiers for wall fixtures
    __allowedTypes = ("harmed", "secure", "unharmed", "flammable_gas", "poison", "corrosive", "organic_proxide")

    # Dctionary with identifier to type equivalence
    __typeToClassification = { "harmed"          : "victim", 
                                "secure"          : "victim", 
                                "unharmed"        : "victim", 
                                "flammable_gas"   : "hazard_map", 
                                "poison"          : "hazard_map", 
                                "corrosive"       : "hazard_map", 
                                "organic_proxide" : "hazard_map"}

    def __init__(self, initType):
        # Type of the wall fixture. can be any of the ones on the __allowedTypes tuple
        self.__type = None
        self.setType(initType)
    
    # returms the wall fixture type
    def getType(self):
        return self.__type

    # Sets a new wall fixture type, only if the new type sis in __allowedTypes
    def setType(self, newType):
        if newType in self.__allowedTypes:
            self.__type = newType
        else:
            raise Exception("Invalid tile type introduced")
    
    # Returns the classification of the wall fixture defined by the __typeToClassification dict
    def getClassification(self):
        return self.__typeToClassification[self.__type]

# Class that defines an obstacle in th nodes of the tile
class Obstacle:
    def __init__(self, initPos, initRadious):
         self.__position = []
         self.__radious = 0
         self.setPosition(initPos)
         self.setRadious(initRadious)

    # Changes the position of the obstacle
    def setPosition(self, newPosition):
        self.__position = newPosition

    # Returns the position of the obstacle
    def getPosition(self):
        return self.__position

    # Changes the radious of the obstacle
    def setRadious(self, newRadious):
        self.__radious = newRadious

    # Returns the radious of the obstacle
    def getRadious(self):
        return self.__radious

# Class that defines a tile node in the grid
class TileNode:
    
    # Tuple with all allowed tile types
    __allowedTypes = ("undefined", "normal", "hole", "swamp", "checkpoint", "start", "connection1-2", "connection2-3")
    
    def __init__(self):
        self.dimensions = [30, 30] # Dimensions of the tile
        self.type = "undefined" # Can be undefined, start, normal, swamp, hole, checkpoint, connection1-2, connection2-3
        self.fixtures = [] # List of all fixtures in walls adjacent to tile
        self.obstacles = [] # List of obstacles in tile

    # returns the score of the tile, wich is defined as how convenient is the tile to traverse  between 0 and 1
    def getScore(self):
        pass 
    
    # sets a new type if it is in the allowed types tuple
    def setType(self, newType):
        if newType in self.__allowedTypes:
            self.__type = newType
        else:
            raise Exception("Invalid tile type introduced")

    # Returns the type of the tile
    def getType(self):
        return self.__type

    # Returns all wall fixtures in tile
    def getFixtures(self):
        return self.__fixtures

    def addFixture(self, setType):
        self.__fixtures.append(WallFixture(setType))
    
    # Returns all obstacles in tile
    def getObstacles(self):
        return self.__obstacles

    def addObstacle(self, position, radious):
        self.__obstacles.append(Obstacle(position, radious))


    # Sets the tile size
    def setSize(self, newSize):
        self.__dimensions = [newSize, newSize]

    # Returns the tile size
    def getSize(self):
        return self.__dimensions[0]

# Class that defines a wall node in the grid
class WallNode:
    def __init__(self):
        self.dimensions = [30, 1, 10] # Dimensions of the wall
        self.occupied = False # If there is a wall. Can be True or false.
        self.isFloating = False # If it is a floating wall
        self.isCurved = False # If it is a curved wall
        self.fixtures = [] # List of all fixtures in wall


    def isOccupied(self):
        pass

    def setOccuppied(self):
        pass

    def isFloating(self):
        pass

    def setFloating(self):
        pass

    def setDimensions(self):
        pass

    def getDimensions(self):
        pass

    def addFixture(self):
        pass

    def getFixtures(self):
        pass

# Class that defines a dynamic grid of nodes with walls and tiles
class Grid:
    def __init__(self):
        self.grid = [[]] # array of nodes

    def getNode(self):
        pass

    def setNode(self):
        pass

    def getTile(self):
        pass

    def setTile(self):
        pass

    def getWall(self, tile, side):
        pass

    def setWall(self, tile, side):
        pass

    def getDimensions(self):
        pass

    def getGrid(self):
        pass
    
    


class BestTileFinder:

    def __init__(self):
        self.__bestTile
        self.__grid

    def setGrid(self):
        # updtade grid
        # run calculate best
        pass

    def __calculateBest(self):
        pass

    def getBestTile(self):
        pass

class Pathfinder:

    def __init__(self):
        self.__bestPath
        self.__grid
        self.__objective

    def setGrid(self):
        #update grid
        #run find path
        pass

    def __findPath(self):
        pass
    
    def setObjective(self):
        # update objective
        # run find path
        pass

    def getBestPath(self):
        pass



# analyzes the best posiible path to follow
class Analyzer:
    def __init__(self):
        self.bestTileFinder = BestTileFinder()
        self.pathFinder = Pathfinder()
        self.grid = Grid()
        self.__optimalPath = [] # Always show the optimal path for the robot to follow
        self.zone = None # Can be 1 2 or 3 depending on wich zone the robot is in
    
    def getOptimalPath(self):
        pass


    def setSensorDetections(self, detections):
        pass

    def setTile(self, position, type):
        pass

    def __isCurved(self):
        pass
    
    def __posToTile(self):
        pass

    def __tileToPos(self):
        pass

