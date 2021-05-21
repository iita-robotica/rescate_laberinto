# Class that defines a tile node in the grid
class TileNode:
    
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