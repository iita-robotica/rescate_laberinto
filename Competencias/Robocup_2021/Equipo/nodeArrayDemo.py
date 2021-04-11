import numpy as np

# Class that defines a tile node in the grid
class TileNode:
    
    # Tuple with all allowed tile types
    __allowedTypes = ("undefined", "normal", "hole", "swamp", "checkpoint", "start", "connection1-2", "connection2-3")
    
    def __init__(self):
        self.dimensions = [30, 30] # Dimensions of the tile
        self.type = "undefined" # Can be undefined, start, normal, swamp, hole, checkpoint, connection1-2, connection2-3
        self.fixtures = [] # List of all fixtures in walls adjacent to tile
        self.obstacles = [] # List of obstacles in tile

# Class that defines a wall node in the grid
class WallNode:
    def __init__(self):
        self.dimensions = [30, 10, 1] # Dimensions of the wall
        self.occupied = False # If there is a wall. Can be True or false.
        self.isFloating = False # If it is a floating wall
        self.isCurved = False # If it is a curved wall
        self.fixtures = [] # List of all fixtures in wall

class VertexNode:
    def __init__(self):
        self.dimensions = [1, 10, 1] # Dimensions of the vertex
        self.occupied = False # If there is a vertex. Can be True or false.
        
array = np.array([
                [WallNode(), VertexNode(), WallNode(), VertexNode(), WallNode(), VertexNode()],

                [TileNode(), WallNode()  , TileNode(), WallNode()  , TileNode(), WallNode()  ],

                [WallNode(), VertexNode(), WallNode(), VertexNode(), WallNode(), VertexNode()],

                [TileNode(), WallNode()  , TileNode(), WallNode()  , TileNode(), WallNode()  ],

                [WallNode(), VertexNode(), WallNode(), VertexNode(), WallNode(), VertexNode()],

                [TileNode(), WallNode()  , TileNode(), WallNode()  , TileNode(), WallNode()  ],

                [WallNode(), VertexNode(), WallNode(), VertexNode(), WallNode(), VertexNode()]
                  ])

