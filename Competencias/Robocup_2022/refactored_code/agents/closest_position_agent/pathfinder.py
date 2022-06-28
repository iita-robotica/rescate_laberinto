absuloute_dir = r'/home/ale/rescate_laberinto/Competencias/Robocup_2022/refactored_code'
import sys
sys.path.append(absuloute_dir)

import utilities
from data_structures.expandable_node_grid import Grid


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


def isTraversable(grid, index):
    node = grid.get_node(index, expand=False, phantom=True)
    if node.node_type == "tile":
        raise ValueError("Invalid instance: expected vortex, got tile")
        return node.tileType != "hole"
    if node.node_type == "wall":
        raise ValueError("Invalid instance: expected vortex, got wall")
        return not node.occupied
    if node.node_type == "vortex":
        if node.status == "occupied":
            return False
        traversable = True
        for adjacentIndex in ((-1, 1), (1, -1), (1, 1), (-1, -1), (0, 1), (0, -1), (1, 0), (-1, 0)):
            adjacent = grid.get_node((index[0] + adjacentIndex[0], index[1] + adjacentIndex[1]), expand=False, phantom=True)
            if adjacent.node_type == "tile":
                if adjacent.tile_type == "hole" or adjacent.status == "occupied":
                    traversable = False
            elif adjacent.node_type == "wall":
                if adjacent.status == "occupied":
                    traversable = False
            else:
                raise ValueError(("invalid instance: " + str(type(adjacent))))
        return traversable
    return False
    

# Returns a list of tuples as a path from the given start to the given end in the given maze
def aStar(grid, start, end):
    assert isTraversable(grid, start)
    assert isTraversable(grid, end)

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
            if not isTraversable(grid, nodePosition):
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

def isBfsAddable(grid, index):
    node = grid.get_node(index, expand=False, phantom=True)
    if node.node_type == "vortex":
        for adjacentPos in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
            adjacent = [index[0] + adjacentPos[0], index[1] + adjacentPos[1]]
            if not grid.get_node(adjacent, expand=False, phantom=True).explored:
                return True
        return False
    else:
        return False

# Breath First Search algorithm
# Returns the tiles in order and with the distance of each one
def bfs(grid, start, limit="undefined"):
    visited = []
    queue = []
    found = []
    start = [start[0], start[1], 0]
    visited.append(start)
    queue.append(start)
    while queue:
        if len(found) > 100:
            break
        coords = queue.pop(0)
        y = coords[1]
        x = coords[0]
        dist = coords[2]
        if limit != "undefined":
            if dist > limit:
                break
        
        if isBfsAddable(grid, coords[:2]):
            found.append(coords)

        for newPosition in ((0, 1), (0, -1), (-1, 0), (1, 0)):
            neighbour = [x + newPosition[0] * 2, y + newPosition[1] * 2, dist + 1]
            inList = False
            for node in visited:
                if node[0] == neighbour[0] and node[1] == neighbour[1]:
                    inList = True
                    break
            if inList:
                continue

            # Make sure walkable terrain
            if isTraversable(grid, neighbour[:2]):
                visited.append(neighbour)
                queue.append(neighbour)

    return found

def getPreferabilityScore(node):
    pass

def getBestNode(possibleNodes, startNode, orientation):
    if len(possibleNodes) > 0:
        bestNode = possibleNodes[0]
        if bestNode[:2] == list(startNode):
            bestNode = possibleNodes[1]
        for posNode in possibleNodes:
            diff = utilities.substractLists(startNode, posNode[:2])
            #print("Diff:", diff)
            #print("Multiplied orientation: ", multiplyLists(orientation, [-2, -2]))
            if posNode[2] > 1:
                break
            
            elif diff == utilities.multiplyLists(orientation, [-2, -2]):
                bestNode = posNode
                break
    else:
        bestNode = startNode
    #return possibleNodes[-1][:2]
    return bestNode[:2]

def find_start_node(grid):
    for y, row in enumerate(grid.grid):
        for x, node in enumerate(row):
            if node.is_robots_position:
                return [x - grid.offsets[0], y - grid.offsets[1]]

def getBestPath(grid, orientation):
    startNode = find_start_node(grid)
    bfsLimits = (100,)
    possibleNodes = []

    for limit in bfsLimits:
        possibleNodes = bfs(grid, startNode, limit)
        if len(possibleNodes):
            break
    
    bestNode = getBestNode(possibleNodes, startNode, orientation)
   
    bestPath = aStar(grid, startNode, bestNode)
    print("BFS NODES: ", possibleNodes)
    print("Best Node:", bestNode)
    print("AStar PATH: ", bestPath)
    print("Start Vortex: ", startNode)

    return bestPath


if __name__ == "__main__":
    grid = Grid((20, 20))
    grid.print_grid()
    
    grid.get_node((0, 0)).is_robots_position = True

    path = getBestPath(grid, [1, 0])

    print("Path: ", path)

    for node in path:
        grid.get_node(node).mark1 = True
    
    grid.print_grid()
