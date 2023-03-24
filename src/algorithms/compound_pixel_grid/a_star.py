import numpy as np
from data_structures.compound_pixel_grid import PointGrid

# aStarNode class for A* pathfinding 
class aStarNode():
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position[0] == other.position[0] and self.position[1] == other.position[1]

class aStarAlgorithm:
    def __init__(self):
        self.adjacents = ((0, 1), (0, -1), (-1, 0), (1, 0))

    # Returns a list of tuples as a path from the given start to the given end in the given maze
    def a_star(self, grid: PointGrid, start, end):
        #assert is_traversable(grid, start)

        

        #assert is_traversable(grid, end)

        # Create start and end node
        startNode = aStarNode(None, start)
        startNode.g = startNode.h = startNode.f = 0
        
        start_point = grid.get_point(np.array(start))
        if grid.traversable_grid[start_point[0], start_point[1]]:
            print("WARNING: Start position is not traversable")

        endNode = aStarNode(None, end)
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
            for newPosition in self.adjacents:  # Adjacent squares
                # Get node position
                nodePosition = (currentNode.position[0] + newPosition[0], currentNode.position[1] + newPosition[1])
                # Make sure walkable terrain
                p = grid.get_point(np.array(nodePosition))
                if grid.traversable_grid[p[0], p[1]]:
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