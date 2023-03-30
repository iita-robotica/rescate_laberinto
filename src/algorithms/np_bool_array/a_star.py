import numpy as np
import cv2 as cv
import math

# aStarNode class for A* pathfinding 
class aStarNode():
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.p = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position
    
    def __repr__(self):
        return str(self.position)

class aStarAlgorithm:
    def __init__(self):
        self.adjacents = [[0, 1], [0, -1], [-1, 0], [1, 0], ]#[1, 1], [1, -1], [-1, -1], [-1, 1]]
        self.preference_weight = 50

    def get_preference(self, preference_grid, position):
        if preference_grid is None:
            return 0
        elif not (position[0] >= preference_grid.shape[0] or position[1] >= preference_grid.shape[1] or position[0] < 0 or position[1] < 0):
            return preference_grid[position[0], position[1]]
        else:
            return 0
        
    # Returns a list of tuples as a path from the given start to the given end in the given maze
    def a_star(self, grid: np.ndarray, start, end, preference_grid=None):
        debug_grid = np.zeros((grid.shape[0], grid.shape[1], 3), dtype=np.uint8)

        # Create start and end node
        startNode = aStarNode(None, list(start))
        startNode.g = startNode.h = startNode.f = 0
        
        if grid[start[0], start[1]]:
            print("WARNING: Start position is not traversable")

        endNode = aStarNode(None, list(end))

        if grid[end[0], end[1]]:
            print("WARNING: End position is not traversable")
            return []

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
            for adj in self.adjacents:  # Adjacent squares
                # Get node position
                nodePosition = [currentNode.position[0] + adj[0], currentNode.position[1] + adj[1]]
                # Make sure walkable terrain
                if not (nodePosition[0] >= grid.shape[0] or nodePosition[1] >= grid.shape[1] or nodePosition[0] < 0 or nodePosition[1] < 0):
                    #print("OUT OF BOUNDS")
                    if grid[nodePosition[0], nodePosition[1]]:
                        #print("NOT TRAVERSABLE")
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
                child.h =  ((child.position[0] - endNode.position[0]) ** 2) + (
                           (child.position[1] - endNode.position[1]) ** 2)
                
                child.p = self.get_preference(preference_grid, child.position) * self.preference_weight
                
                child.f = child.g + child.h + child.p
                # Child is already in the open list
                for index, openNode in enumerate(openList):
                    if child == openNode:
                        if child.p + child.g > openNode.p + openNode.g:
                            continueLoop = True
                            break



                if continueLoop:
                    continue
                # Add the child to the open list
                openList.append(child)
            
            
            for o in openList:
                debug_grid[o.position[0], o.position[1]] = [0, 0, 255]

            cv.imshow("debug", debug_grid)

            cv.waitKey(1)
            
        return []