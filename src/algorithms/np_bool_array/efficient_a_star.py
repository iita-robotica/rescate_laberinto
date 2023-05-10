import numpy as np
import cv2 as cv
from heapq import heappop, heappush
import math


class aStarNode:
    def __init__(self, location):
        self.location = location
        self.parent = None
        self.g = float('inf')
        self.p = 0
        self.f = 0

    def __gt__(self, other):  # make nodes comparable
        return self.f > other.f

    def __repr__(self):
        return str(self.location)


class aStarAlgorithm:
    def __init__(self):
        self.adjacents = [[0, 1], [0, -1], [-1, 0], [1, 0], ]#[1, 1], [1, -1], [-1, -1], [-1, 1]]
        #self.preference_weight = 5
        self.preference_weight = 2
    
    @staticmethod
    def reconstructpath(node):
        path = []
        while node is not None:
            path.append(node.location)
            node = node.parent
        path.reverse()
        return path

    @staticmethod
    def heuristic(start, target):
        # optimistic score, assuming all cells are friendly
        dy = abs(start[0] - target[0])
        dx = abs(start[1] - target[1])
        return min(dx, dy) * 15 + abs(dx - dy) * 10

    @staticmethod
    def get_preference(preference_grid, position):
        if preference_grid is None:
            return 0
        elif not (position[0] >= preference_grid.shape[0] or position[1] >= preference_grid.shape[1] or position[0] < 0 or position[1] < 0):
            return preference_grid[position[0], position[1]]
        else:
            return 0
    
    @staticmethod
    def is_traversable(grid, position):
        if not (position[0] >= grid.shape[0] or position[1] >= grid.shape[1] or position[0] < 0 or position[1] < 0):
            return not grid[position[0], position[1]]
        else:
            return True


        
    # Returns a list of tuples as a path from the given start to the given end in the given maze
    def a_star(self, grid: np.ndarray, start, end, preference_grid=None, search_limit=float('inf')):
        debug_grid = np.zeros((grid.shape[0], grid.shape[1], 3), dtype=np.uint8)

        # Create start and end node
        start_node = aStarNode(tuple(start))
        start_node.g = 0
        
        if not self.is_traversable(grid, start):
            print("WARNING: Start position is not traversable")

        end_node = aStarNode(tuple(end))

        if not self.is_traversable(grid, end):
            print("WARNING: End position is not traversable")
            return []

        end_node.g = end_node.h = end_node.f = 0
        # Initialize open and closed list
        openList = [start_node]
        best_cost_for_node_lookup = {tuple(start_node.location): start_node.g}
        closed = set()

        loop_n = 0
        # Loop until end
        while openList:            
            # Get the current node
            node = heappop(openList)
            if node.location in closed:
                continue

            closed.add(node.location)
            # If found the goal
            if node.location == end_node.location:
                #print(f"Finished Astar. Took {loop_n} loops.")
                return self.reconstructpath(node)
            
            # Generate children
            for adj in self.adjacents:  # Adjacent squares
                # Get node position
                child_location = (node.location[0] + adj[0], node.location[1] + adj[1])
                # Make sure walkable terrain
                if not self.is_traversable(grid, child_location):
                    continue
                # Create new node
                new_child = aStarNode(child_location)
                new_child.parent = node

                new_child.g = node.g + 1
                new_child.h =  self.heuristic(new_child.location, end_node.location)
                
                new_child.p = self.get_preference(preference_grid, new_child.location) * self.preference_weight
                
                new_child.f = new_child.g + new_child.h + new_child.p

                if child_location in best_cost_for_node_lookup.keys():
                    if new_child.g + new_child.p < best_cost_for_node_lookup[child_location]:
                        best_cost_for_node_lookup[child_location] = new_child.g + new_child.p
                        heappush(openList, new_child)
                        
                else:
                    best_cost_for_node_lookup[child_location] = new_child.g + new_child.p
                    heappush(openList, new_child)

            loop_n += 1
            if loop_n > search_limit:
                break
            
            """
            for o in openList:
                debug_grid[o.location[0], o.location[1]] = [0, 0, 255]

            cv.imshow("debug", debug_grid)

            cv.waitKey(1)
            """
            
        return []