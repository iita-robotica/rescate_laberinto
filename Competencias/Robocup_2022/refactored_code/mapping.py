import numpy as np
import cv2 as cv
import sys
import copy

import point_cloud_processor

class Grid:
    def __init__(self, initial_shape, res=100):
        self.offsets = [initial_shape[0] // 2, initial_shape[1] // 2]
        self.resolution = res
        self.shape = initial_shape
        self.grid = np.zeros(self.shape, dtype=np.uint8)

    def expand_grid_to_point(self, point):
        x, y = point
        x, y = x + self.offsets[0], y + self.offsets[1]

        if y + 1 > self.shape[0]:
            self.add_end_row(y - self.shape[0] +1)
        if x + 1 > self.shape[1]:
            self.add_end_column(x - self.shape[1] +1)
        if y < 0:
            self.add_begining_row(-y)
        if x < 0:
            self.add_begining_column(-x)
    
    
    def add_point(self, point, value=255):
        self.expand_grid_to_point(point)
        
        x, y = point
        x, y = x + self.offsets[0], y + self.offsets[1]
    
        self.grid[y, x] = value
    
    def sum_to_point(self, point, value):
        self.expand_grid_to_point(point)
        
        x, y = point
        x, y = x + self.offsets[0], y + self.offsets[1]
    
        self.grid[y, x] = min(255, self.grid[y, x] + value)
    
    def get_point(self, point):
        x, y = point
        x, y = x + self.offsets[0], y + self.offsets[1]
        return self.grid[y, x]
     
    def add_end_row(self, size):
        self.shape = (self.shape[0]+ size, self.shape[1] )
        self.grid = np.vstack((self.grid, np.zeros((size, self.shape[1]), dtype=np.uint8)))
    
    def add_begining_row(self, size):
        self.offsets[1] += size
        self.shape = (self.shape[0]+ size, self.shape[1] )
        self.grid = np.vstack((np.zeros((size, self.shape[1]), dtype=np.uint8), self.grid))
    
    def add_end_column(self, size):
        self.shape = (self.shape[0], self.shape[1] + size)
        self.grid = np.hstack((self.grid, np.zeros((self.shape[0], size), dtype=np.uint8)))

    def add_begining_column(self, size):
        self.offsets[0] += size
        self.shape = (self.shape[0], self.shape[1] + size)
        self.grid = np.hstack((np.zeros((self.shape[0], size), dtype=np.uint8), self.grid))

    def print_grid(self, max_size=(2000, 1000)):
        print("grid shape: ", self.grid.shape)
        grid1 = copy.deepcopy(self.grid)
        for y, row in enumerate(grid1):
            for x, pixel in enumerate(row):
                if y % self.resolution == 0 or x % self.resolution == 0:
                    grid1[y][x] = 255

        if grid1.shape[0] > max_size[0]:
            ratio = max_size[0] / grid1.shape[0]

            width = round(grid1.shape[1] * ratio)
            grid1 = cv.resize(grid1, dsize=(width, max_size[0]))
        
        elif grid1.shape[1] > max_size[1]:
            ratio = max_size[1] / grid1.shape[1]

            height = round(grid1.shape[0] * ratio)
            grid1 = cv.resize(grid1, dsize=(max_size[1], height))

        cv.imshow("grid", grid1)
        cv.waitKey(1)




if __name__ == "__main__":
    my_grid = Grid((5, 5))

    my_grid.add_point((2, 2))
    print(my_grid.offsets)

    my_grid.print_grid()

    my_grid.add_point((-5, -10))
    print(my_grid.offsets)

    my_grid.print_grid()

class Graph:
    def __init__(self) -> None:
        self.graph = {}
    
    def isAdjacent(self, pos1, pos2):
        diff = []
        for p1, p2 in zip(pos1, pos2):
            diff = p1 - p2
            if diff > 1 or diff < -1:
                return False
        return True

    def block(self, pos1, pos2):
        if pos1 in self.graph:
            self.graph[pos1].append(pos2)
        else:
            self.graph[pos1] = [pos2, ]
        if pos2 in self.graph:
            self.graph[pos2].append(pos2)
        else:
            self.graph[pos2] = [pos1, ]
    
    def check(self, pos1, pos2):
        if not self.isAdjacent(pos1, pos2):
            return False
        if pos1 in self.graph:
            return pos2 not in self.graph[pos1]
        else:
            return True
    
    def unblock(self, pos1, pos2):
        if pos1 in self.graph:
            self.graph[pos1].remove(pos2)
        if pos2 in self.graph:
            self.graph[pos2].remove(pos2)

my_graph = Graph()

my_graph.block((0, 0), (0, 1))

print(my_graph.graph)

print("puedo pasar?", my_graph.check((0,2), (0,1)))


