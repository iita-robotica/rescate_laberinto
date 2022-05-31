import numpy as np
import cv2 as cv
import copy
import utilities

class Grid:
    def __init__(self, initial_shape, res=100):
        self.offsets = [initial_shape[0] // 2, initial_shape[1] // 2]
        self.resolution = res
        self.shape = initial_shape
        self.grid = np.zeros(self.shape, dtype=np.int)

        self.value_divider = 1023 / 255
        self.value_limit = 1023

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
    
        self.grid[y, x] = min(self.grid[y, x] + value, self.value_limit)
    
    def get_point(self, point):
        x, y = point
        x, y = x + self.offsets[0], y + self.offsets[1]
        return self.grid[y, x]
     
    def add_end_row(self, size):
        self.shape = (self.shape[0]+ size, self.shape[1] )
        self.grid = np.vstack((self.grid, np.zeros((size, self.shape[1]), dtype=np.int)))
    
    def add_begining_row(self, size):
        self.offsets[1] += size
        self.shape = (self.shape[0]+ size, self.shape[1] )
        self.grid = np.vstack((np.zeros((size, self.shape[1]), dtype=np.int), self.grid))
    
    def add_end_column(self, size):
        self.shape = (self.shape[0], self.shape[1] + size)
        self.grid = np.hstack((self.grid, np.zeros((self.shape[0], size), dtype=np.int)))

    def add_begining_column(self, size):
        self.offsets[0] += size
        self.shape = (self.shape[0], self.shape[1] + size)
        self.grid = np.hstack((np.zeros((self.shape[0], size), dtype=np.int), self.grid))

    def print_grid(self, max_size=(2000, 1000)):
        print("grid shape: ", self.grid.shape)
        grid1 = copy.deepcopy(self.grid)
        #utilities.draw_grid(grid1, self.resolution, self.offsets)

        if grid1.shape[0] > max_size[0]:
            ratio = max_size[0] / grid1.shape[0]

            width = round(grid1.shape[1] * ratio)
            grid1 = cv.resize((grid1 // self.value_divider).astype(np.uint8), dsize=(width, max_size[0]))
        
        elif grid1.shape[1] > max_size[1]:
            ratio = max_size[1] / grid1.shape[1]

            height = round(grid1.shape[0] * ratio)
            grid1 = cv.resize((grid1 // self.value_divider).astype(np.uint8), dsize=(max_size[1], height))
        
        elif grid1.shape[1] >= grid1.shape[0]:
            ratio = max_size[1] / grid1.shape[1]

            height = round(grid1.shape[0] * ratio)
            grid1 = cv.resize((grid1 // self.value_divider).astype(np.uint8), dsize=(max_size[1], height), interpolation=cv.INTER_NEAREST)
        
        elif grid1.shape[0] >= grid1.shape[1]:
            ratio = max_size[0] / grid1.shape[0]

            width = round(grid1.shape[1] * ratio)
            grid1 = cv.resize((grid1 // self.value_divider).astype(np.uint8), dsize=(width, max_size[0]), interpolation=cv.INTER_NEAREST)

        cv.imshow("grid", grid1.astype(np.uint8))
        cv.waitKey(1)

if __name__ == "__main__":
    my_grid = Grid((5, 5))

    my_grid.add_point((2, 2))
    print(my_grid.offsets)

    my_grid.print_grid()

    my_grid.add_point((-5, -10))
    print(my_grid.offsets)

    my_grid.print_grid()