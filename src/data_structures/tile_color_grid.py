import numpy as np
import cv2 as cv
import copy


class TileColorExpandableGrid:
    def __init__(self, initial_shape, tile_size):
        self.array_shape = np.array(initial_shape, dtype=int)
        self.offsets = self.array_shape // 2

        self.grid_index_max = self.array_shape - self.offsets # Maximum grid index
        self.grid_index_min = self.offsets * -1 # Minimum grid index

        self.array = np.zeros(self.array_shape, np.bool_)

        self.resolution = 1 / tile_size # resolution of the grid with regards to the coordinate system of the gps / the world
    
    # Index conversion
    def coordinates_to_grid_index(self, coordinates: np.ndarray) -> np.ndarray:
        coords = (coordinates * self.resolution).astype(int)
        return np.flip(coords)

    def grid_index_to_coordinates(self, grid_index: np.ndarray) -> np.ndarray:
        index = (grid_index.astype(float) / self.resolution)
        return np.flip(index)

    def array_index_to_grid_index(self, array_index: np.ndarray) -> np.ndarray:
        return array_index - self.offsets
    
    def grid_index_to_array_index(self, grid_index: np.ndarray) -> np.ndarray:
        return grid_index + self.offsets
    
    def array_index_to_coordinates(self, array_index) -> np.ndarray:
        grid_index = self.array_index_to_grid_index(array_index)
        return self.grid_index_to_coordinates(grid_index)
    
    def coordinates_to_array_index(self, coordinates) -> np.ndarray:
        grid_index = self.coordinates_to_grid_index(coordinates)
        return self.grid_index_to_array_index(grid_index)

    # Grid expansion
    def expand_to_grid_index(self, grid_index: np.ndarray):
        """
        Expands all arrays to the specified index. 
        Note that all array_idexes should be recalculated after this operation.
        """

        array_index = self.grid_index_to_array_index(grid_index)
        if array_index[0] + 1 > self.array_shape[0]:
            self.add_end_row(array_index[0] - self.array_shape[0] + 1)

        if array_index[1] + 1 > self.array_shape[1]:
            self.add_end_column(array_index[1] - self.array_shape[1] + 1)

        if array_index[0] < 0:
            self.add_begining_row(array_index[0] * -1)
        if array_index[1] < 0:
            self.add_begining_column(array_index[1] * -1)
    
    def add_end_row(self, size):
        self.array_shape = np.array([self.array_shape[0] + size, self.array_shape[1]])
        
        self.array = self.__add_end_row_to_array(self.array, size)
        
    def add_begining_row(self, size):
        self.offsets[0] += size
        self.array_shape = np.array([self.array_shape[0] + size, self.array_shape[1]])

        self.array = self.__add_begining_row_to_array(self.array, size)

    def add_end_column(self, size):
        self.array_shape = np.array([self.array_shape[0], self.array_shape[1] + size])

        self.array = self.__add_end_column_to_array(self.array, size)

    def add_begining_column(self, size):
        self.offsets[1] += size
        self.array_shape = np.array([self.array_shape[0], self.array_shape[1] + size])

        self.array = self.__add_begining_column_to_array(self.array, size)

    def __add_end_row_to_array(self, array, size):
        array = np.vstack((array, np.zeros((size, self.array_shape[1]), dtype=array.dtype)))
        return array
    
    def __add_begining_row_to_array(self, array, size):
        array = np.vstack((np.zeros((size, self.array_shape[1]), dtype=array.dtype), array))
        return array
    
    def __add_end_column_to_array(self, array, size):
        array = np.hstack((array, np.zeros((self.array_shape[0], size), dtype=array.dtype)))
        return array

    def __add_begining_column_to_array(self, array, size):
        array = np.hstack((np.zeros((self.array_shape[0], size), dtype=array.dtype), array))
        return array
    
    # Debug
    def get_colored_grid(self):
       pass
    