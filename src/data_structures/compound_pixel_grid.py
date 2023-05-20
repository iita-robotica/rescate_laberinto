import numpy as np
import cv2 as cv
import copy
from data_structures.vectors import Position2D, Vector2D
from data_structures.angle import Angle
import math
from flow_control.step_counter import StepCounter


class CompoundExpandablePixelGrid:
    def __init__(self, initial_shape, pixel_per_m, robot_radius_m):
        self.array_shape = np.array(initial_shape, dtype=int)
        self.offsets = self.array_shape // 2
        self.resolution = pixel_per_m # resolution of the grid with regards to the coordinate system of the gps / the world

        self.arrays = {
            "detected_points": np.zeros(self.array_shape, np.uint8), # Number of points detected in position
            "walls": np.zeros(self.array_shape, np.bool_),
            "occupied": np.zeros(self.array_shape, np.bool_), # Confirmed occupied point
            "traversable": np.zeros(self.array_shape, np.bool_), # Is or not traversable by the robot, assuming that the robot center is there. True means not traversable.
            "navigation_preference": np.zeros(self.array_shape, np.float32), # The preference for navigation for each pixel. More means less preferred to navigate through.
            "traversed": np.zeros(self.array_shape, np.bool_), # Robot has already gone through there
            "seen_by_camera": np.zeros(self.array_shape, np.bool_), # Has been seen by any of the cameras
            "seen_by_lidar": np.zeros(self.array_shape, np.bool_), # Has been seen by the lidar (Though not necessarily detected as occupied)
            "walls_seen_by_camera": np.zeros(self.array_shape, np.bool_),
            "walls_not_seen_by_camera": np.zeros(self.array_shape, np.bool_),
            "discovered": np.zeros(self.array_shape, np.bool_),
            "floor_color": np.zeros((self.array_shape[0], self.array_shape[1], 3), np.uint8),
            "floor_color_detection_distance": np.zeros(self.array_shape, np.uint8),
            "average_floor_color": np.zeros((self.array_shape[0], self.array_shape[1], 3), np.uint8),
            "holes": np.zeros(self.array_shape, np.bool_),
            "victims": np.zeros(self.array_shape, np.bool_),
            "victim_angles": np.zeros(self.array_shape, np.float32),
            "fixture_detection": np.zeros(self.array_shape, np.bool_),
            "fixture_detection_zone": np.zeros(self.array_shape, np.bool_),
            "fixture_distance_margin": np.zeros(self.array_shape, np.bool_),
            "robot_detected_fixture_from": np.zeros(self.array_shape, np.bool_),
            "robot_center_traversed": np.zeros(self.array_shape, np.bool_),
        }

    @property
    def grid_index_max(self):
        return self.array_shape - self.offsets # Maximum grid index
    
    @property
    def grid_index_min(self):
        return self.offsets * -1 # Minimum grid index

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
        
        for key in self.arrays:
            self.arrays[key] = self.__add_end_row_to_grid(self.arrays[key], size)
        
    def add_begining_row(self, size):
        self.offsets[0] += size
        self.array_shape = np.array([self.array_shape[0] + size, self.array_shape[1]])

        for key in self.arrays:
            self.arrays[key] = self.__add_begining_row_to_grid(self.arrays[key], size)

    def add_end_column(self, size):
        self.array_shape = np.array([self.array_shape[0], self.array_shape[1] + size])

        for key in self.arrays:
            self.arrays[key] = self.__add_end_column_to_grid(self.arrays[key], size)

    def add_begining_column(self, size):
        self.offsets[1] += size
        self.array_shape = np.array([self.array_shape[0], self.array_shape[1] + size])

        for key in self.arrays:
            self.arrays[key] = self.__add_begining_column_to_grid(self.arrays[key], size)

    def __add_end_row_to_grid(self, grid, size):
        shape = np.array(grid.shape)
        shape[0] = size
        shape[1] = self.array_shape[1]
        grid = np.vstack((grid, np.zeros(shape, dtype=grid.dtype)))
        return grid
    
    def __add_begining_row_to_grid(self, grid, size):
        shape = np.array(grid.shape)
        shape[0] = size
        shape[1] = self.array_shape[1]
        grid = np.vstack((np.zeros(shape, dtype=grid.dtype), grid))
        return grid
    
    def __add_end_column_to_grid(self, grid, size):
        shape = np.array(grid.shape)
        shape[0] = self.array_shape[0]
        shape[1] = size
        grid = np.hstack((grid, np.zeros(shape, dtype=grid.dtype)))
        return grid

    def __add_begining_column_to_grid(self, grid, size):
        shape = np.array(grid.shape)
        shape[0] = self.array_shape[0]
        shape[1] = size
        grid = np.hstack((np.zeros(shape, dtype=grid.dtype), grid))
        return grid

    # Debug
    def get_colored_grid(self):
        """
        Get graphical representation of the grid for debug.
        """
        #color_grid = np.zeros((self.array_shape[0], self.array_shape[1], 3), dtype=np.float32)
        color_grid = self.arrays["floor_color"].astype(np.float32) / 255
        
        #color_grid[:, :, 1] = self.arrays["navigation_preference"][:, :] / 200
        color_grid[self.arrays["traversable"]] = (1, 0, 0)
        
        #color_grid[self.arrays["discovered"]] = (0, 1, 1)
        #color_grid[self.arrays["seen_by_camera"]] += (0.5, 0, 0)
        #color_grid[self.arrays["fixture_detection_zone"]] = (0, 1, 1)
        color_grid[self.arrays["fixture_distance_margin"]] = (0, 0, 1)


        color_grid[self.arrays["occupied"]] = (1, 1, 1)

        #color_grid[self.arrays["walls_not_seen_by_camera"]] = (0, 0, 1)

        color_grid *= 0.3

        color_grid[self.arrays["victims"]] = (0, 1, 0)

        color_grid[self.arrays["robot_center_traversed"]] = (.5, 0., .5)
        
 
        return color_grid