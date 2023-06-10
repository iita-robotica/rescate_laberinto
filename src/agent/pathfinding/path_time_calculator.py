import numpy as np
import cv2 as cv

from data_structures.vectors import Position2D
from mapping.mapper import Mapper

from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid
from algorithms.np_bool_array.efficient_a_star import aStarAlgorithm
from algorithms.np_bool_array.bfs import NavigatingBFSAlgorithm

from flags import SHOW_PATHFINDING_DEBUG, SHOW_GRANULAR_NAVIGATION_GRID

class PathTimeCalculator():
    def __init__(self, mapper: Mapper, factor: float, exponent: float):
        self.__a_star = aStarAlgorithm()
        self.__closest_free_point_finder = NavigatingBFSAlgorithm(lambda x : x == 0, lambda x: True)
        
        self.__mapper = mapper

        self.factor = factor
        self.exponent = exponent

    
    def calculate(self, target_position: np.ndarray):
        n = self.__calculate_path_lenght(target_position)
        return n * self.factor + n ** self.exponent

    def __calculate_path_lenght(self, target_position):
        
        # Expand grid to target index
        target_grid_index = self.__mapper.pixel_grid.coordinates_to_grid_index(target_position)
        self.__mapper.pixel_grid.expand_to_grid_index(target_grid_index)

        # Get start array index (if robot index occupied, get closest unoccupied point)
        start_array_index = self.__mapper.pixel_grid.coordinates_to_array_index(self.__mapper.robot_position)
        start_array_index = self.__get_closest_traversable_array_index(start_array_index)

        # Get target array index (if target index occupied, get closest unoccupied point)
        target_array_index = self.__mapper.pixel_grid.coordinates_to_array_index(target_position)
        target_array_index = self.__get_closest_traversable_array_index(target_array_index)

        # Calculate path
        a_star_path = self.__a_star.a_star(self.__mapper.pixel_grid.arrays["traversable"], 
                                        start_array_index,
                                        target_array_index,
                                        self.__mapper.pixel_grid.arrays["navigation_preference"])
        
        
        return len(a_star_path)


    def __get_closest_traversable_array_index(self, array_index):
        if self.__mapper.pixel_grid.arrays["traversable"][array_index[0], array_index[1]]:
            return  self.__closest_free_point_finder.bfs(found_array=self.__mapper.pixel_grid.arrays["traversable"],
                                                         traversable_array=self.__mapper.pixel_grid.arrays["traversable"],
                                                         start_node=array_index)[0]
        else:
            return array_index