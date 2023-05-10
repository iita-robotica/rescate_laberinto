import numpy as np
import cv2 as cv

from mapping.mapper import Mapper
from data_structures.vectors import Position2D, Vector2D
from data_structures.angle import Angle
from algorithms.np_bool_array.bfs import NavigatingBFSAlgorithm

import math

class VictimPositionFinder:
    def __init__(self, mapper: Mapper) -> None:
        self.mapper = mapper
        self.bfs = NavigatingBFSAlgorithm(lambda x: x, lambda x: True)
        self.__is_victim = False
        self.__victim_position = None
        self.min_victim_detection_distance = 0.04

    def update(self):
        self.find_victim_position()
        
    def find_victim_position(self):
        victim_array = self.mapper.pixel_grid.arrays["victims"]
        victim_array[self.mapper.pixel_grid.arrays["fixture_detection"]] = False
        robot_array_index = self.mapper.pixel_grid.grid_index_to_array_index(self.mapper.robot_grid_index)
        results = self.bfs.bfs(victim_array,
                               self.mapper.pixel_grid.arrays["fixture_detection_zone"], 
                               robot_array_index)
        
        if len(results) == 0:
            self.__is_victim = False
        else:
            self.__is_victim = True
            self.__victim_position = Position2D(self.mapper.pixel_grid.array_index_to_coordinates(results[0]))

    def get_victim_position(self):
        return self.__victim_position

    def is_there_victims(self):
        return self.__is_victim

    def is_close_to_victim(self) -> bool:
        if self.__victim_position is None: return False
        return self.mapper.robot_position.get_distance_to(self.__victim_position) < self.min_victim_detection_distance

