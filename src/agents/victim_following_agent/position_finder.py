import numpy as np
import cv2 as cv

from mapping.mapper import Mapper
from data_structures.vectors import Position2D, Vector2D
from data_structures.angle import Angle
from algorithms.np_bool_array.bfs import NavigatingBFSAlgorithm, BFSAlgorithm
import skimage
import copy

import math

class VictimPositionFinder:
    def __init__(self, mapper: Mapper) -> None:
        self.mapper = mapper
        self.fixture_finder = NavigatingBFSAlgorithm(lambda x: x, lambda x: True)
        self.detection_position_finder = NavigatingBFSAlgorithm(lambda x: x, lambda x: not x, max_result_number=5)
        self.robot_position_finder = NavigatingBFSAlgorithm(lambda x: x, lambda x: not x)
        self.min_fixture_detection_distance = 0.06
        self.detection_line_size = 12
        self.path = []

        
    def find_fixture_array_index(self):
        fixture_array = self.mapper.pixel_grid.arrays["victims"]
        fixture_array[self.mapper.pixel_grid.arrays["fixture_detection"]] = False
        if np.count_nonzero(fixture_array):
            robot_array_index = self.mapper.pixel_grid.grid_index_to_array_index(self.mapper.robot_grid_index)
            return self.fixture_finder.bfs(fixture_array, fixture_array, robot_array_index)[0]
        else:
            return None
    

    def get_next_position(self):
        fixture_array_index = self.find_fixture_array_index()

        if fixture_array_index is None:
            #print("not any victims")
            return None
            
        
        robot_array_index = Position2D(self.mapper.pixel_grid.grid_index_to_array_index(self.mapper.robot_grid_index))
        robot_detection_array_index = self.detection_position_finder.bfs(self.mapper.pixel_grid.arrays["fixture_detection_margin"], self.mapper.pixel_grid.arrays["walls"], fixture_array_index)
        
        if not len(robot_detection_array_index):
            return None
        
        circle_indexes = skimage.draw.disk(tuple(robot_detection_array_index[0]), self.detection_line_size//2, shape=tuple(self.mapper.pixel_grid.array_shape))

        mask = np.ones_like(self.mapper.pixel_grid.arrays["fixture_distance_margin"])

        mask[circle_indexes] = False

        detection_line_array = copy.deepcopy(self.mapper.pixel_grid.arrays["fixture_distance_margin"])
        detection_line_array[mask] = False

        detection_line_array[self.mapper.pixel_grid.arrays["robot_center_traversed"]] = False

        debug = self.mapper.pixel_grid.get_colored_grid()
        debug[detection_line_array] = (1, 1, 1)

        cv.imshow("fixture path", debug)

        if np.count_nonzero(detection_line_array):
            

            indexes = np.nonzero(detection_line_array)
            indexes = np.transpose(indexes)

            print(indexes)
            
            min_distance = math.inf
            best_index = None
            for index in indexes:
                pos = Position2D(index)
                dist = pos.get_distance_to(robot_array_index)
                if dist < min_distance:
                    min_distance = dist
                    best_index = index
            
            return Position2D(self.mapper.pixel_grid.array_index_to_coordinates(best_index))
        else:
            self.mapper.pixel_grid.arrays["fixture_detection"][fixture_array_index[0], fixture_array_index[1]] = True
            return None
        










