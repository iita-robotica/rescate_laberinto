
from mapping import Mapper
from data_structures.vectors import Position2D
import numpy as np
import cv2 as cv
import math
import skimage
from copy import copy, deepcopy
from algorithms.np_bool_array.bfs import BFSAlgorithm, NavigatingBFSAlgorithm

class BestPositionFinder:
    def __init__(self) -> None:
        self.mapper = None
        self.closest_not_seen_wall_index = None
        self.closest_not_seen_finder = NavigatingBFSAlgorithm(found_function=lambda x: x == True, 
                                                              traversable_function=lambda x: x == False)

    def update(self, mapper: Mapper):
        self.mapper = mapper

    def calculate_best_position(self):
        not_seen_wall_indexes = np.nonzero(self.mapper.granular_grid.arrays["walls_not_seen_by_camera"])
        robot_grid_index = self.mapper.granular_grid.coordinates_to_grid_index(self.mapper.robot_position.get_np_array())
        robot_array_index = self.mapper.granular_grid.grid_index_to_array_index(robot_grid_index)

        self.closest_not_seen_wall_index = None

        if len(not_seen_wall_indexes[0]):
            self.closest_not_seen_wall_index = self.closest_not_seen_finder.bfs(self.mapper.granular_grid.arrays["walls_not_seen_by_camera"],
                                                                                self.mapper.granular_grid.arrays["occupied"],
                                                                                robot_array_index)
            
        debug_grid = self.mapper.granular_grid.get_colored_grid()    
        if self.closest_not_seen_wall_index is not None:
            
            debug_grid = cv.circle(debug_grid, (self.closest_not_seen_wall_index[1], self.closest_not_seen_wall_index[0]), 4, (0, 255, 100), -1)
        cv.imshow("closest_position_finder_debug", debug_grid)

    def get_best_position(self):
        if self.closest_not_seen_wall_index is None:
            return None
        else:
            closest_not_seen_grid_index = self.mapper.granular_grid.array_index_to_grid_index(self.closest_not_seen_wall_index)
            return self.mapper.granular_grid.grid_index_to_coordinates(np.array(closest_not_seen_grid_index))
    

    def __get_line(self, point1, point2):
        xx, yy = skimage.draw.line(point1[0], point1[1], point2[0], point2[1])
        indexes = [[x, y] for x, y in zip(xx, yy)]

    def __has_line_of_sight(self, point1, point2, matrix):
        xx, yy = skimage.draw.line(point1[0], point1[1], point2[0], point2[1])
        for x, y in zip(xx[1:-1], yy[1:-1]):
            if matrix[x, y]:
                return False
        return True
    
    def __get_seen_circle(self, radius, center_point, matrix):
        xx, yy = skimage.draw.circle(center_point, radius)
        indexes = [[x, y] for x, y in zip(xx, yy)]

        farthest_points = deepcopy(indexes)

        for index, current_farthest_point in enumerate(indexes):
            for possible_farthest_point in self.get_line(current_farthest_point, center_point):
                if self.__has_line_of_sight(possible_farthest_point, center_point, matrix):
                    farthest_points[index] = possible_farthest_point
                    break
        
        return farthest_points


        


    
