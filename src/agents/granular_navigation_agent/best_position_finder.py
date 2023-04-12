
from mapping import Mapper
from data_structures.vectors import Position2D
import numpy as np
import cv2 as cv
import math
import skimage
from copy import copy, deepcopy
from algorithms.np_bool_array.bfs import BFSAlgorithm, NavigatingBFSAlgorithm

class BestPositionFinder:
    def __init__(self, mapper: Mapper) -> None:
        self.mapper = mapper
        self.closest_not_seen_grid_index = None
        self.closest_not_seen_finder = NavigatingBFSAlgorithm(found_function=lambda x: x == False, 
                                                              traversable_function=lambda x: x == False)

    def calculate_best_position(self, finished_path):
        if self.closest_not_seen_grid_index is None:
            self.closest_not_seen_grid_index = self.get_best_grid_index()

        closest_not_seen_array_index = self.mapper.granular_grid.grid_index_to_array_index(self.closest_not_seen_grid_index)


        objective_not_traversable = self.mapper.granular_grid.arrays["traversable"][closest_not_seen_array_index[0], closest_not_seen_array_index[1]]

        if objective_not_traversable or finished_path:
            self.closest_not_seen_grid_index = self.get_best_grid_index()

                
        debug_grid = self.mapper.granular_grid.get_colored_grid()    
        cv.circle(debug_grid, (closest_not_seen_array_index[1], closest_not_seen_array_index[0]), 4, (0, 255, 100), -1)
        cv.imshow("closest_position_finder_debug", debug_grid)
    

    def get_best_grid_index(self):
        print("Calculando punto")
        robot_grid_index = self.mapper.granular_grid.coordinates_to_grid_index(np.array(self.mapper.robot_position))
        robot_array_index = self.mapper.granular_grid.grid_index_to_array_index(robot_grid_index)

        closest_not_seen_array_index = self.closest_not_seen_finder.bfs(found_array=self.mapper.granular_grid.arrays["seen_by_camera"],
                                                                        traversable_array=self.mapper.granular_grid.arrays["traversable"],
                                                                        start_node=robot_array_index)
        
        return self.mapper.granular_grid.array_index_to_grid_index(closest_not_seen_array_index)

    def get_best_position(self):
        if self.closest_not_seen_grid_index is not None:
            coords = self.mapper.granular_grid.grid_index_to_coordinates(self.closest_not_seen_grid_index)
            return Position2D(coords)
        else:
            return self.mapper.robot_position
    

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


        


    
