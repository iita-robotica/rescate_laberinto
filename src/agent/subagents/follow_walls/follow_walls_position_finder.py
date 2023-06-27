import numpy as np
import cv2 as cv
import math
from data_structures.vectors import Position2D

from algorithms.np_bool_array.bfs import NavigatingBFSAlgorithm

from agent.agent_interface import PositionFinderInterface
from mapping.mapper import Mapper


class PositionFinder(PositionFinderInterface):
    def __init__(self, mapper: Mapper) -> None:
        self.__mapper = mapper
        self.__next_position_finder = NavigatingBFSAlgorithm(lambda x: x, lambda x: not x)
        self.__still_reachable_bfs = NavigatingBFSAlgorithm(lambda x: x, lambda x: not x)
        self.__target = None

        smoother_template_radious = int(0.03 * self.__mapper.pixel_grid.resolution)
        smoother_template_diameter = math.ceil(smoother_template_radious * 2 + 1)

        self.min_number_to_be_valid = 10

        # Solid disk to smooth margins
        self.smoother_template = np.zeros((smoother_template_diameter, smoother_template_diameter), dtype=np.int8)
        self.smoother_template = cv.circle(self.smoother_template, (smoother_template_radious, smoother_template_radious), smoother_template_radious, 1, -1)
        self.smoother_template[smoother_template_radious, smoother_template_radious] = 0


    def update(self, force_calculation=False) -> None:
        if  not self.target_position_exists() or \
            not self.__is_grid_index_still_reachable(self.__target) or \
            self.__already_passed_through_grid_index(self.__target) or \
            force_calculation:
            
            self.__calculate_position()

    def get_target_position(self) -> Position2D:
        if self.target_position_exists():
            return self.__mapper.pixel_grid.grid_index_to_coordinates(self.__target)
    
    def target_position_exists(self) -> bool:
        #print("victim_target_exists!")
        return self.__target is not None

    def __calculate_position(self):
        possible_targets_array = self.__mapper.pixel_grid.arrays["fixture_distance_margin"]
        isolated = cv.filter2D(self.__mapper.pixel_grid.arrays["fixture_distance_margin"].astype(np.uint8), -1, self.smoother_template) < self.min_number_to_be_valid
        #cv.imshow("isolated", isolated.astype(np.uint8) * 255)
        possible_targets_array[isolated] = False
        self.__dither_array(possible_targets_array, dither_interval=2)
        possible_targets_array[self.__mapper.pixel_grid.arrays["robot_center_traversed"]] = False
        self.__mapper.pixel_grid.arrays["fixture_distance_margin"][self.__mapper.pixel_grid.arrays["traversable"]] = False
        self.__mapper.pixel_grid.arrays["fixture_distance_margin"][self.__mapper.pixel_grid.arrays["swamps"]] = False

        

        #cv.imshow("possible wall targets", possible_targets_array.astype(np.uint8) * 255)

        if not np.any(possible_targets_array):
            print("no tragets")
            self.__target = None
            return

        robot_array_index = self.__mapper.pixel_grid.grid_index_to_array_index(self.__mapper.robot_grid_index)

        results = self.__next_position_finder.bfs(possible_targets_array, self.__mapper.pixel_grid.arrays["traversable"], robot_array_index)

        self.__target = self.__mapper.pixel_grid.array_index_to_grid_index(results[0]) if len(results) else None

    
    def __is_grid_index_still_reachable(self, grid_index) -> bool:
        start_array_index = self.__mapper.pixel_grid.grid_index_to_array_index(grid_index)

        if self.__mapper.pixel_grid.arrays["traversable"][start_array_index[0], start_array_index[1]]:
             return False

        results = self.__still_reachable_bfs.bfs(self.__mapper.pixel_grid.arrays["traversed"], self.__mapper.pixel_grid.arrays["traversable"], start_array_index)

        return bool(len(results))
    
    def __already_passed_through_grid_index(self, grid_index):
        array_index = self.__mapper.pixel_grid.grid_index_to_array_index(grid_index)

        return self.__mapper.pixel_grid.arrays["robot_center_traversed"][array_index[0], array_index[1]]
    

    def __dither_array(self, possible_targets_array: np.ndarray, dither_interval=2):
        mask = np.ones_like(possible_targets_array)

        mask[::dither_interval, ::dither_interval] = False

        mask[dither_interval//2::dither_interval, dither_interval//2::dither_interval] = False

        #cv.imshow("dither_mask", (mask == False).astype(np.uint8) * 255)

        possible_targets_array[mask] = False
