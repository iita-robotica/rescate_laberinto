import numpy as np
import cv2 as cv
from data_structures.vectors import Position2D

from algorithms.np_bool_array.bfs import NavigatingLimitedBFSAlgorithm, NavigatingBFSAlgorithm

from agent.agent_interface import PositionFinderInterface
from mapping.mapper import Mapper


class PositionFinder(PositionFinderInterface):
    def __init__(self, mapper: Mapper) -> None:
        self.__mapper = mapper
        self.__next_position_finder = NavigatingLimitedBFSAlgorithm(lambda x: x, lambda x: not x, limit=1000)
        self.__still_reachable_bfs = NavigatingBFSAlgorithm(lambda x: x, lambda x: not x)
        self.__target = None

        circle_radius = round(self.__mapper.robot_diameter / 2 * self.__mapper.pixel_grid.resolution) + 3

        self.circle_kernel = np.zeros((circle_radius * 2, circle_radius * 2), dtype=np.uint8)
        self.circle_kernel = cv.circle(self.circle_kernel, (circle_radius, circle_radius), circle_radius, 1, -1)


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
        return self.__target is not None

    def __calculate_position(self):
        possible_targets_array = self.__get_fixtures_zone_of_influence()
        possible_targets_array += self.__mapper.pixel_grid.arrays["checkpoints"]
        possible_targets_array[self.__mapper.pixel_grid.arrays["robot_center_traversed"]] = False
        
        """
        debug = self.__mapper.pixel_grid.get_colored_grid()
        debug *= 0.3
        debug[possible_targets_array] = (0, 255, 0)
        cv.imshow("possible fixture targets", debug)
        """
        

        if not np.any(possible_targets_array):
            #print("no fixture targets")
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
    
    
    def __get_fixtures_zone_of_influence(self) -> np.ndarray:
        zones = cv.filter2D(self.__mapper.pixel_grid.arrays["victims"].astype(np.uint8), -1, self.circle_kernel).astype(np.bool_)

        return np.bitwise_and(zones, self.__mapper.pixel_grid.arrays["fixture_distance_margin"])