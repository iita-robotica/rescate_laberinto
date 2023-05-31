import numpy as np
from data_structures.vectors import Position2D

from algorithms.np_bool_array.bfs import NavigatingBFSAlgorithm

from agent.agent_interface import PositionFinderInterface
from mapping.mapper import Mapper


class PositionFinder(PositionFinderInterface):
    def __init__(self, mapper: Mapper) -> None:
        self.__mapper = mapper
        self.__next_position_finder = NavigatingBFSAlgorithm(lambda x: x, lambda x: not x)
        self.__still_reachable_bfs = NavigatingBFSAlgorithm(lambda x: x, lambda x: not x)
        self.__closest_free_point_finder = NavigatingBFSAlgorithm(lambda x : x == 0, lambda x: True)
        self.__target = None


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
        possible_targets_array = self.__mapper.pixel_grid.arrays["fixture_distance_margin"]
        self.__dither_array(possible_targets_array, dither_interval=2)
        possible_targets_array[self.__mapper.pixel_grid.arrays["robot_center_traversed"]] = False

        #cv.imshow("possible wall targets", possible_targets_array.astype(np.uint8) * 255)

        if not np.any(possible_targets_array):
            print("no tragets")
            return

        robot_array_index = self.__mapper.pixel_grid.grid_index_to_array_index(self.__mapper.robot_grid_index)

        robot_array_index = self.__get_closest_traversable_array_index(robot_array_index)

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

    

    def __get_closest_traversable_array_index(self, array_index):
        if self.__mapper.pixel_grid.arrays["traversable"][array_index[0], array_index[1]]:
            return  self.__closest_free_point_finder.bfs(found_array=self.__mapper.pixel_grid.arrays["traversable"],
                                                         traversable_array=self.__mapper.pixel_grid.arrays["traversable"],
                                                         start_node=array_index)[0]
        else:
            return array_index
