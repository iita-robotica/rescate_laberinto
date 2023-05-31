from mapping.mapper import Mapper
from data_structures.vectors import Position2D
import numpy as np
import cv2 as cv

from algorithms.np_bool_array.bfs import BFSAlgorithm, NavigatingBFSAlgorithm
from flags import SHOW_BEST_POSITION_FINDER_DEBUG

from agent.agent_interface import PositionFinderInterface

class PositionFinder(PositionFinderInterface):
    """
    Finds the best position for the robot to go to, with the objective of exploring the maze.
    """
    def __init__(self, mapper: Mapper) -> None:
        self.mapper = mapper
        self.closest_unseen_finder = NavigatingBFSAlgorithm(found_function=lambda x: x == False, 
                                                            traversable_function=lambda x: x == False,
                                                            max_result_number=1)
        
        self.closest_free_point_finder = BFSAlgorithm(lambda x : x == 0)
        
        self.closest_unseen_grid_index = None
        

    def update(self, force_calculation=False):
        """
        Calculate closest unseen position if the objective is no longer traversable or if it's told to do it with the 'force' parameter.
        """
        if self.__is_objective_untraversable() or force_calculation:
            self.closest_unseen_grid_index = self.__get_closest_unseen_grid_index()

        # DEBUG
        if SHOW_BEST_POSITION_FINDER_DEBUG:
            debug_grid = self.mapper.pixel_grid.get_colored_grid()  
            if self.target_position_exists():
                closest_unseen_array_index = self.mapper.pixel_grid.grid_index_to_array_index(self.closest_unseen_grid_index)
                cv.circle(debug_grid, (closest_unseen_array_index[1], closest_unseen_array_index[0]), 4, (0, 255, 100), -1)
                cv.imshow("closest_position_finder_debug", debug_grid)

    def get_target_position(self):
        if self.target_position_exists():
            coords = self.mapper.pixel_grid.grid_index_to_coordinates(self.closest_unseen_grid_index)
            return Position2D(coords)
        
    def target_position_exists(self) -> bool:
        return self.closest_unseen_grid_index is not None
    
    def __is_objective_untraversable(self):
        if self.target_position_exists():
            closest_unseen_array_index = self.mapper.pixel_grid.grid_index_to_array_index(self.closest_unseen_grid_index)
            return self.mapper.pixel_grid.arrays["traversable"][closest_unseen_array_index[0], closest_unseen_array_index[1]]
        else:
            return False
    
    def __get_closest_unseen_grid_index(self):
        robot_array_index = self.mapper.pixel_grid.coordinates_to_array_index(self.mapper.robot_position)
        start_node = self.__get_closest_traversable_array_index(robot_array_index)

        closest_unseen_array_indexes = self.closest_unseen_finder.bfs(found_array=self.mapper.pixel_grid.arrays["discovered"],
                                                                      traversable_array=self.mapper.pixel_grid.arrays["traversable"],
                                                                      start_node=start_node)
        
        if len(closest_unseen_array_indexes):
            print("found not discovered")
            return self.mapper.pixel_grid.array_index_to_grid_index(closest_unseen_array_indexes[0])
        
        else:
            print("Ain't found no nothin'")
            return None


    def __get_closest_traversable_array_index(self, array_index):
        if self.mapper.pixel_grid.arrays["traversable"][array_index[0], array_index[1]]:
            return  self.closest_free_point_finder.bfs(array=self.mapper.pixel_grid.arrays["traversable"],
                                                       start_node=array_index)
        else:
            return array_index


        


    
