import numpy as np
import cv2 as cv

from agent.agent_interface import SubagentInterface

from data_structures.vectors import Position2D
from mapping.mapper import Mapper

from agent.pathfinding.pathfinder import PathFinder
from agent.subagents.go_to_non_discovered.go_to_non_discovered_position_finder import PositionFinder

class GoToNonDiscoveredAgent(SubagentInterface):
    """
    Navigates the map without any concept of 'tiles'.
    """
    def __init__(self, mapper: Mapper):
        self.__path_finder = PathFinder(mapper)
        self.__position_finder = PositionFinder(mapper)
    
    def update(self, force_calculation=False) -> None:
        self.__position_finder.update(force_calculation=self.__do_force_position_finder() or force_calculation)

        if self.__position_finder.target_position_exists():
            target = self.__position_finder.get_target_position()
            self.__path_finder.update(target_position=np.array(target), force_calculation=force_calculation)

    def get_target_position(self) -> Position2D:
        return self.__path_finder.get_next_position()    
    
    def __do_force_position_finder(self) -> bool:
        return self.__path_finder.is_path_finished() or self.__path_finder.path_not_found
    
    def target_position_exists(self) -> bool:
        return self.__position_finder.target_position_exists()
        