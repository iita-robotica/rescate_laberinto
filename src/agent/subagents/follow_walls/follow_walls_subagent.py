import numpy as np
from data_structures.vectors import Position2D

from agent.agent_interface import SubagentInterface
from mapping.mapper import Mapper

from agent.subagents.follow_walls.follow_walls_position_finder import PositionFinder
from agent.pathfinding.pathfinder import PathFinder

class FollowWallsAgent(SubagentInterface):
    def __init__(self, mapper: Mapper) -> None:
        self.mapper = mapper
        self.__position_finder = PositionFinder(mapper)
        self.__pathfinder = PathFinder(mapper)

    def update(self, force_calculation=False):
        self.__position_finder.update(force_calculation=force_calculation)

        if self.__position_finder.target_position_exists():
            target = self.__position_finder.get_target_position()
            self.__pathfinder.update(np.array(target), force_calculation)  

    def get_target_position(self) -> Position2D:
        return self.__pathfinder.get_next_position()
    
    def target_position_exists(self) -> bool:
        return self.__position_finder.target_position_exists()