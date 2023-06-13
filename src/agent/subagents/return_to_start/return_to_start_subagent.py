import numpy as np

from data_structures.vectors import Position2D

from agent.agent_interface import SubagentInterface
from mapping.mapper import Mapper

from agent.pathfinding.pathfinder import PathFinder


class ReturnToStartAgent(SubagentInterface):
    def __init__(self, mapper: Mapper) -> None:
        self.__mapper = mapper
        self.__pathfinder = PathFinder(self.__mapper)

    def update(self, force_calculation) -> None:
        self.__pathfinder.update(np.array(self.__mapper.start_position), force_calculation=force_calculation)

    def get_target_position(self) -> Position2D:
        return self.__pathfinder.get_next_position()

    def target_position_exists(self) -> bool:
        return self.__mapper.start_position is not None