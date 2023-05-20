import numpy as np

from agents.agent import Agent
from data_structures.vectors import Position2D
from agents.wall_following_agent.position_finder import PositionFinder
from agents.wall_following_agent.path_finder import PathFinder

from mapping.mapper import Mapper

class WallFollowingAgent(Agent):
    def __init__(self, mapper: Mapper) -> None:
        self.mapper = mapper
        self.__position_finder = PositionFinder(mapper)
        self.__pathfinder = PathFinder(mapper)

        self.__position = None

    def update(self):
        target = self.__position_finder.get_target_position()
        if target is None:
            self.__position = None
        else:
            self.__pathfinder.update(np.array(target))
            self.__position = self.__pathfinder.get_next_position()

    def get_target_position(self) -> Position2D:
        return self.__position

    def do_end(self) -> bool:
        return False

    def do_report_fixture(self) -> bool:
        return False

    def get_fixture_letter(self) -> str:
        return "N"