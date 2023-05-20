import numpy as np
import cv2 as cv

from agents.agent import Agent
from data_structures.vectors import Position2D

from agents.victim_following_agent.position_finder import VictimPositionFinder
from agents.victim_following_agent.path_finder import PathFinder


class VictimFollowingAgent(Agent):
    def __init__(self, mapper) -> None:
        super().__init__(mapper)
        self.__position = None
        self.position_finder = VictimPositionFinder(mapper)
        self.path_finder = PathFinder(mapper)
        
    def update(self) -> None:
        target = self.position_finder.get_next_position()
        if target is None:
            self.__position = None
        else:
            self.path_finder.update(np.array(target))
            self.__position = self.path_finder.get_next_position()

    def get_target_position(self) -> Position2D:
        return self.__position
    
    def do_end(self) -> bool:
        return False
    
    def do_report_fixture(self) -> bool:
        return False
    
    def get_fixture_letter(self) -> str:
        return "N"