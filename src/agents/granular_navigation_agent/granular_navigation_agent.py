import numpy as np
import cv2 as cv

from agents.agent import Agent

from data_structures.vectors import Position2D
from mapping.mapper import Mapper

from agents.granular_navigation_agent.path_finder import PathFinder
from agents.granular_navigation_agent.best_position_finder import BestPositionFinder
from agents.granular_navigation_agent.victim_position_finder import VictimPositionFinder

class GranularNavigationAgent(Agent):
    """
    Navigates the map without any concept of 'tiles'.
    """
    def __init__(self, mapper: Mapper):
        self.path_finder = PathFinder(mapper)
        self.best_position_finder = BestPositionFinder(mapper)
        self.victim_position_finder = VictimPositionFinder(mapper)
        self.best_position = None
        self.mapper = mapper
        self.__end = False
    
    def update(self) -> None:
        self.best_position_finder.calculate_best_position(finished_path=self.path_finder.is_path_finished() or self.path_finder.path_not_found)
        self.victim_position_finder.update()

        if self.victim_position_finder.is_there_victims():
            self.best_position = self.victim_position_finder.get_victim_position()
        else:
            self.best_position = self.best_position_finder.get_best_position()

        self.path_finder.update(target_position=self.best_position)#np.array(Position2D(-0.08884384679907074, -0.01975882018000104)))

        if self.path_finder.is_path_finished() and \
           self.best_position == self.mapper.start_position and \
           self.best_position.get_distance_to(self.mapper.robot_position) < 0.04:
            self.__end = True

        
    def get_target_position(self) -> Position2D:
        return self.path_finder.get_next_position()

    def do_end(self) -> bool:
        return self.__end
    
    def do_report_victim(self) -> bool:
        if self.victim_position_finder.is_close_to_victim():
            self.mapper.fixture_detector.mark_reported_fixture(self.mapper.robot_position, self.victim_position_finder.get_victim_position())
        return self.victim_position_finder.is_close_to_victim()
        