import numpy as np
import cv2 as cv

from agents.agent import Agent

from data_structures.vectors import Position2D
from mapping import Mapper

from agents.granular_navigation_agent.path_finder import PathFinder
from agents.granular_navigation_agent.best_position_finder import BestPositionFinder

class GranularNavigationAgent(Agent):
    def __init__(self):
        self.path_finder = PathFinder()
        self.best_position_finder = BestPositionFinder()
        self.best_position = None
    
    def update(self, mapper: Mapper) -> None:
        self.best_position_finder.update(mapper)
        #if not self.path_finder.check_path(mapper.granular_grid) or self.best_position is None:

       
        self.best_position_finder.calculate_best_position(finished_path=self.path_finder.finished_path)

        self.best_position = self.best_position_finder.get_best_position()
        self.path_finder.update(mapper=mapper, target_position=self.best_position)#Position2D(-0.08884384679907074, -0.01975882018000104).get_np_array())
    
    def get_target_position(self, mapper: Mapper) -> Position2D:
        return self.path_finder.get_next_position()

    def do_end(self) -> bool:
        return False
    
    def do_report_victim(self) -> bool:
        return False
        