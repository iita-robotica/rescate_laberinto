import numpy as np
import cv2 as cv

from agents.agent import Agent

from data_structures.vectors import Position2D
from mapping import Mapper

from agents.granular_navigation_agent.path_finder import PathFinder

class GranularNavigationAgent(Agent):
    def __init__(self):
        self.path_finder = PathFinder()
    
    def update(self, mapper: Mapper) -> None:
        self.path_finder.update(mapper=mapper, target_position=np.array([-0.0981416353934171, -0.018619773492883556]))
    
    def get_target_position(self, mapper: Mapper) -> Position2D:
        return self.path_finder.get_next_position()

    def do_end(self) -> bool:
        return False
    
    def do_report_victim(self) -> bool:
        return False
        