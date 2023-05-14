from agents.agent import Agent
from data_structures.vectors import Position2D

from agents.victim_following_agent.position_finder import VictimPositionFinder


class VictimFollowingAgent(Agent):
    def __init__(self, mapper) -> None:
        super().__init__(mapper)
        self.__position = None
        self.position_finder = VictimPositionFinder(mapper)
        
    def update(self) -> None:
        self.__position = self.position_finder.get_next_position()

    def get_target_position(self) -> Position2D:
        return self.__position
    
    def do_end(self) -> bool:
        return False
    
    def do_report_fixture(self) -> bool:
        return False
    
    def get_fixture_letter(self) -> str:
        return "N"