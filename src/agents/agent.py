import random
from data_structures.vectors import Position2D
from abc import ABC, abstractmethod

class Agent(ABC):
    def __init__(self, mapper) -> None:
        self.mapper = mapper

    @abstractmethod
    def update(self) -> None:
        pass
    
    @abstractmethod
    def get_target_position(self) -> Position2D:
        pass

    @abstractmethod
    def do_end(self) -> bool:
        pass

    @abstractmethod
    def do_report_victim(self) -> bool:
        pass