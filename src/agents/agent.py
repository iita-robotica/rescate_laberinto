import random
from data_structures.vectors import Position2D
from abc import ABC, abstractmethod

class Agent(ABC):
    def __init__(self) -> None:
        pass

    @abstractmethod
    def update(self, mapper) -> None:
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