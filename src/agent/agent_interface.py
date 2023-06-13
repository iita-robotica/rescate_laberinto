import random
from data_structures.vectors import Position2D
from abc import ABC, abstractmethod

class AgentInterface(ABC):
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

class SubagentInterface(ABC):
    def __init__(self, mapper) -> None:
        self.mapper = mapper

    @abstractmethod
    def update(self, force_calculation=False) -> None:
        pass
    
    @abstractmethod
    def get_target_position(self) -> Position2D:
        pass

    @abstractmethod
    def target_position_exists(self) -> bool:
        pass

class PositionFinderInterface(ABC):
    @abstractmethod
    def __init__(self, mapper) -> None:
        pass

    @abstractmethod
    def update(self, force_calculation=False) -> None:
        pass

    @abstractmethod
    def get_target_position(self) -> Position2D:
        pass

    @abstractmethod
    def target_position_exists(self) -> bool:
        pass