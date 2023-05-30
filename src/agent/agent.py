from collections import namedtuple
from data_structures.vectors import Position2D

from agent.agent_interface import AgentInterface, SubAgentInterface
from mapping.mapper import Mapper

from agent.sub_agents.follow_walls.follow_walls_sub_agent import FollowWallsAgent
from agent.sub_agents.go_to_non_discovered.go_to_non_discovered_sub_agent import GoToNonDiscoveredAgent
from agent.sub_agents.return_to_start.return_to_start_sub_agent import ReturnToStartAgent

class Agent(AgentInterface):
    FOLLOW_WALL_AGENT_KEY = 0
    GO_TO_NON_DISCOVERED_AGENT_KEY = 1
    RETURN_TO_START_AGENT_KEY = 2

    def __init__(self, mapper: Mapper) -> None:
        self.__mapper = mapper

        self.__current_agent_key = self.FOLLOW_WALL_AGENT_KEY
        self.__previous_agent_key = self.FOLLOW_WALL_AGENT_KEY

        self.__subagents = {
            self.FOLLOW_WALL_AGENT_KEY: FollowWallsAgent(self.__mapper),
            self.GO_TO_NON_DISCOVERED_AGENT_KEY: GoToNonDiscoveredAgent(self.__mapper),
            self.RETURN_TO_START_AGENT_KEY: ReturnToStartAgent(self.__mapper)
        }

        self.__subagent_keys_priority = (
            self.FOLLOW_WALL_AGENT_KEY, 
            self.GO_TO_NON_DISCOVERED_AGENT_KEY,
            self.RETURN_TO_START_AGENT_KEY)
        
        self.ConditionalSubagent = namedtuple("ConditionalSubagent", ("agent_key", "condition"))

        # TODO add flag when no position is found the first time, to avoid recalculating
        self.__conditional_subagents = (
            self.ConditionalSubagent(agent_key=self.RETURN_TO_START_AGENT_KEY, condition=self.__little_time_left),
        )

        self.end_reached_distance_threshold = 0.04

    def update(self) -> None:
        if self.agent_changed(): print("CHANGING AGENT")

        # TODO make list of alreay updated agents to not repeat updates

        for agent_key, condition in self.__conditional_subagents:
            self.__subagents[agent_key].update(force_calculation=self.agent_changed())
            if condition() and self.__subagents[agent_key].target_position_exists():
                self.__previous_agent_key = self.__current_agent_key
                self.__current_agent_key = agent_key
                print("using_agent:", agent_key)
                return
        
        for agent_key in self.__subagent_keys_priority:
            self.__subagents[agent_key].update(force_calculation=self.agent_changed())
            if self.__subagents[agent_key].target_position_exists():
                self.__previous_agent_key = self.__current_agent_key
                self.__current_agent_key = agent_key
                print("using_agent:", agent_key)
                return

    def get_target_position(self) -> Position2D:
        return self.__subagents[self.__current_agent_key].get_target_position()
    
    def do_end(self) -> bool:
        return self.__current_agent_key == self.RETURN_TO_START_AGENT_KEY and \
               self.__mapper.robot_position.get_distance_to(self.__mapper.start_position) < self.end_reached_distance_threshold
    
    def __little_time_left(self) -> bool:
        return False
    
    def agent_changed(self) -> bool:
        return self.__previous_agent_key != self.__current_agent_key

    