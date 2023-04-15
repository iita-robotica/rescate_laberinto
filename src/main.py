from executor.executor import Executor
from agents.granular_navigation_agent.granular_navigation_agent import GranularNavigationAgent
from mapping.mapper import Mapper
from robot.robot import Robot

def main():
    robot = Robot(time_step=32)
    mapper = Mapper(tile_size=0.006, robot_diameter=robot.diameter)
    agent = GranularNavigationAgent(mapper=mapper)

    executor = Executor(agent, mapper, robot)

    executor.run()


main()