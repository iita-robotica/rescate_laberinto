import numpy as np
import cv2 as cv

from agents.agent import Agent

import utilities
from data_structures.vectors import Position2D
from mapping import Mapper

from data_structures.compound_pixel_grid import PointGrid
from algorithms.compound_pixel_grid.a_star import aStarAlgorithm
from algorithms.expandable_node_grid.bfs import bfs
from algorithms.expandable_node_grid.traversable import is_traversable

from flags import SHOW_DEBUG,SHOW_GRANULAR_NAVIGATION_GRID

class GranularNavigationAgent(Agent):
    def __init__(self):
        self.a_star = aStarAlgorithm()

        self.current_grid_index = np.array([0, 0])
        self.target_position = np.array([-0.018619773492883556, -0.0981416353934171])

        self.a_star_path = []
        self.a_star_index = 0
    
    def update(self, mapper: Mapper) -> None:
        self.current_grid_index = mapper.granular_grid.coordinates_to_grid_index(mapper.robot_position.get_np_array())

        mapper.granular_grid.expand_grid_to_grid_index(self.current_grid_index)
        start_array_index = mapper.granular_grid.grid_index_to_array_index(self.current_grid_index)
        

        if len(self.a_star_path) <= self.a_star_index:
            print("FINISHED PATH")

        if not self.check_path(mapper.granular_grid.traversable_grid):
            print("FAILED CHECK")

        if len(self.a_star_path) - 1 <= self.a_star_index or not self.check_path(mapper.granular_grid.traversable_grid):

            target_grid_index = mapper.granular_grid.coordinates_to_grid_index(self.target_position)
            mapper.granular_grid.expand_grid_to_grid_index(target_grid_index)
            target_array_index = mapper.granular_grid.grid_index_to_array_index(target_grid_index)

            best_path = self.a_star.a_star(mapper.granular_grid.traversable_grid, 
                                           start_array_index,
                                           target_array_index)

            if len(best_path) > 1:
                self.a_star_path = best_path[1:]
                self.a_star_index = 0

        if SHOW_GRANULAR_NAVIGATION_GRID:
            debug_grid = mapper.granular_grid.get_colored_grid()
            for node in self.a_star_path:
                n = np.array(node)
                debug_grid[n[0], n[1]] = [0, 0, 255]

            cv.imshow("granular_grid", debug_grid)

        self.a_star_index = min(self.a_star_index, len(self.a_star_path))   
        if len(self.a_star_path) > 0:
            print("a_star_index:", self.a_star_index)
            print("path len:", len(self.a_star_path))
            next_node = self.a_star_path[self.a_star_index]
            next_node = Position2D(next_node[0], next_node[1])

            current_pos = mapper.robot_position
            current_pos = np.array([current_pos[0], current_pos[1]])
            current_grid_index = mapper.granular_grid.coordinates_to_grid_index(current_pos)
            current_node = mapper.granular_grid.grid_index_to_array_index(current_grid_index)
            current_node = Position2D(current_node[0], current_node[1])


            print("current_array_index:", current_node)
            print("next_array_indx:", next_node)

            print("dist:", abs(current_node.get_distance_to(next_node)))

            if abs(current_node.get_distance_to(next_node)) < 3:
                self.a_star_index += 1


        if SHOW_DEBUG:
            print("Best node:", self.target_node)
            print("Start node:", self.current_robot_node)
            print("AStar path: ", self.a_star_path)
    
    def get_target_position(self, mapper: Mapper) -> Position2D:
        self.a_star_index = min(self.a_star_index, len(self.a_star_path) -1)
        print(self.a_star_path[self.a_star_index])
        index = mapper.granular_grid.array_index_to_grid_index(np.array(self.a_star_path[self.a_star_index]))
        pos = mapper.granular_grid.grid_index_to_coordinates(np.array(index))
        pos = Position2D(pos[0], pos[1])
        print("target_position:", pos)
        print("robot_position:", mapper.robot_position)
        return pos

    def do_end(self) -> bool:
        return False
    
    def do_report_victim(self) -> bool:
        return False
    
    
    def check_path(self, granular_grid: np.ndarray):
        for position in self.a_star_path:
            if granular_grid[position[0], position[1]]:
                return False
        return True
        