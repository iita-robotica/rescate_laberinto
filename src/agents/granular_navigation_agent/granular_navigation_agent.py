import numpy as np
import cv2 as cv

from agents.agent import Agent

import utilities
from data_structures.vectors import Position2D
from mapping import Mapper

from data_structures.compound_pixel_grid import PointGrid
from algorithms.np_bool_array.efficient_a_star import aStarAlgorithm
from algorithms.np_bool_array.bfs import BFSAlgorithm

from agents.granular_navigation_agent.path_smoothing import PathSmoother

from flags import SHOW_PATHFINDING_DEBUG, SHOW_GRANULAR_NAVIGATION_GRID

class GranularNavigationAgent(Agent):
    def __init__(self):
        self.a_star = aStarAlgorithm()
        self.closest_free_point_finder = BFSAlgorithm(lambda x : x == 0)

        self.a_star_path_smoother = PathSmoother(1)

        self.current_grid_index = np.array([0, 0])
        self.target_position = np.array([-0.0981416353934171, -0.018619773492883556]) # np.array([-0.5919078827365277, -0.14052309679063227])

        self.a_star_path = []
        self.smooth_astar_path = []
        self.a_star_index = 0
    
    def update(self, mapper: Mapper) -> None:
        # Get start index
        self.current_grid_index = mapper.granular_grid.coordinates_to_grid_index(mapper.robot_position.get_np_array())
        mapper.granular_grid.expand_grid_to_grid_index(self.current_grid_index)
        start_array_index = mapper.granular_grid.grid_index_to_array_index(self.current_grid_index)
        
        # If current position not traversable, find closest traversable position
        if mapper.granular_grid.arrays["traversable"][start_array_index[0], start_array_index[1]]:
            if SHOW_PATHFINDING_DEBUG: print("INITIAL POSITION NOT TRAVERSABLE, CALCULATING BFS")
            start_array_index = self.closest_free_point_finder.bfs(mapper.granular_grid.arrays["traversable"], start_array_index)
            n_trav = mapper.granular_grid.arrays["traversable"][start_array_index[0], start_array_index[1]]
            if SHOW_PATHFINDING_DEBUG: print("FINISHED CLACULATING BFS: ", n_trav)

        if len(self.a_star_path) <= self.a_star_index:
            if SHOW_PATHFINDING_DEBUG: print("FINISHED PATH")

        if not self.check_path(mapper.granular_grid):
            if SHOW_PATHFINDING_DEBUG: print("FAILED PATH CHECK")

        # If path finished or current path obstructed
        if len(self.a_star_path) - 1 <= self.a_star_index or not self.check_path(mapper.granular_grid):

            target_grid_index = mapper.granular_grid.coordinates_to_grid_index(self.target_position)
            mapper.granular_grid.expand_grid_to_grid_index(target_grid_index)
            target_array_index = mapper.granular_grid.grid_index_to_array_index(target_grid_index)

            if mapper.granular_grid.arrays["traversable"][target_array_index[0], target_array_index[1]]:
                target_array_index = self.closest_free_point_finder.bfs(mapper.granular_grid.arrays["traversable"], target_array_index)

            # Calculate path
            best_path = self.a_star.a_star(mapper.granular_grid.arrays["traversable"], 
                                           start_array_index,
                                           target_array_index,
                                           mapper.granular_grid.arrays["navigation_preference"])

            if len(best_path) > 1:
                self.a_star_path = []
                for array_index in best_path:
                    self.a_star_path.append(mapper.granular_grid.array_index_to_grid_index(array_index))

                self.a_star_path = self.a_star_path[1:]
                self.a_star_index = 0
            
            self.a_star_path = self.smooth_path(self.a_star_path)
            self.smooth_astar_path = self.a_star_path_smoother.smooth(self.a_star_path)

        if SHOW_GRANULAR_NAVIGATION_GRID:
            debug_grid = mapper.granular_grid.get_colored_grid()
            for node in self.a_star_path:
                n = np.array(mapper.granular_grid.grid_index_to_array_index(node))
                try:
                    debug_grid[n[0], n[1]] = [0, 0, 255]
                except IndexError:
                    pass

            cv.imshow("granular_grid", debug_grid)

        self.a_star_index = min(self.a_star_index, len(self.a_star_path) - 1)   
        if len(self.a_star_path) > 0:
            #print("a_star_index:", self.a_star_index)
            #print("path len:", len(self.a_star_path))
            next_node = self.a_star_path[self.a_star_index]
            next_node = Position2D(next_node[0], next_node[1])

            current_pos = mapper.robot_position
            current_pos = np.array([current_pos[0], current_pos[1]])
            current_grid_index = mapper.granular_grid.coordinates_to_grid_index(current_pos)
            current_node = Position2D(current_grid_index[0], current_grid_index[1])


            #print("current_array_index:", current_node)
            #print("next_array_indx:", next_node)

            #print("dist:", abs(current_node.get_distance_to(next_node)))

            if abs(current_node.get_distance_to(next_node)) < 3:
                self.a_star_index += 1

    def smooth_path(self, path):
        final_path = []
        dither_interval = 2
        for index, value in enumerate(path):
            if index % dither_interval == 0:
                final_path.append(value)
        if len(final_path):
            return final_path
        else:
            return path
    
    def get_target_position(self, mapper: Mapper) -> Position2D:
        self.a_star_index = min(self.a_star_index, len(self.a_star_path) -1)
        #print(self.a_star_path[self.a_star_index])
        pos = mapper.granular_grid.grid_index_to_coordinates(np.array(self.smooth_astar_path[self.a_star_index]))
        pos = Position2D(pos[0], pos[1])
        #print("target_position:", pos)
        #print("robot_position:", mapper.robot_position)
        return pos

    def do_end(self) -> bool:
        return False
    
    def do_report_victim(self) -> bool:
        return False
    
    # Is current Astar path obstructed?
    def check_path(self, granular_grid: PointGrid):
        array_index_path = []
        for n in self.a_star_path:
            array_index_path.append(granular_grid.grid_index_to_array_index(n))
            
        for position in array_index_path:
            if position[0] >= granular_grid.arrays["traversable"].shape[0] or position[1] >= granular_grid.arrays["traversable"].shape[1]:
                continue

            if position[0] < 0 or position[1] < 0:
                continue

            if granular_grid.arrays["traversable"][position[0], position[1]]:
                return False
        return True
        