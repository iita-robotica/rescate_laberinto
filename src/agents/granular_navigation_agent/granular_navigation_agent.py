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
        self.current_robot_node = [0, 0]
        self.previous_robot_node = [0, 0]
        self.current_granular_grid_index = Position2D()
        self.target_node = [None, None]
        self.a_star_path = []
        self.a_star_index = 0
    
    def update(self, mapper: Mapper) -> None:
        robot_node = self.find_robot_node(mapper.node_grid)
        self.current_granular_grid_index = mapper.granular_grid.coordinates_to_index(mapper.robot_position.get_np_array())
        self.current_granular_grid_index = Position2D(self.current_granular_grid_index[0], self.current_granular_grid_index[1])
        
        if robot_node != self.current_robot_node:
            self.previous_robot_node = self.current_robot_node
            self.current_robot_node = robot_node
        if self.previous_robot_node is None:
            self.previous_robot_node = self.current_robot_node

        if len(self.a_star_path) <= self.a_star_index:
            print("FINISHED PATH")

        if not self.check_path(mapper.granular_grid):
            print("FAILED CHECK")

        if len(self.a_star_path) - 1 <= self.a_star_index or not self.check_path(mapper.granular_grid):
            if is_traversable(mapper.node_grid, self.current_robot_node):
                possible_nodes = bfs(mapper.node_grid, self.current_robot_node, limit=100)
            else:
                possible_nodes = bfs(mapper.node_grid, self.previous_robot_node, limit=100)

            #print("Possible nodes:", possible_nodes)
            if len(possible_nodes):
                self.target_node = self.get_best_node(possible_nodes)
            else:
                self.target_node = self.find_start_node(mapper.node_grid)

            target_position = mapper.node_grid_index_to_position(self.target_node)
            target_granular_grid_index = mapper.granular_grid.coordinates_to_index(target_position.get_np_array())

            best_path = self.a_star.a_star(mapper.granular_grid,  self.current_granular_grid_index.get_np_array(), target_granular_grid_index)

            if len(best_path) > 1:
                self.a_star_path = best_path[1:]
                self.a_star_index = 0

        if SHOW_GRANULAR_NAVIGATION_GRID:
            debug_grid = mapper.granular_grid.get_colored_grid()
            for node in self.a_star_path:
                n = mapper.granular_grid.get_point(np.array(node))
                debug_grid[n[0], n[1]] = [0, 0, 255]

            cv.imshow("granular_grid", debug_grid)

        self.a_star_index = min(self.a_star_index, len(self.a_star_path))   
        if len(self.a_star_path) > 0:
            print("a_star_index:", self.a_star_index)
            print("path len:", len(self.a_star_path))
            next_node = self.a_star_path[self.a_star_index]
            next_node = Position2D(next_node[0], next_node[1])

            if abs(self.current_granular_grid_index.get_distance_to(next_node)) < 3:
                self.a_star_index += 1

        

        if SHOW_DEBUG:
            print("Best node:", self.target_node)
            print("Start node:", self.current_robot_node)
            print("AStar path: ", self.a_star_path)
    
    def get_target_position(self, mapper: Mapper) -> Position2D:
        self.a_star_index = min(self.a_star_index, len(self.a_star_path) -1)
        pos = mapper.granular_grid.index_to_coordinates(np.array(self.a_star_path[self.a_star_index]))
        return Position2D(pos[0], pos[1])


    def do_end(self) -> bool:
        return False
    
    def do_report_victim(self) -> bool:
        return False
    
    def find_robot_node(self, node_grid):
        for y, row in enumerate(node_grid.grid):
            for x, node in enumerate(row):
                if node.is_robots_position:
                    return [x - node_grid.offsets[0], y - node_grid.offsets[1]]
    
    def find_start_node(self, node_grid):
        for y, row in enumerate(node_grid.grid):
            for x, node in enumerate(row):
                if node.is_start:
                    return [x - node_grid.offsets[0], y - node_grid.offsets[1]]
                
    def get_best_node(self, possible_nodes):
        if len(possible_nodes) > 0:
            best_node = possible_nodes[0]
            if best_node[:2] == list(self.current_robot_node):
                best_node = possible_nodes[1]

            orientation = utilities.substractLists(self.current_robot_node, self.previous_robot_node)
            forward_node = utilities.sumLists(self.current_robot_node, orientation)
            for node in possible_nodes[:10]:
                if list(node[:2]) == list(forward_node):
                    best_node = forward_node

        else:
            best_node = self.current_robot_node
        #return possibleNodes[-1][:2]
        return Position2D(best_node[0], best_node[1])# best_node[:2]
    
    def check_path(self, granular_grid: PointGrid):
        for position in self.a_star_path:
            p =  granular_grid.get_point(np.array([position[0], position[1]]))
            if granular_grid.traversable_grid[p[0], p[1]]:
                return False
        return True
        