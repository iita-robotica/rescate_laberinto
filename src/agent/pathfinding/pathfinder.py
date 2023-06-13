import numpy as np
import cv2 as cv

from data_structures.vectors import Position2D
from mapping.mapper import Mapper

from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid
from algorithms.np_bool_array.efficient_a_star import aStarAlgorithm
from algorithms.np_bool_array.bfs import NavigatingBFSAlgorithm

from agent.pathfinding.path_smoothing import PathSmoother

from flags import SHOW_PATHFINDING_DEBUG, SHOW_GRANULAR_NAVIGATION_GRID

class PathFinder():
    def __init__(self, mapper: Mapper):
        self.__a_star = aStarAlgorithm()
        self.__closest_free_point_finder = NavigatingBFSAlgorithm(lambda x : x == 0, lambda x: True)

        self.__a_star_path_smoother = PathSmoother(1)

        self.__robot_grid_index = np.array([0, 0])
        self.__target_position = np.array([0, 0])

        self.__a_star_path = []
        self.__smooth_astar_path = []
        self.__a_star_index = 0

        self.__mapper = mapper

        self.path_not_found = False
        self.__position_changed = True
    
    def update(self, target_position: np.ndarray = None, force_calculation=False) -> None:
        if target_position is not None:
            self.__position_changed = np.any(self.__target_position != target_position)
            self.__target_position = target_position



        self.__robot_grid_index = self.__mapper.pixel_grid.coordinates_to_grid_index(self.__mapper.robot_position) # Get robot position grid index
        self.__mapper.pixel_grid.expand_to_grid_index(self.__robot_grid_index) # Expand grid to robot position

        if SHOW_PATHFINDING_DEBUG: 
            if self.is_path_finished(): print("FINISHED PATH")
            if self.__is_path_obstructed(): print("PATH OBSTRUCTED")

        if self.is_path_finished() or self.__is_path_obstructed() or self.__position_changed or force_calculation:
            self.__calculate_path()
            
        self.__calculate_path_index()

        #DEBUG
        if SHOW_GRANULAR_NAVIGATION_GRID:
            debug_grid = self.__mapper.pixel_grid.get_colored_grid()
            for node in self.__a_star_path:
                n = np.array(self.__mapper.pixel_grid.grid_index_to_array_index(node))
                try:
                    debug_grid[n[0], n[1]] = [0, 0, 255]
                except IndexError:
                    pass

            cv.imshow("granular_grid", debug_grid)
        


    def __calculate_path(self):
        # Get start array index (if robot index occupied, get closest unoccupied point)
        start_array_index = self.__mapper.pixel_grid.coordinates_to_array_index(self.__mapper.robot_position)
        start_array_index = self.__get_closest_traversable_array_index(start_array_index)

        # Expand grid to target index
        target_grid_index = self.__mapper.pixel_grid.coordinates_to_grid_index(self.__target_position)
        self.__mapper.pixel_grid.expand_to_grid_index(target_grid_index)

        # Get target array index (if target index occupied, get closest unoccupied point)
        target_array_index = self.__mapper.pixel_grid.coordinates_to_array_index(self.__target_position)
        target_array_index = self.__get_closest_traversable_array_index(target_array_index)

        # Calculate path
        best_path = self.__a_star.a_star(self.__mapper.pixel_grid.arrays["traversable"], 
                                        start_array_index,
                                        target_array_index,
                                        self.__mapper.pixel_grid.arrays["navigation_preference"])

        # If path was successfully calculated, transform all indexes to grid indexes
        if len(best_path) > 1:
            self.__a_star_path = []
            for array_index in best_path:
                self.__a_star_path.append(self.__mapper.pixel_grid.array_index_to_grid_index(array_index))

            self.__a_star_path = self.__a_star_path[1:]
            self.__a_star_index = 0
            self.path_not_found = False
        else:
            if SHOW_PATHFINDING_DEBUG: print("PATH NOT FOUND")
            print("PATH NOT FOUND")
            self.path_not_found = True
        
        self.__a_star_path = self.__dither_path(self.__a_star_path) # Remove every second positon of the path
        self.__smooth_astar_path = self.__a_star_path_smoother.smooth(self.__a_star_path) # Smooth the path

    def __calculate_path_index(self):
        self.__a_star_index = min(self.__a_star_index, len(self.__a_star_path) - 1)   
        if len(self.__a_star_path) > 0:
            next_node = self.__a_star_path[self.__a_star_index]
            next_node = Position2D(next_node)

            current_grid_index = self.__mapper.pixel_grid.coordinates_to_grid_index(self.__mapper.robot_position)
            current_node = Position2D(current_grid_index[0], current_grid_index[1])

            if abs(current_node.get_distance_to(next_node)) < 3:
                self.__a_star_index += 1

    def __dither_path(self, path):
        if not len(path):
            return path
        final_path = []
        dither_interval = 2
        for index, value in enumerate(path):
            if index % dither_interval == 0:
                final_path.append(value)
        if tuple(final_path[-1]) != tuple(path[-1]):
            final_path.append(path[-1])

        if len(final_path):
            return final_path
        else:
            return path
    
    def get_next_position(self) -> Position2D:
        self.__a_star_index = min(self.__a_star_index, len(self.__a_star_path) -1)
        if len(self.__smooth_astar_path):
            pos = self.__mapper.pixel_grid.grid_index_to_coordinates(np.array(self.__smooth_astar_path[self.__a_star_index]))
            pos = Position2D(pos[0], pos[1])
            return pos
        
        else:
            return self.__mapper.robot_position
    
    def __is_path_obstructed(self):
        """
        Is current Astar path obstructed?
        """
        array_index_path = []
        for n in self.__a_star_path:
            array_index_path.append(self.__mapper.pixel_grid.grid_index_to_array_index(n))
            
        for position in array_index_path:
            if position[0] >= self.__mapper.pixel_grid.arrays["traversable"].shape[0] or \
               position[1] >=  self.__mapper.pixel_grid.arrays["traversable"].shape[1]:
                continue

            if position[0] < 0 or position[1] < 0:
                continue

            if self.__mapper.pixel_grid.arrays["traversable"][position[0], position[1]]:
                return True
            
        return False
    
    def is_path_finished(self):
        return len(self.__a_star_path) - 1 <= self.__a_star_index
    

    def __get_closest_traversable_array_index(self, array_index):
        if self.__mapper.pixel_grid.arrays["traversable"][array_index[0], array_index[1]]:
            return  self.__closest_free_point_finder.bfs(found_array=self.__mapper.pixel_grid.arrays["traversable"],
                                                         traversable_array=self.__mapper.pixel_grid.arrays["traversable"],
                                                         start_node=array_index)[0]
        else:
            return array_index