import numpy as np
import cv2 as cv
import copy

class PointGrid:
    def __init__(self, initial_shape, pixel_per_cm, robot_radius_m):
        self.shape = np.array(initial_shape, dtype=int)
        self.offsets = self.shape // 2
        
        self.dtype = np.dtype([('detected_points', np.int8, 1),
                               
                               ('seen_by_lidar', np.bool_, 1),

                               ('seen_by_camera', np.bool_, 1),

                               ('has_fixture', np.bool_, 1),
                               ('fixture', np.unicode_, 1),
                               ])
        
        self.to_boolean_threshold = 25
        self.delete_threshold = 1

        self.data_grid = np.zeros(self.shape, self.dtype)
        self.detected_points_grid = np.zeros(self.shape, np.uint8)
        self.occupied_grid = np.zeros(self.shape, np.bool_)
        self.traversable_grid = np.zeros(self.shape, np.bool_)

        self.resolution = pixel_per_cm
        self.robot_radius = int(robot_radius_m * 1000 // pixel_per_cm)
        self.robot_diameter = int(self.robot_radius * 2 + 1)
        self.robot_circle_template = np.zeros((self.robot_diameter, self.robot_diameter), dtype=np.uint8)
        self.robot_circle_template = cv.circle(self.robot_circle_template, (self.robot_radius, self.robot_radius), self.robot_radius, 255, -1)

        self.robot_circle_template = self.robot_circle_template.astype(np.bool_)


    def load_point_cloud(self, point_cloud, robot_position):
        for p in point_cloud:
            p1 = np.array(p)
            p1 += robot_position
            p1 = (p1 * 1000 // self.resolution).astype(int)
            
            position = self.get_point(copy.deepcopy(p1))
            data_point = self.data_grid[position[0], position[1]]

            data_point['seen_by_lidar'] = True
            if not self.occupied_grid[position[0], position[1]]:
                if self.detected_points_grid[position[0], position[1]] < self.to_boolean_threshold:
                    self.detected_points_grid[position[0], position[1]] += 1
                elif self.detected_points_grid[position[0], position[1]] >= self.to_boolean_threshold:
                    self.occupied_grid[position[0], position[1]] = True
        
        self.detected_points_grid = self.detected_points_grid * (self.detected_points_grid > self.delete_threshold)
        
        self.traversable_grid = cv.filter2D(self.occupied_grid.astype(np.uint8), -1, self.robot_circle_template.astype(np.uint8))
        self.traversable_grid = self.traversable_grid.astype(np.bool_)

    def get_point(self, point: np.ndarray):
        point += self.offsets
        self.expand_grid_to_point(point)
        return point[1], point[0]
    
    def expand_grid_to_point(self, point: np.ndarray):
        if point[1] + 1 > self.shape[0]:
            self.add_end_row(point[1] - self.shape[0] + 1)

        if point[0] + 1 > self.shape[1]:
            self.add_end_column(point[0] - self.shape[1] + 1)
        if point[1] < 0:
            self.add_begining_row(point[1] * -1)
        if point[0] < 0:
            self.add_begining_column(point[0] * -1)
    
    def add_end_row(self, size):
        self.shape = np.array([self.shape[0] + size, self.shape[1]])

        
        self.data_grid =        self.__add_end_row_to_grid(self.data_grid, size)
        self.occupied_grid =    self.__add_end_row_to_grid(self.occupied_grid, size)
        self.traversable_grid = self.__add_end_row_to_grid(self.traversable_grid, size)
        self.detected_points_grid = self.__add_end_row_to_grid(self.detected_points_grid, size)
        
    
    def add_begining_row(self, size):
        self.offsets[1] += size
        self.shape = np.array([self.shape[0] + size, self.shape[1]])
        
        self.data_grid =        self.__add_begining_row_to_grid(self.data_grid, size)
        self.occupied_grid =    self.__add_begining_row_to_grid(self.occupied_grid, size)
        self.traversable_grid = self.__add_begining_row_to_grid(self.traversable_grid, size)
        self.detected_points_grid = self.__add_begining_row_to_grid(self.detected_points_grid, size)

    def add_end_column(self, size):
        self.shape = np.array([self.shape[0], self.shape[1] + size])

        self.data_grid =        self.__add_end_column_to_grid(self.data_grid, size)
        self.occupied_grid =    self.__add_end_column_to_grid(self.occupied_grid, size)
        self.traversable_grid = self.__add_end_column_to_grid(self.traversable_grid, size)
        self.detected_points_grid = self.__add_end_column_to_grid(self.detected_points_grid, size)

    def add_begining_column(self, size):
        self.offsets[0] += size
        self.shape = np.array([self.shape[0], self.shape[1] + size])

        self.data_grid =        self.__add_begining_column_to_grid(self.data_grid, size)
        self.occupied_grid =    self.__add_begining_column_to_grid(self.occupied_grid, size)
        self.traversable_grid = self.__add_begining_column_to_grid(self.traversable_grid, size)
        self.detected_points_grid = self.__add_begining_column_to_grid(self.detected_points_grid, size)

    def __add_end_row_to_grid(self, grid, size):
        grid = np.vstack((grid, np.zeros((size, self.shape[1]), dtype=grid.dtype)))
        return grid
    
    def __add_begining_row_to_grid(self, grid, size):
        grid = np.vstack((np.zeros((size, self.shape[1]), dtype=grid.dtype), grid))
        return grid
    
    def __add_end_column_to_grid(self, grid, size):
        grid = np.hstack((grid, np.zeros((self.shape[0], size), dtype=grid.dtype)))
        return grid

    def __add_begining_column_to_grid(self, grid, size):
        grid = np.hstack((np.zeros((self.shape[0], size), dtype=grid.dtype), grid))
        return grid

    def get_node_color_representation(self, position):
        color = [0, 0, 0]
        data_node = self.data_grid[position[0], position[1]]
        occupied = self.occupied_grid[position[0], position[1]]
        not_traversable = self.traversable_grid[position[0], position[1]]

        if data_node["seen_by_lidar"] and occupied:
            if data_node["seen_by_camera"]:
                color = [255, 0, 0]
            else:
                color = [255, 255, 255]
        
        elif not_traversable:
            color = [0, 0, 255]
        
        color.reverse()
        return np.array(color)
    
    def get_colored_grid(self):
        color_grid = np.zeros((self.shape[0], self.shape[1], 3), dtype=np.uint8)

        color_grid[self.traversable_grid] = (255, 0, 0)
        color_grid[self.occupied_grid] = (255, 255, 255)

        """          
        for x, row in enumerate(self.data_grid):
            for y, node in enumerate(row):
                color_grid[x, y] = self.get_node_color_representation([x, y])
        """
        
        return color_grid
    
    def print_grid(self):
        #cv.imshow("circle_template", self.robot_circle_template.astype(np.uint8) * 255)
        cv.imshow("test", self.get_colored_grid())