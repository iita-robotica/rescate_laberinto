import numpy as np
import cv2 as cv
from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid

from data_structures.vectors import Position2D

import skimage

class WallMapper:
    def __init__(self, compound_grid: CompoundExpandablePixelGrid, robot_diameter: float) -> None:
        self.grid = compound_grid

        compensation = 0

        self.robot_diameter = int(robot_diameter * self.grid.resolution) + compensation * 2
        self.robot_radius = int(robot_diameter / 2 * self.grid.resolution) + compensation

        self.to_boolean_threshold = 3
        self.delete_threshold = 1
        
        # Circle with the radius of the robot
        self.robot_diameter_template = np.zeros((self.robot_diameter, self.robot_diameter), dtype=np.uint8)
        self.robot_diameter_template = cv.circle(self.robot_diameter_template, (self.robot_radius, self.robot_radius), self.robot_radius, 255, -1)
        self.robot_diameter_template = self.robot_diameter_template.astype(np.bool_)

        # A template to calculated the preference of each pixel for navigation taking into account the distance from the wall
        #self.preference_template = self.__generate_quadratic_circle_gradient(self.robot_radius, self.robot_radius * 2)
        self.preference_template = self.__generate_quadratic_circle_gradient(self.robot_radius, self.robot_radius * 1.7)




    def load_point_cloud(self, in_bounds_point_cloud, out_of_bounds_point_cloud, robot_position):
        """
        Loads into the corresponding arrays what has been seen by the lidar, what the lidar has detected, and
        what walls the lidar has detected but the camera hasn't seen.
        Calculates the travesable areas and the preference of each point for navigation.
        """
        
        
        robot_position_as_array = np.array(robot_position, dtype=float)
        
        self.__reset_seen_by_lidar()

        self.load_in_bounds_point_cloud(in_bounds_point_cloud, robot_position_as_array)
        self.load_out_of_bounds_point_cloud(out_of_bounds_point_cloud, robot_position_as_array)


    def load_in_bounds_point_cloud(self, point_cloud, robot_position):
        for p in point_cloud:
            point = np.array(p, dtype=float) + robot_position

            point_grid_index = self.grid.coordinates_to_grid_index(point)

            self.grid.expand_to_grid_index(point_grid_index)

            robot_array_index = self.grid.coordinates_to_array_index(robot_position)
            point_array_index = self.grid.grid_index_to_array_index(point_grid_index)

            self.occupy_point(point_array_index)

            self.mark_point_as_seen_by_lidar(robot_array_index, point_array_index)
            
        self.filter_out_noise()

        self.generate_navigation_margins()

    def load_out_of_bounds_point_cloud(self, point_cloud, robot_position):
        for p in point_cloud:
            point = np.array(p, dtype=float) + robot_position

            point_grid_index = self.grid.coordinates_to_grid_index(point)
            self.grid.expand_to_grid_index(point_grid_index)

            robot_array_index = self.grid.coordinates_to_array_index(robot_position)
            point_array_index = self.grid.grid_index_to_array_index(point_grid_index)

            self.mark_point_as_seen_by_lidar(robot_array_index, point_array_index)

        self.calculate_seen_walls()

    def calculate_seen_walls(self):
        self.grid.arrays["walls_seen_by_camera"] = self.grid.arrays["seen_by_camera"] * self.grid.arrays["walls"]
        self.grid.arrays["walls_not_seen_by_camera"] =  np.logical_xor(self.grid.arrays["walls"], self.grid.arrays["walls_seen_by_camera"])

    def generate_navigation_margins(self):
        # Areas traversable by the robot
        occupied_as_int = self.grid.arrays["occupied"].astype(np.uint8)
        diameter_template_as_int = self.robot_diameter_template.astype(np.uint8)

        self.grid.arrays["traversable"] = np.zeros_like(self.grid.arrays["traversable"])
        self.grid.arrays["traversable"] = cv.filter2D(occupied_as_int, -1, diameter_template_as_int)
        self.grid.arrays["traversable"] = self.grid.arrays["traversable"].astype(np.bool_)

        # Areas that the robot prefers to navigate through
        self.grid.arrays["navigation_preference"] = cv.filter2D(occupied_as_int, -1, self.preference_template)
        self.grid.arrays["navigation_preference"][self.grid.arrays["swamps"]] = 150

    def filter_out_noise(self):
        """
        Filters out noise from the 'detected_points' array.
        """
        self.grid.arrays["detected_points"] = self.grid.arrays["detected_points"] * (self.grid.arrays["detected_points"] > self.delete_threshold)


    # Initialization methods
    def __generate_quadratic_circle_gradient(self, min_radius, max_radius):
        min_radius = round(min_radius)
        max_radius = round(max_radius)
        template = np.zeros((max_radius * 2 + 1, max_radius * 2 + 1), dtype=np.float32)
        for i in range(max_radius, min_radius, -1):
            template = cv.circle(template, (max_radius, max_radius), i, max_radius ** 2 - i ** 2, -1)
        
        return template * 0.1
    
    def __generate_linear_circle_gradient(self, min_radius, max_radius):
        min_radius = round(min_radius)
        max_radius = round(max_radius)
        template = np.zeros((max_radius * 2 + 1, max_radius * 2 + 1), dtype=np.float32)
        for i in range(max_radius, min_radius, -1):
            print("i:", i)
            template = cv.circle(template, (max_radius, max_radius), i, max_radius - i, -1)
        
        return template * 0.5
    
    def occupy_point(self, point_array_index):        
        if not self.grid.arrays["walls"][point_array_index[0], point_array_index[1]]:
            self.grid.arrays["detected_points"][point_array_index[0], point_array_index[1]] += 1
            
            if self.grid.arrays["detected_points"][point_array_index[0], point_array_index[1]] > self.to_boolean_threshold:
                if not self.grid.arrays["traversed"][point_array_index[0], point_array_index[1]]:
                    self.grid.arrays["walls"][point_array_index[0], point_array_index[1]] = True
                    #self.grid.arrays["occupied"][point_array_index[0], point_array_index[1]] = True
                    

    def mark_point_as_seen_by_lidar(self, robot_array_index, point_array_index):
        self.grid.arrays["seen_by_lidar"] = self.__draw_bool_line(self.grid.arrays["seen_by_lidar"], robot_array_index, point_array_index)
    
    def __draw_bool_line(self, array, point1, point2):
        indexes = skimage.draw.line(point1[0], point1[1], point2[0], point2[1])
    
        array[indexes[0][:-2], indexes[1][:-2]] = True
        return array
        #array = cv.line(array.astype(np.uint8), (point1[1], point1[0]), (point2[1], point2[0]), 255, thickness=1, lineType=cv.LINE_8)
        #return array.astype(np.bool_)
    
    def __reset_seen_by_lidar(self):
        self.grid.arrays["seen_by_lidar"] = np.zeros_like(self.grid.arrays["seen_by_lidar"])

