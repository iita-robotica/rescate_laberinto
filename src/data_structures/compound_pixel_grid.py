import numpy as np
import cv2 as cv
import copy
from data_structures.vectors import Position2D, Vector2D
from data_structures.angle import Angle, Unit
import math
from flow_control.step_counter import StepCounter

class Grid:
    def __init__(self, initial_shape, pixel_per_m, robot_radius_m):
        self.array_shape = np.array(initial_shape, dtype=int)
        self.offsets = self.array_shape // 2

        self.grid_index_max = self.array_shape - self.offsets # Maximum grid index
        self.grid_index_min = self.offsets * -1 # Minimum grid index
        
        self.to_boolean_threshold = 2 # How many points need to be detected by the lidar to qualify as a sure point
        self.delete_step_counter = StepCounter(2) # Every how many steps should the lidar detections be filtered
        self.delete_threshold = 1 # Points detected less times than this number will be deleted in __cleanup()

        self.arrays = {
            "detected_points": np.zeros(self.array_shape, np.uint8), # Number of points detected in position
            "occupied": np.zeros(self.array_shape, np.bool_), # Confermed occupied point
            "traversable": np.zeros(self.array_shape, np.bool_), # Is or not traversable by the robot, assuming that the robot center is there. True means not traversable.
            "navigation_preference": np.zeros(self.array_shape, np.float32), # The preference for navigation for each pixel. More means less preferred to navigate through.
            "traversed": np.zeros(self.array_shape, np.bool_), # Robot has already gone through there
            "seen_by_camera": np.zeros(self.array_shape, np.bool_), # Has been seen by any of the cameras
            "seen_by_lidar": np.zeros(self.array_shape, np.bool_), # Has been seen by the lidar (Though not necessarily detected as occupied)
            "walls_seen_by_camera": np.zeros(self.array_shape, np.bool_),
            "walls_not_seen_by_camera": np.zeros(self.array_shape, np.bool_),
            "discovered": np.zeros(self.array_shape, np.bool_),
            "floor_color": np.zeros(self.array_shape, np.uint8)
        }

        self.resolution = pixel_per_m # resolution of the grid with regards to the coordinate system of the gps / the world
        
        self.robot_radius = int(robot_radius_m * self.resolution)
        print("ROBOT RADIUS:", self.robot_radius)
        self.robot_diameter = int(self.robot_radius * 2 + 1)

        self.camera_pov_amplitude = Angle(30, Unit.DEGREES) # Horizontal amplitued of the fostrum of each camera
        self.camera_pov_lenght = int(0.12 * 2 * self.resolution) # Range of each camera
        self.camera_orientations = (Angle(0, Unit.DEGREES), Angle(270, Unit.DEGREES), Angle(90, Unit.DEGREES)) # Orientation of the cameras
        
        self.discovery_pov_amplitude =  Angle(170, Unit.DEGREES)
        self.discovery_pov_lenght = self.camera_pov_lenght
        self.discovery_pov_orientation = Angle(0, Unit.DEGREES)

        # Circle with the radius of the robot
        self.robot_diameter_template = np.zeros((self.robot_diameter, self.robot_diameter), dtype=np.uint8)
        self.robot_diameter_template = cv.circle(self.robot_diameter_template, (self.robot_radius, self.robot_radius), self.robot_radius, 255, -1)
        self.robot_diameter_template = self.robot_diameter_template.astype(np.bool_)
        
        # True indexes inside the circle
        self.robot_diameter_indexes = self.__get_circle_template_indexes(self.robot_radius)

        # A template to calculated the preference of each pixel for navigation taking into account the distance from the wall
        self.preference_template = self.__generate_quadratic_circle_gradient(self.robot_radius, self.robot_radius * 2)

    def load_point_cloud(self, in_bounds_point_cloud, out_of_bounds_point_cloud, robot_position):
        """
        Loads into the corresponding arrays what has been seen by the lidar, what the lidar has detected, and
        what walls the lidar has detected but the camera hasn't seen.
        Calculates the travesable areas and the preference of each point for navigation.
        """
        # Seen by lidar
        self.__load_seen_by_lidar(in_bounds_point_cloud, out_of_bounds_point_cloud, robot_position)

        # Occupied points
        for p in in_bounds_point_cloud:
            p1 = np.array(p, dtype=float)
            p1 += robot_position
            p1 = self.coordinates_to_grid_index(p1)

            self.expand_grid_to_grid_index(p1)
            position = self.grid_index_to_array_index(p1)
            
            if not self.arrays["occupied"][position[0], position[1]]:
                if self.arrays["detected_points"][position[0], position[1]] < self.to_boolean_threshold:
                    self.arrays["detected_points"][position[0], position[1]] += 1
                elif self.arrays["detected_points"][position[0], position[1]] >= self.to_boolean_threshold:
                    if not self.arrays["traversed"][position[0], position[1]]:
                        self.arrays["occupied"][position[0], position[1]] = True

        # Filters out noise
        self.__clean_up()
        
        occupied_as_int = self.arrays["occupied"].astype(np.uint8)

        # Areas traversable by the robot
        self.arrays["traversable"] = cv.filter2D(occupied_as_int, -1, self.robot_diameter_template.astype(np.uint8))
        self.arrays["traversable"] = self.arrays["traversable"].astype(np.bool_)

        # Areas that the robot prefers to navigate through
        self.arrays["navigation_preference"] = cv.filter2D(occupied_as_int, -1, self.preference_template)

    def __load_seen_by_lidar(self, in_bounds_point_cloud, out_of_bounds_point_cloud, robot_position):
        """
        Loads into the corresponding arrays what has been seen by the lidar and
        what walls the lidar has detected but the camera hasn't seen.
        """

        robot_position = robot_position.astype(float)
        self.arrays["seen_by_lidar"] = np.zeros_like(self.arrays["seen_by_lidar"])
        for p in in_bounds_point_cloud + out_of_bounds_point_cloud:
            p1 = np.array(p, dtype=float)
            p1 += robot_position
            p1 = self.coordinates_to_grid_index(p1)
            self.expand_grid_to_grid_index(p1)
            position = self.grid_index_to_array_index(p1)
            robot_grid_index = self.coordinates_to_grid_index(robot_position)
            robot_array_index = self.grid_index_to_array_index(robot_grid_index)

            self.arrays["seen_by_lidar"] = cv.line(self.arrays["seen_by_lidar"].astype(np.uint8), (position[1], position[0]), (robot_array_index[1], robot_array_index[0]), 255, thickness=1, lineType=cv.LINE_8)
            self.arrays["seen_by_lidar"] = self.arrays["seen_by_lidar"].astype(np.bool_)
        
        self.arrays["walls_seen_by_camera"] = self.arrays["seen_by_camera"] * self.arrays["occupied"]
        self.arrays["walls_not_seen_by_camera"] =  np.logical_xor(self.arrays["occupied"], self.arrays["walls_seen_by_camera"])

    def __clean_up(self):
        """
        Filters out noise from the 'detected_points' array.
        """
        self.arrays["detected_points"] = self.arrays["detected_points"] * (self.arrays["detected_points"] > self.delete_threshold)

    def load_robot(self, robot_position, robot_rotation: Angle):
        """
        Loads into the corresponding arrays where has the robot gone trough and what the cameras have seen.
        """
        robot_grid_index = self.coordinates_to_grid_index(robot_position)
        # Load points traversed by robot
        self.__load_traversed_by_robot(robot_grid_index)
        # Load points seen by camera
        self.__load_seen_by_camera(robot_grid_index, robot_rotation)

        self.__load_discovered(robot_grid_index, robot_rotation)
    
    def __load_traversed_by_robot(self, robot_grid_index):
        circle = self.robot_diameter_indexes + np.array(robot_grid_index)
        for item in circle:
            self.expand_grid_to_grid_index(item)
            array_index = self.grid_index_to_array_index(item)
            self.arrays["traversed"][array_index[0], array_index[1]] = True

    def __load_seen_by_camera(self, robot_grid_index, robot_rotation: Angle):
        global_camera_orientations = []

        for o in self.camera_orientations:
            o1 = o + robot_rotation
            o1.normalize()
            global_camera_orientations.append(o1)

        camera_povs = self.__get_camera_povs_template_indexes(global_camera_orientations, robot_grid_index)
        for item in camera_povs:
            self.expand_grid_to_grid_index(item)

            array_index = self.grid_index_to_array_index(item)
            robot_array_index = self.grid_index_to_array_index(robot_grid_index)
            if self.arrays["seen_by_lidar"][array_index[0], array_index[1]]:
                self.arrays["seen_by_camera"][array_index[0], array_index[1]] = True

    def __load_discovered(self, robot_grid_index, robot_rotation: Angle):
        global_discovered_orientation = self.discovery_pov_orientation + robot_rotation
        global_discovered_orientation.normalize()
        
        discovered_template = self.__get_cone_template(self.discovery_pov_lenght, 
                                                       global_discovered_orientation, 
                                                       self.discovery_pov_amplitude)
        
        disc_povs = self._get_indexes_from_template(discovered_template, robot_grid_index - np.array((self.discovery_pov_lenght, self.discovery_pov_lenght)))

        for item in disc_povs:
            self.expand_grid_to_grid_index(item)
            array_index = self.grid_index_to_array_index(item)

            if self.arrays["seen_by_lidar"][array_index[0], array_index[1]]:
                self.arrays["discovered"][array_index[0], array_index[1]] = True
    
    # Index conversion
    def coordinates_to_grid_index(self, coordinates) -> np.ndarray:
        coords = np.array(coordinates)
        coords = (coords * self.resolution).astype(int)
        return np.array([coords[1], coords[0]])

    def grid_index_to_coordinates(self, grid_index) -> np.ndarray:
        index = np.array(grid_index)
        index = (index.astype(float) / self.resolution)
    
        return np.array([index[1], index[0]])

    def array_index_to_grid_index(self, array_index) -> np.ndarray:
        return np.array(array_index) - self.offsets
    
    def grid_index_to_array_index(self, grid_index) -> np.ndarray:
        return np.array(grid_index) + self.offsets
    
    def array_index_to_coordinates(self, array_index) -> np.ndarray:
        grid_index = self.array_index_to_grid_index(array_index)
        return self.grid_index_to_coordinates(grid_index)
    
    def coordinates_to_array_index(self, coordinates) -> np.ndarray:
        grid_index = self.coordinates_to_grid_index(coordinates)
        return self.grid_index_to_array_index(grid_index)

    # Grid expansion
    def expand_grid_to_grid_index(self, grid_index: np.ndarray):
        """
        Expands all arrays to the specified index. 
        Note that all array_idexes should be recalculated after this operation.
        """

        array_index = self.grid_index_to_array_index(grid_index)
        if array_index[0] + 1 > self.array_shape[0]:
            self.add_end_row(array_index[0] - self.array_shape[0] + 1)

        if array_index[1] + 1 > self.array_shape[1]:
            self.add_end_column(array_index[1] - self.array_shape[1] + 1)

        if array_index[0] < 0:
            self.add_begining_row(array_index[0] * -1)
        if array_index[1] < 0:
            self.add_begining_column(array_index[1] * -1)
    
    def add_end_row(self, size):
        self.array_shape = np.array([self.array_shape[0] + size, self.array_shape[1]])
        
        for key in self.arrays:
            self.arrays[key] = self.__add_end_row_to_grid(self.arrays[key], size)
        
    def add_begining_row(self, size):
        self.offsets[0] += size
        self.array_shape = np.array([self.array_shape[0] + size, self.array_shape[1]])

        for key in self.arrays:
            self.arrays[key] = self.__add_begining_row_to_grid(self.arrays[key], size)

    def add_end_column(self, size):
        self.array_shape = np.array([self.array_shape[0], self.array_shape[1] + size])

        for key in self.arrays:
            self.arrays[key] = self.__add_end_column_to_grid(self.arrays[key], size)

    def add_begining_column(self, size):
        self.offsets[1] += size
        self.array_shape = np.array([self.array_shape[0], self.array_shape[1] + size])

        for key in self.arrays:
            self.arrays[key] = self.__add_begining_column_to_grid(self.arrays[key], size)

    def __add_end_row_to_grid(self, grid, size):
        grid = np.vstack((grid, np.zeros((size, self.array_shape[1]), dtype=grid.dtype)))
        return grid
    
    def __add_begining_row_to_grid(self, grid, size):
        grid = np.vstack((np.zeros((size, self.array_shape[1]), dtype=grid.dtype), grid))
        return grid
    
    def __add_end_column_to_grid(self, grid, size):
        grid = np.hstack((grid, np.zeros((self.array_shape[0], size), dtype=grid.dtype)))
        return grid

    def __add_begining_column_to_grid(self, grid, size):
        grid = np.hstack((np.zeros((self.array_shape[0], size), dtype=grid.dtype), grid))
        return grid
    
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
    
    # Camera fostrum template generation
    def __get_circle_template_indexes(self, radius):
        diameter = int(radius * 2 + 1)

        diameter_template = np.zeros((diameter, diameter), dtype=np.uint8)
        diameter_template = cv.circle(diameter_template, (radius, radius), radius, 255, -1)
        diameter_template = self.robot_diameter_template.astype(np.bool_)

        return self._get_indexes_from_template(diameter_template, (-radius, -radius))
    
    def _get_indexes_from_template(self, template, offsets=(0, 0)):
        indexes = []
        for x, row in enumerate(template):
            for y, val in enumerate(row):
                if val:
                    indexes.append((x + offsets[0], y + offsets[1]))
        
        return np.array(indexes)

    
    def __get_cone_template(self, lenght, orientation: Angle, amplitude: Angle):
        matrix_size = math.ceil(lenght) * 2
        int_lenght = math.ceil(lenght)

        matrix = np.zeros((matrix_size + 1, matrix_size + 1), np.uint8)

        circle_matrix = cv.circle(np.zeros_like(matrix), (int_lenght,  int_lenght), int_lenght, 1, -1)
        
        center_position = Position2D(int_lenght, int_lenght)
        
        start_angle = orientation - (amplitude / 2)
        start_angle.normalize()
        start_vector = Vector2D(start_angle, lenght * 2)
        start_position = start_vector.to_position()
        start_position += center_position
        start_position = (math.ceil(start_position.x), math.ceil(start_position.y))

        center_angle = orientation
        center_angle.normalize()
        center_vector = Vector2D(center_angle, lenght * 2)
        center_up_position = center_vector.to_position()
        center_up_position += center_position
        center_up_position = center_up_position.astype(int)

        end_angle = orientation + (amplitude / 2)
        end_angle.normalize()
        end_vector = Vector2D(end_angle, lenght * 2)
        end_position = end_vector.to_position()
        end_position += center_position
        end_position = (math.ceil(end_position.x), math.ceil(end_position.y))

        triangle_matrix = cv.fillPoly(np.zeros_like(matrix), 
                                      [np.array([start_position, center_up_position, end_position, np.array(center_position)])],
                                      1)
        
        final_matrix = triangle_matrix * circle_matrix

        cv.imshow("cone template", final_matrix * 100)

        return final_matrix
    
    def __get_camera_povs_template_indexes(self,  camera_orientations, robot_index):
        final_template = None
        for orientation in camera_orientations:
            cone_template = self.__get_cone_template(self.camera_pov_lenght, orientation, self.camera_pov_amplitude)
            if final_template is None:
                final_template = cone_template
            else:
                final_template += cone_template

        povs_indexes = self._get_indexes_from_template(final_template, (-self.camera_pov_lenght + robot_index[0], -self.camera_pov_lenght + robot_index[1]))

        return povs_indexes

    # Debug
    def get_colored_grid(self):
        """
        Get graphical representation of the grid for debug.
        """
        color_grid = np.zeros((self.array_shape[0], self.array_shape[1], 3), dtype=np.float32)

        color_grid[self.arrays["traversed"]] = (.5, 0., .5)
        color_grid[:, :, 1] = self.arrays["navigation_preference"][:, :] / 100
        color_grid[self.arrays["traversable"]] = (1, 0, 0)
        
        color_grid[self.arrays["discovered"]] = (0, 0, 1)
        color_grid[self.arrays["seen_by_camera"]] = (1, 0, 0)
        color_grid[self.arrays["occupied"]] = (1, 1, 1)

        return color_grid
    
    def print_grid(self):
        cv.imshow("circle_template", self.preference_template / 4)
        #cv.imshow("test", self.get_colored_grid())