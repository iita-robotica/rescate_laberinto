import numpy as np
import cv2 as cv
import copy
from data_structures.vectors import Position2D, Vector2D
from data_structures.angle import Angle, Unit
import math
import skimage
from utilities import StepCounter

class PointGrid:
    def __init__(self, initial_shape, pixel_per_cm, robot_radius_m):
        self.array_shape = np.array(initial_shape, dtype=int)
        self.offsets = self.array_shape // 2

        self.grid_index_max = self.array_shape - self.offsets

        self.grid_index_min = self.offsets * -1
        
        self.to_boolean_threshold = 2
        self.delete_step_counter = StepCounter(2)
        self.delete_threshold = 0

        self.arrays = {
            "detected_points": np.zeros(self.array_shape, np.uint8),
            "occupied": np.zeros(self.array_shape, np.bool_),
            "traversable": np.zeros(self.array_shape, np.bool_),
            "navigation_preference": np.zeros(self.array_shape, np.float32),
            "traversed": np.zeros(self.array_shape, np.bool_),
            "seen_by_camera": np.zeros(self.array_shape, np.bool_),
            "seen_by_lidar": np.zeros(self.array_shape, np.bool_),
            "walls_seen_by_camera": np.zeros(self.array_shape, np.bool_),
            "walls_not_seen_by_camera": np.zeros(self.array_shape, np.bool_),
        }

        self.resolution = pixel_per_cm * 100
        
        self.robot_radius = int(robot_radius_m * self.resolution)
        print("ROBOT RADIUS:", self.robot_radius)
        self.robot_diameter = int(self.robot_radius * 2 + 1)


        self.camera_pov_amplitude = Angle(30, Unit.DEGREES)
        self.camera_pov_lenght = int(0.12 * 2 * self.resolution)
        #self.camera_orientations = (Angle(0, Unit.DEGREES), Angle(330, Unit.DEGREES), Angle(30, Unit.DEGREES))
        self.camera_orientations = (Angle(0, Unit.DEGREES), Angle(270, Unit.DEGREES), Angle(90, Unit.DEGREES))


        self.robot_diameter_template = np.zeros((self.robot_diameter, self.robot_diameter), dtype=np.uint8)
        self.robot_diameter_template = cv.circle(self.robot_diameter_template, (self.robot_radius, self.robot_radius), self.robot_radius, 255, -1)
        self.robot_diameter_template = self.robot_diameter_template.astype(np.bool_)
        
        self.robot_diameter_indexes = self.__get_circle_template_indexes(self.robot_radius)

        self.preference_template = self.__generate_quadratic_circle_gradient(self.robot_radius, self.robot_radius * 2)

    def coordinates_to_grid_index(self, coordinates):
        if isinstance(coordinates, np.ndarray):
            coords = (coordinates * self.resolution).astype(int)

            return np.array([coords[1], coords[0]])

    def grid_index_to_coordinates(self, grid_index):
        if isinstance(grid_index, np.ndarray):
            index = (grid_index.astype(float) / self.resolution)
        
            return np.array([index[1], index[0]])
        
    def clean_up(self):
        if self.delete_step_counter.check():
            self.arrays["detected_points"] = self.arrays["detected_points"] * (self.arrays["detected_points"] > self.delete_threshold)
        self.delete_step_counter.increase()

    def load_point_cloud(self, in_bounds_point_cloud, out_of_bounds_point_cloud, robot_position): 
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
       
        self.clean_up()
        
        occupied_as_int = self.arrays["occupied"].astype(np.uint8)

        self.arrays["traversable"] = cv.filter2D(occupied_as_int, -1, self.robot_diameter_template.astype(np.uint8))
        self.arrays["traversable"] = self.arrays["traversable"].astype(np.bool_)
        self.arrays["navigation_preference"] = cv.filter2D(occupied_as_int, -1, self.preference_template)


    def __load_seen_by_lidar(self, in_bounds_point_cloud, out_of_bounds_point_cloud, robot_position):
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


    def load_robot(self, robot_position, robot_rotation: Angle):
        robot_grid_index = self.coordinates_to_grid_index(robot_position)
        # Load points traversed by robot
        self.__load_traversed_by_robot(robot_grid_index)
        # Load points seen by camera
        self.__load_seen_by_camera(robot_grid_index, robot_rotation)
    
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

    def __has_line_of_sight(self, point1, point2, matrix):
        xx, yy = skimage.draw.line(point1[0], point1[1], point2[0], point2[1])
        for x, y in zip(xx[1:-1], yy[1:-1]):
            if matrix[x, y]:
                return False
        return True


    def array_index_to_grid_index(self, array_index: np.ndarray):
        return array_index[0] - self.offsets[0], array_index[1] - self.offsets[1]
    
    def grid_index_to_array_index(self, grid_index: np.ndarray):
        return  grid_index[0] + self.offsets[0], grid_index[1] + self.offsets[1]

    def expand_grid_to_grid_index(self, grid_index: np.ndarray):
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

        end_angle = orientation + (amplitude / 2)
        end_angle.normalize()
        end_vector = Vector2D(end_angle, lenght * 2)
        end_position = end_vector.to_position()
        end_position += center_position
        end_position = (math.ceil(end_position.x), math.ceil(end_position.y))

        triangle_matrix = cv.fillPoly(np.zeros_like(matrix), 
                                      [np.array([start_position, end_position, center_position.get_np_array()])],
                                      1)
        
        final_matrix = triangle_matrix * circle_matrix

        #cv.imshow("triangle", triangle_matrix * 255)
        #cv.imshow("circle", circle_matrix * 255)
        #cv.imshow("final_matrix", final_matrix * 255)

        return final_matrix
    
    def __get_camera_povs_template_indexes(self,  camera_orientations, robot_index):
        final_template = None
        for orientation in camera_orientations:
            cone_template = self.__get_cone_template(self.camera_pov_lenght, orientation, self.camera_pov_amplitude)
            if final_template is None:
                final_template = cone_template
            else:
                final_template += cone_template
        
        #cv.imshow("final_template", final_template * 100)

        povs_indexes = self._get_indexes_from_template(final_template, (-self.camera_pov_lenght + robot_index[0], -self.camera_pov_lenght + robot_index[1]))

        return povs_indexes

    
    def get_colored_grid(self):
        color_grid = np.zeros((self.array_shape[0], self.array_shape[1], 3), dtype=np.float32)

        color_grid[self.arrays["traversed"]] = (.5, 0., .5)
        color_grid[:, :, 1] = self.arrays["navigation_preference"][:, :] / 100
        color_grid[self.arrays["traversable"]] = (1, 0, 0)
        
        
        color_grid[self.arrays["occupied"]] = (1, 1, 1)
        color_grid[self.arrays["walls_not_seen_by_camera"]] = (0, 0, 1)
        
        return color_grid
    
    def print_grid(self):
        cv.imshow("circle_template", self.preference_template / 4)
        #cv.imshow("test", self.get_colored_grid())