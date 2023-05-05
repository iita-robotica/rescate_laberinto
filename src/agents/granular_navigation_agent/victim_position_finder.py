import numpy as np
import cv2 as cv

from mapping.mapper import Mapper
from data_structures.vectors import Position2D, Vector2D
from data_structures.angle import Angle

import math

class VictimPositionFinder:
    def __init__(self, mapper: Mapper) -> None:
        self.mapper = mapper
        self.fixture_normal_template = self.get_fixture_normal_template(radius=8)

    def update(self):
        debug = self.mapper.pixel_grid.get_colored_grid()
        fixture_indexes = np.nonzero(self.mapper.pixel_grid.arrays["victims"])
        for fixture_x, fixture_y in zip(fixture_indexes[0], fixture_indexes[1]):
            pos1 = Position2D(fixture_x, fixture_y)

            detection_angle = self.mapper.pixel_grid.arrays["victim_angles"][fixture_x, fixture_y]
            detection_angle = Angle(detection_angle)
            detection_angle = Angle(180, unit=Angle.DEGREES) + detection_angle
            detection_angle.normalize()

            fixture_square = self.get_fixture_normal_detection_square(np.array(pos1))
            if fixture_square.shape[0] > 1 and fixture_square.shape[1] > 1:
                cv.imshow("fixture_square", fixture_square.astype(np.float32))


                indices = np.array(np.nonzero(fixture_square))

                max_x = np.max(indices[0])
                min_x = np.min(indices[0])
                max_y = np.max(indices[1])
                min_y = np.min(indices[1])

                diff_x = max_x - min_x
                diff_y = max_y - min_y

                wall_angle = np.arctan(diff_x / diff_y)

                wall_angle = Angle(wall_angle)

                print(wall_angle)

                #wall_angle = wall_angle + Angle(180, Angle.DEGREES) * ((detection_angle // Angle(180, Angle.DEGREES)) - (wall_angle // Angle(180, Angle.DEGREES)))

                wall_angle.normalize()


                absolute_diff = wall_angle.get_absolute_distance_to(detection_angle)

                """
                angle_difference = self.initial_angle - target_angle
                if 180 > angle_difference.degrees > 0 or angle_difference.degrees < -180:
                    return self.Directions.LEFT
                else:
                    return self.Directions.RIGHT
                """

                if absolute_diff > Angle(90, Angle.DEGREES):
                    wall_angle = Angle(180, Angle.DEGREES) + wall_angle
                
                wall_angle.normalize()

                vec = Vector2D(wall_angle, 10)
                pos2 = vec.to_position()
                pos2 = Position2D(pos2.y, pos2.x) + pos1
                pos2 = pos2.astype(int)

                debug = cv.line(debug, (pos1[1], pos1[0]), (pos2[1], pos2[0]), (0, 255, 0), 1)
        
        cv.imshow("victim_normal_debug", debug)
        


    def find_victim_position(self):
        pass

    def get_victim_position(self):
        pass

    def is_there_victims(self):
        pass


    def get_fixture_normal_detection_square(self, victim_array_index: np.ndarray):
        vns_min_x = victim_array_index[0] - (self.fixture_normal_template.shape[0] // 2)
        vns_min_y = victim_array_index[1] - (self.fixture_normal_template.shape[1] // 2)
        vns_max_x = vns_min_x + self.fixture_normal_template.shape[0]
        vns_max_y = vns_min_y + self.fixture_normal_template.shape[1]

        self.mapper.pixel_grid.expand_to_grid_index(np.array((vns_min_x, vns_min_y)))
        self.mapper.pixel_grid.expand_to_grid_index(np.array((vns_max_x, vns_max_y)))
        return self.mapper.pixel_grid.arrays["walls"][vns_min_x:vns_max_x, vns_min_y:vns_max_y]


    def get_fixture_normal_angle(self, victim_array_index: np.ndarray):
        vns_min_x = victim_array_index[0] - (self.fixture_normal_template.shape[0] // 2)
        vns_min_y = victim_array_index[1] - (self.fixture_normal_template.shape[1] // 2)
        vns_max_x = vns_min_x + self.fixture_normal_template.shape[0]
        vns_max_y = vns_min_y + self.fixture_normal_template.shape[1]

        self.mapper.pixel_grid.expand_to_grid_index(np.array((vns_min_x, vns_min_y)))
        self.mapper.pixel_grid.expand_to_grid_index(np.array((vns_max_x, vns_max_y)))

        victim_normal_square = np.zeros_like(self.fixture_normal_template)
        victim_normal_square[self.fixture_normal_template] = self.mapper.pixel_grid.arrays["walls"][vns_min_x:vns_max_x, vns_min_y:vns_max_y][self.fixture_normal_template]

        cv.imshow("normal_sqaure", victim_normal_square.astype(np.uint8) * 255)

        moments = cv.moments(victim_normal_square.astype(np.uint8)) # calculate moments of binary image
        centroid = Position2D(moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]) # calculate x,y coordinate of center
        normal_pos =  Position2D(victim_normal_square.shape) + centroid * -1

        print("normal_pos", centroid)

        center = Position2D(victim_normal_square.shape)
        center = center / 2

        return center.get_angle_to(centroid)

    def get_fixture_normal_template(self, radius):
        diameter = radius * 2 + 1

        template = np.zeros((diameter, diameter), dtype=np.uint8)

        template = cv.circle(template, (radius, radius), radius, 255, -1)

        return template.astype(np.bool_)

