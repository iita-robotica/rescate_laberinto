import math

import numpy as np
import cv2 as cv

from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid


class FixtureMapper:
    def __init__(self, pixel_grid: CompoundExpandablePixelGrid, tile_size: float) -> None:
        self.tile_size = tile_size
        self.grid = pixel_grid

        

        template_radious = int(0.05 * self.grid.resolution)
        template_diameter = math.ceil(template_radious * 2 + 1)

        # Empty circle with the radius of the fixture detection zone
        self.fixture_distance_margin_template = np.zeros((template_diameter, template_diameter), dtype=np.int8)
        self.fixture_distance_margin_template = cv.circle(self.fixture_distance_margin_template, (template_radious, template_radious), template_radious, -50, -1)
        self.fixture_distance_margin_template = cv.circle(self.fixture_distance_margin_template, (template_radious, template_radious), template_radious, 1, 1)
        
        self.detected_from_radius = round(0.02 * self.grid.resolution)
    
    def generate_detection_zone(self):
        occupied_as_int = self.grid.arrays["walls"].astype(np.int8)

        #self.grid.arrays["fixture_detection_zone"] = cv.filter2D(occupied_as_int, -1, self.fixture_detection_zone_template)> 0

        self.grid.arrays["fixture_distance_margin"] = cv.filter2D(occupied_as_int, -1, self.fixture_distance_margin_template) > 0
        
        #cv.imshow("detection_template", self.fixture_detection_zone_template)

    def clean_up_fixtures(self):
        self.grid.arrays["victims"][self.grid.arrays["occupied"]] = False

    def map_detected_fixture(self, robot_position):
        robot_array_index = self.grid.coordinates_to_array_index(robot_position)
        template = self.__get_circle_template_indexes(self.detected_from_radius, robot_array_index)
        self.grid.arrays["robot_detected_fixture_from"][template[0], template[1]] = True

    def __get_circle_template_indexes(self, radius, offsets=(0, 0)):
        diameter = int(radius * 2 + 1)

        diameter_template = np.zeros((diameter, diameter), dtype=np.uint8)
        diameter_template = cv.circle(diameter_template, (radius, radius), radius, 255, -1)
        diameter_template = diameter_template.astype(np.bool_)

        return self.__get_indexes_from_template(diameter_template, (-radius + offsets[0], -radius + offsets[1]))

    def __get_indexes_from_template(self, template: np.ndarray, offsets=(0, 0)):
        indexes = []
        indexes = template.nonzero()
        indexes = np.array(indexes)
        offsets = np.array(offsets)
        indexes[0] += offsets[0]
        indexes[1] += offsets[1]
        return indexes
