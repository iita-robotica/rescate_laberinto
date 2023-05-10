import math

import numpy as np
import cv2 as cv

from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid


class FixtureMapper:
    def __init__(self, pixel_grid: CompoundExpandablePixelGrid, tile_size: float) -> None:
        self.tile_size = tile_size
        self.grid = pixel_grid

        template_radious = int(0.034 * self.grid.resolution)
        template_diameter = math.ceil(template_radious * 2 + 1)

        # Circle with the radius of the fixture detection zone
        self.fixture_detection_zone_template = np.zeros((template_diameter, template_diameter), dtype=np.float32)
        #self.fixture_detection_zone_template = cv.circle(self.fixture_detection_zone_template, (template_radious, template_radious), template_radious, -10, -1)
        #self.fixture_detection_zone_template = cv.circle(self.fixture_detection_zone_template, (template_radious, template_radious), template_radious, 1, 1)
        self.fixture_detection_zone_template = cv.circle(self.fixture_detection_zone_template, (template_radious, template_radious), template_radious, 1, -1)
        
    
    def generate_detection_zone(self):
        occupied_as_float = self.grid.arrays["occupied"].astype(np.float32)

        self.grid.arrays["fixture_detection_zone"] = cv.filter2D(occupied_as_float, -1, self.fixture_detection_zone_template)
        self.grid.arrays["fixture_detection_zone"] = self.grid.arrays["fixture_detection_zone"] > 0.999

        cv.imshow("detection_template", self.fixture_detection_zone_template)
