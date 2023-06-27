from data_structures.vectors import Position2D, Vector2D
from data_structures.angle import Angle
from typing import List
from robot.devices.camera import CameraImage
from fixture_detection.color_filter import ColorFilter
import skimage

import copy

import math

import numpy as np
import cv2 as cv

from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid
from flags import SHOW_FIXTURE_DEBUG

class FixtureDetector:
    def __init__(self, pixel_grid: CompoundExpandablePixelGrid) -> None:
        self.pixel_grid = pixel_grid

        # Color filtering
        self.colors = ("black", "white", "yellow", "red")
        self.color_filters = {
            "black": ColorFilter(lower_hsv=(0, 0, 0), upper_hsv=(0, 0, 9)),
            "white": ColorFilter(lower_hsv=(0, 0, 193), upper_hsv=(255, 110, 208)),
            "yellow": ColorFilter(lower_hsv=(25, 170, 82), upper_hsv=(30, 255, 255)),
            "red": ColorFilter(lower_hsv=(134, 91, 185), upper_hsv=(175, 255, 204))           
        }

        self.wall_color_filter = ColorFilter((90, 44,  0), (95, 213, 158))

        self.max_detection_distance = 0.12 * 5

    def get_wall_mask(self, image: np.ndarray):
        margin = 1
        raw_wall = self.wall_color_filter.filter(image)

        wall = np.ones(shape=(raw_wall.shape[0], raw_wall.shape[1] + margin * 2), dtype=np.uint8) * 255

        wall[:, margin: -margin] = raw_wall
        
        #cv.imshow("pre_wall", wall)

        conts, _ = cv.findContours(wall, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        debug = np.copy(image)

        filled_wall = np.zeros_like(wall, dtype=np.bool_)

        for c in conts:
            this_cont = np.zeros_like(wall, dtype=np.uint8)
            cv.fillPoly(this_cont, [c,], 255)
            filled_wall += this_cont > 0

        filled_wall = filled_wall[:, margin:-margin]

        return filled_wall

    def get_fixture_positions_and_angles(self, robot_position: Position2D, camera_image: CameraImage) -> list:
        positions_in_image = self.get_fixture_positions_in_image(np.flip(camera_image.image, axis=1))

        #debug = self.pixel_grid.get_colored_grid()

        fixture_positions = []
        fixture_angles = []
        for position in positions_in_image:
            relative_horizontal_angle = Angle(position[1] * (camera_image.data.horizontal_fov.radians / camera_image.data.width))

            fixture_horizontal_angle = (relative_horizontal_angle - camera_image.data.horizontal_fov / 2) + camera_image.data.horizontal_orientation 

            fixture_horizontal_angle.normalize()

            camera_vector = Vector2D(camera_image.data.horizontal_orientation, camera_image.data.distance_from_center)
            camera_pos = camera_vector.to_position()
            camera_pos += robot_position

            detection_vector = Vector2D(fixture_horizontal_angle, self.max_detection_distance)
            detection_pos = detection_vector.to_position()

            detection_pos += camera_pos

            camera_array_index = self.pixel_grid.coordinates_to_array_index(camera_pos)
            detection_array_index = self.pixel_grid.coordinates_to_array_index(detection_pos)

            line_xx, line_yy = skimage.draw.line(camera_array_index[0], camera_array_index[1], detection_array_index[0], detection_array_index[1])

            index = 0
            for x, y in zip(line_xx, line_yy):
                if x >= 0 and y >= 0 and x < self.pixel_grid.array_shape[0] and y < self.pixel_grid.array_shape[1]:
                    #debug[x, y] = (0, 255, 0)
                    back_index = index - 2
                    back_index = max(back_index, 0)
                    if self.pixel_grid.arrays["walls"][x, y]:
                        x1 = line_xx[back_index]
                        y1 = line_yy[back_index]
                        fixture_positions.append(self.pixel_grid.array_index_to_coordinates(np.array([x1, y1])))
                        fixture_angles.append(copy.deepcopy(fixture_horizontal_angle))
                        break
                index += 1

        #cv.imshow("fixture_detection_debug", debug)

        return fixture_positions, fixture_angles
    
    def get_fixture_positions_in_image(self, image: np.ndarray) -> List[Position2D]:
        image_sum = np.zeros(image.shape[:2], dtype=np.bool_)
        for filter in self.color_filters.values():
            image_sum += filter.filter(image) > 0

        image_sum = image_sum.astype(np.uint8) * 255

        wall_mask = self.get_wall_mask(image)

        image_sum *= wall_mask

        if SHOW_FIXTURE_DEBUG:
            cv.imshow("fixtures", image_sum)
        
        contours, _ = cv.findContours(image_sum, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        
        
        final_victims = []
        for c in contours:
            x, y, w, h = cv.boundingRect(c)
            final_victims.append(Position2D((x + x + w) / 2, (y + y + h) / 2))

        if SHOW_FIXTURE_DEBUG:
            debug = copy.deepcopy(image)
            for f in final_victims:
                debug = cv.circle(debug, np.array(f, dtype=int), 3, (255, 0, 0), -1)
            
            cv.imshow("victim_pos_debug", debug)

        return final_victims
    
    def map_fixtures(self, camera_images, robot_position):
        for i in camera_images:
            positions, angles = self.get_fixture_positions_and_angles(robot_position, i)
            for pos, angle in zip(positions, angles):
                index = self.pixel_grid.coordinates_to_array_index(pos)
                self.pixel_grid.arrays["victims"][index[0], index[1]] = True
                self.pixel_grid.arrays["victim_angles"][index[0], index[1]] = angle.radians

    def mark_reported_fixture(self, robot_position, fixture_position):
        fixture_array_index = self.pixel_grid.coordinates_to_array_index(fixture_position)
        rr, cc = skimage.draw.disk(fixture_array_index, 4)
        self.pixel_grid.arrays["fixture_detection"][rr, cc] = True

