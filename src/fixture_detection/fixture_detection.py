from data_structures.vectors import Position2D, Vector2D
from data_structures.angle import Angle
from typing import List
from robot.devices.camera import CameraImage
from fixture_detection.color_filter import ColorFilter

import math

import numpy as np
import cv2 as cv

class FixtureDetector:
    def __init__(self) -> None:
        # Color filtering
        self.colors = ("black", "white", "yellow", "red")
        self.color_filters = {
            "black": ColorFilter(lower_hsv=(0, 0, 0), upper_hsv=(0, 0, 0)),
            "white": ColorFilter(lower_hsv=(0, 0, 207), upper_hsv=(0, 0, 207)),
            "yellow": ColorFilter(lower_hsv=(25, 157, 82), upper_hsv=(30, 255, 255)),
            "red": ColorFilter(lower_hsv=(160, 170, 127), upper_hsv=(170, 255, 255))
        }

    def get_fixture_positions(self, robot_position: Position2D, camera_image: CameraImage, lidar_detections: List[Vector2D]) -> List[Position2D]:
        positions_in_image = self.get_fixture_positions_in_image(camera_image.image)

        fixture_positions = []
        for position in positions_in_image:
            relative_horizontal_angle = Angle(position[1] * (camera_image.data.width / camera_image.data.horizontal_fov.radians))

            fixture_horizontal_angle = (relative_horizontal_angle - camera_image.data.horizontal_fov / 2) + camera_image.data.horizontal_orientation 

            fixture_horizontal_angle.normalize()

            best_lidar_detection = Vector2D()
            best_diff = Angle(math.inf)
            for lidar_detection in lidar_detections:
                diff = abs(lidar_detection.direction - fixture_horizontal_angle)
                if diff < best_diff:
                    best_diff = diff
                    best_lidar_detection = lidar_detection
                
            fixture_positions.append(best_lidar_detection.to_position() + robot_position)

        return fixture_positions
    
    def get_fixture_positions_in_image(self, image: np.ndarray) -> List[Position2D]:
        image_sum = np.zeros(image.shape[:2], dtype=np.bool_)
        for filter in self.color_filters.values():
            image_sum += filter.filter(image) > 0

        image_sum = image_sum.astype(np.uint8) * 255

        cv.imshow("fixtures", image_sum)
        
        contours, _ = cv.findContours(image_sum, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        
        final_victims = []
        for c in contours:
            x, y, w, h = cv.boundingRect(c)
            final_victims.append(Position2D((x + w) / 2, (y + h) / 2))
        
        return final_victims
