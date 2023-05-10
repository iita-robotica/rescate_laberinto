import numpy as np
from robot.devices.sensor import TimedSensor
import cv2 as cv

from flow_control.step_counter import StepCounter

from data_structures.angle import Angle

from dataclasses import dataclass

import math

@dataclass
class CameraData:
    height: int
    width: int
    vertical_fov: Angle
    horizontal_fov: Angle
    relative_vertical_orientation: Angle
    relative_horizontal_orientation: Angle
    vertical_orientation: Angle
    horizontal_orientation: Angle
    distance_from_center: float

class CameraImage:
    def __init__(self) -> None:
        self.image: np.ndarray = None
        self.data: CameraData = None

# Captures images and processes them
class Camera(TimedSensor):
    def __init__(self, webots_device, time_step, step_counter: StepCounter, orientation: Angle, distance_from_center: float, rotate180=False):
        super().__init__(webots_device, time_step, step_counter)
        self.rotate180 = rotate180
        self.height = self.device.getHeight()
        self.width = self.device.getWidth()
        self.horizontal_fov = Angle(self.device.getFov())
        self.vertical_fov = Angle(2 * math.atan(math.tan(self.horizontal_fov * 0.5) * (self.height / self.width)))
        self.image = CameraImage()
        
        self.horizontal_orientation_in_robot = orientation
        self.vertical_orientation_in_robot = Angle(0)

        self.horizontal_orientation = orientation
        self.vertical_orientation = Angle(0)
        self.distance_from_center = distance_from_center

    # Returns the camera image
    def get_image(self):
        if self.step_counter.check():
            return self.image
    
    def get_last_image(self):
        return self.image

        
    def get_data(self):
        data = CameraData(self.height,
                          self.width,
                          self.vertical_fov,
                          self.horizontal_fov,
                          self.vertical_orientation_in_robot,
                          self.horizontal_orientation_in_robot,
                          self.vertical_orientation,
                          self.horizontal_orientation,
                          self.distance_from_center)
        return data
    
    def update(self, robot_orientation: Angle):
        super().update()

        self.horizontal_orientation = self.horizontal_orientation_in_robot + robot_orientation
        
        # Do evey n steps
        if self.step_counter.check():
            # Extract image from buffer
            image_data = self.device.getImage()
            self.image.image = np.array(np.frombuffer(image_data, np.uint8).reshape((self.height, self.width, 4)))

            if self.rotate180:
                self.image.image = np.rot90(self.image.image, 2, (0, 1))

            self.image.orientation = self.horizontal_orientation

            self.image.data = self.get_data()

            