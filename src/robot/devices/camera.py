import numpy as np
from robot.devices.sensor import TimedSensor
import cv2 as cv

from flow_control.step_counter import StepCounter

from data_structures.angle import Angle

class CameraImage:
    def __init__(self) -> None:
        self.image = None
        self.orientation = Angle(0)

# Captures images and processes them
class Camera(TimedSensor):
    def __init__(self, webots_device, time_step, step_counter: StepCounter, orientation: Angle, distance_from_center: float, rotate180=False):
        super().__init__(webots_device, time_step, step_counter)
        self.rotate180 = rotate180
        self.height = self.device.getHeight()
        self.width = self.device.getWidth()
        self.image = CameraImage()
        self.orientation_in_robot = orientation
        self.orientation = orientation
        self.distance_from_center = distance_from_center

    # Returns the camera image
    def get_image(self):
        if self.step_counter.check():
            return self.image
    
    def update(self, robot_orientation: Angle):
        super().update()

        self.orientation = self.orientation_in_robot + robot_orientation
        
        # Do evey n steps
        if self.step_counter.check():
            # Extract image from buffer
            image_data = self.device.getImage()
            self.image.image = np.array(np.frombuffer(image_data, np.uint8).reshape((self.height, self.width, 4)))

            if self.rotate180:
                self.image.image = np.rot90(self.image.image, 2, (0, 1))

            self.image.orientation = self.orientation

            