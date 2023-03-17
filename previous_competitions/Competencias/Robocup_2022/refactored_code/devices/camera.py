import numpy as np
from devices.sensor import TimedSensor

# Captures images and processes them
class Camera(TimedSensor):
    def __init__(self, webots_device, time_step, step_counter, rotate180=False):
        super().__init__(webots_device, time_step, step_counter)
        self.rotate180 = rotate180
        self.height = self.device.getHeight()
        self.width = self.device.getWidth()
        self.image = None

    # Returns the camera image
    def get_image(self):
        if self.step_counter.check():
            return self.image
    
    def update(self):
        super().update()
        
        # Do evey n steps
        if self.step_counter.check():
            # Extract image from buffer
            image_data = self.device.getImage()
            self.image = np.array(np.frombuffer(image_data, np.uint8).reshape((self.height, self.width, 4)))
            if self.rotate180:
                self.image = np.rot90(self.image, 2, (0, 1))