""" Class for the camera """
import numpy as np
class Camera:
    """ Gets an image from the raw camera data
    """
    def __init__(self, camera, time_step):
        """ Constructor of the class

        Args:
            camera (Camera): _description_
            timeStep (int): _description_
        Returns:
            _type_: _description_
        """
        self.camera = camera
        self.camera.enable(time_step)
        self.height = self.camera.getHeight()
        self.width = self.camera.getWidth()

    def get_img(self):
        """"
        Gets an image from the camera

        Args:
            self (Camera): _description_
        Returns:
            np.array() : _description_
        """
        image_data = self.camera.getImage()
        return np.array(np.frombuffer(image_data, np.uint8).reshape((self.height, self.width, 4)))
    