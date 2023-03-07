import numpy as np

# Captures images and processes them
class Camera:
    def __init__(self, camera, timeStep):
        self.rotate180 = False
        self.camera = camera
        self.camera.enable(timeStep)
        self.height = self.camera.getHeight()
        self.width = self.camera.getWidth()

    # Gets an image from the raw camera data
    def getImg(self):
        imageData = self.camera.getImage()
        camImg = np.array(np.frombuffer(imageData, np.uint8).reshape((self.height, self.width, 4)))
        if self.rotate180:
            camImg = np.rot90(camImg, 2, (0, 1))
        return camImg