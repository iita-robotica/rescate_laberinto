import utilities
from data_structures.angle import Angle

# Tracks global rotation
class Gyroscope:
    def __init__(self, gyro, index, timeStep):
        self.sensor = gyro
        self.sensor.enable(timeStep)
        self.oldTime = 0.0
        self.index = index
        self.rotation = Angle(0)
        self.lastRads = Angle(0)

    # Do on every timestep
    def update(self, time):
        timeElapsed = time - self.oldTime  # Time passed in time step
        radsInTimestep = Angle((self.sensor.getValues())[self.index] * timeElapsed)
        self.lastRads = radsInTimestep
        self.rotation += radsInTimestep
        self.rotation.normalize()
        self.oldTime = time

    # Gets the actual angular Velocity
    def getDiff(self):
        if self.lastRads < 0:
            return self.lastRads * -1
        
        return self.lastRads

    def get_angle(self):
        return self.rotation
    
    def set_angle(self, angle):
        self.rotation = angle