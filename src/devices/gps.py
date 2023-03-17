import utilities

from data_structures.vectors import Position2D

# Tracks global position
class Gps:
    def __init__(self, gps, timeStep, coordsMultiplier=1):
        self.gps = gps
        self.gps.enable(timeStep)
        self.multiplier = coordsMultiplier
        self.__prevPosition = []
        self.position = self.getPosition()

    # updates gps, must run every timestep
    def update(self):
        self.__prevPosition = self.position
        self.position = self.getPosition()

    # Returns the global position
    def getPosition(self):
        vals = self.gps.getValues()
        return Position2D(vals[0] * self.multiplier, vals[2] * self.multiplier)

    # Returns the global rotation according to gps
    def getRotation(self):
        if self.__prevPosition != self.position:
            posDiff = ((self.position[0] - self.__prevPosition[0]), (self.position[1] - self.__prevPosition[1]))
            accuracy = utilities.getDistance(posDiff)
            if accuracy > 0.001:
                degs = utilities.getDegsFromCoords(posDiff)
                return utilities.normalizeDegs(degs)
        return None