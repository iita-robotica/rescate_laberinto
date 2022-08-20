""" Class for the colour sensor """
import utilities
class ColourSensor:
    def __init__(self, sensor, distancefromCenter, timeStep):
        self.distance = distancefromCenter
        self.position = [0, 0]
        self.sensor = sensor
        self.sensor.enable(timeStep)
        self.r = 0
        self.g = 0
        self.b = 0
    def setPosition(self, robotGlobalPosition, robotGlobalRotation):
        realPosition = utilities.getCoordsFromDegs(robotGlobalRotation, self.distance)
        self.position = [robotGlobalPosition[0] + realPosition[0], robotGlobalPosition[1] + realPosition[1]]
    def __update(self):
        colour = self.sensor.getImage()
        # print("Colourimg:", colour)
        self.r = self.sensor.imageGetRed(colour, 1, 0, 0)
        self.g = self.sensor.imageGetGreen(colour, 1, 0, 0)
        self.b = self.sensor.imageGetBlue(colour, 1, 0, 0)
        # print("Colour:", self.r, self.g, self.b)
    
    def __isTrap(self):
        return (35 < self.r < 45 and 35 < self.g < 45)

    def __isSwamp(self):
        return (200 < self.r < 210 and 165 < self.g < 175 and 95 < self.b < 105)

    def __isCheckpoint(self):
        return (self.r > 232 and self.g > 232 and self.b > 232)

    def __isNormal(self):
        return self.r == 227 and self.g == 227

    def __isBlue(self):
        return (55 < self.r < 65 and 55 < self.g < 65 and 245 < self.b < 255)

    def __isPurple(self):
        return (135 < self.r < 145 and 55 < self.g < 65 and 215 < self.b < 225)

    def __isRed(self):
        return (245 < self.r < 255 and 55 < self.g < 65 and 55 < self.b < 65)

    # Returns the type of tyle detected from the colour data
    def getTileType(self):
        self.__update()
        tileType = "undefined"
        if self.__isNormal():
            tileType = "normal"
        elif self.__isTrap():
            tileType = "hole"
        elif self.__isSwamp():
            tileType = "swamp"
        elif self.__isCheckpoint():
            tileType = "checkpoint"
        elif self.__isBlue():
            tileType = "connection1-2"
        elif self.__isPurple():
            tileType = "connection2-3"
        elif self.__isRed():
            tileType = "connection1-3"

        # print("Color: " + tileType)
        # print("r: " + str(self.r) + "g: " + str(self.g) + "b: " +  str(self.b))
        return tileType
