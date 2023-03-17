import utilities

class DistanceSensor:
    def __init__(self, threshold, distanceFromCenter, angle, sensor, timeStep):
        self.sensor = sensor
        self.angle = angle
        self.distance = distanceFromCenter
        self.timeStep = timeStep
        self.threshold = threshold
        self.position = [0, 0]
        self.sensor.enable(self.timeStep)

    def isFar(self):
        distance = self.sensor.getValue()

        return distance > self.threshold

    def setPosition(self, robotPosition, robotRotation):
        sensorRotation = robotRotation + self.angle
        sensorPosition = utilities.getCoordsFromDegs(sensorRotation, self.distance)
        self.position = utilities.sumLists(sensorPosition, robotPosition)

class FrontDistanceSensor():
    def __init__(self, sensor, threshold, timeStep, offset=0):
        self.sensor = sensor
        self.timeStep = timeStep
        self.sensor.enable(self.timeStep)
        self.threshold = threshold
        self.offset = offset

    def getValue(self):
        return self.sensor.getValue() - self.offset

    def isClose(self):
        return self.getValue() < self.threshold