class Device:
    def __init__(self, device, realtivePosition, timeStep):
        self.device = device
        self.relativePosition = [0, 0, 0]
        self.robotPosition = [0, 0, 0]
        self.relativeRotation = 0
        self.robotRotation = 0
        
        self.device.enable(timeStep)

    @property
    def globalPosition(self):
        pos = []
        for i, j in zip(self.relativePosition, self.robotPosition):
            pos.append(i + j)
        return pos

    @property
    def globalRotation(self):
        return self.relativeRotation + self.globalRotation