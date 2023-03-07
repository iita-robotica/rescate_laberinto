import utilities
import math
import copy

# Returns a point cloud of the detctions it makes
class Lidar():
    def __init__(self, device, timeStep, pointIsCloseThresh, pointIsCloseRange):
        self.device = device
        self.device.enable(timeStep)
        self.x = 0
        self.y = 0
        self.z = 0
        self.rotation = 0
        self.fov = device.getFov()
        self.verticalFov = self.device.getVerticalFov()
        self.horizontalRes = self.device.getHorizontalResolution()
        self.verticalRes = self.device.getNumberOfLayers()
        self.hRadPerDetection = self.fov / self.horizontalRes
        self.vRadPerDetection = self.verticalFov / self.verticalRes
        self.detectRotOffset = 0  # math.pi * 0.75
        self.maxDetectionDistance = 0.06 * 8
        self.minDetectionDistance = 0.06 * 0.6
        self.pointIsClose = False
        self.pointIsCloseThresh = pointIsCloseThresh
        self.pointIsCloseRange = pointIsCloseRange
        self.distBias = 0.06 * 0.2
        self.distCoeff = 1
        self.distFactor = 1 #0.8
    
    def getRotationsAndDistances(self, layers=range(3)):
        self.pointIsClose = False
        
        # (degsToRads(359 - radsToDegs(self.rotation)))
        # rangeImage = self.device.getRangeImageArray()
        # print("Lidar vFov: ", self.verticalFov/ self.verticalRes)

        rots = []
        distances = []
        
        for layer in layers:
            actualVDetectionRot = (layer * self.vRadPerDetection) + self.verticalFov / 2
            depthArray = self.device.getLayerRangeImage(layer)
            actualHDetectionRot = self.detectRotOffset + ((2 * math.pi) - self.rotation)
            for item in depthArray:
                if self.minDetectionDistance <= item:# <= self.maxDetectionDistance:

                    if item == float("inf") or item == float("inf") * -1:
                        item = 0.5
                    x = item * math.cos(actualVDetectionRot)
                    
                    x += self.distBias
                    x *= self.distCoeff
                    x = x ** self.distFactor


                    if utilities.degsToRads(self.pointIsCloseRange[0]) > actualHDetectionRot > utilities.degsToRads(self.pointIsCloseRange[1]) and x < self.pointIsCloseThresh:
                        self.pointIsClose = True

                    rots.append(actualHDetectionRot)
                    distances.append(x)
                actualHDetectionRot += self.hRadPerDetection
        return rots, distances


    # Does a detection pass and returns a point cloud with the results
    @utilities.do_every_n_frames(5, 32)
    def getPointCloud(self, layers=range(3)):

        def chunks(lst, n):
            """Yield successive n-sized chunks from lst."""
            for i in range(0, len(lst), n):
                yield lst[i:i + n]

        self.pointIsClose = False
        
        # (degsToRads(359 - radsToDegs(self.rotation)))
        # rangeImage = self.device.getRangeImageArray()
        # print("Lidar vFov: ", self.verticalFov/ self.verticalRes)

        pointCloud = []

        outOfBounds = []
        total_depth_array = self.device.getRangeImage()


        total_depth_array = chunks(total_depth_array, self.horizontalRes)
        #print(total_depth_array)
        for layer, depthArray in enumerate(total_depth_array):
            #print("DEPTH ARRAY: ", depthArray)
            if layer in layers:
                actualVDetectionRot = (layer * self.vRadPerDetection) + self.verticalFov / 2
                #depthArray = self.device.getLayerRangeImage(layer)
                actualHDetectionRot = self.detectRotOffset + ((2 * math.pi) - self.rotation)
                for item in depthArray:
                    
                    if item >= self.maxDetectionDistance or item == float("inf"):
                        x = 10 * math.cos(actualVDetectionRot)
                        x += self.distBias
                        x *= self.distCoeff
                        x = x ** self.distFactor

                        coords = utilities.getCoordsFromRads(actualHDetectionRot, x)
                        outOfBounds.append([coords[0] - 0, (coords[1] * -1) - 0])

                    else:
                        if item >= self.minDetectionDistance:
                                #item = self.maxDetectionDistance
                                if item != float("inf") and item != float("inf") * -1 and item != 0:
                                    x = item * math.cos(actualVDetectionRot)
                                    x += self.distBias
                                    x *= self.distCoeff
                                    x = x ** self.distFactor

                                    if utilities.degsToRads(self.pointIsCloseRange[0]) > actualHDetectionRot > utilities.degsToRads(self.pointIsCloseRange[1]) and x < self.pointIsCloseThresh:
                                        self.pointIsClose = True

                                    coords = utilities.getCoordsFromRads(actualHDetectionRot, x)
                                    pointCloud.append([coords[0] - 0, (coords[1] * -1) - 0])

                    actualHDetectionRot += self.hRadPerDetection
        
        if len(outOfBounds) == 0:
            outOfBounds = [[0, 0]]
        
        if len(pointCloud) == 0:
            pointCloud = [[0, 0]]

        return pointCloud, outOfBounds

    # Sets the rotation of the sensors in radians
    def setRotationRadians(self, rads):
        self.rotation = rads
    
    # Sets the rotation of the sensors in degrees
    def setRotationDegrees(self, degs):
        self.rotation = utilities.degsToRads(degs)